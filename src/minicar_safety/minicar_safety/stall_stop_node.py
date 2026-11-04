#!/usr/bin/env python3
"""停止検知と後退復帰のフィルタ — 指令チェーンの最終段 (実機用)。

    controller_server --/cmd_vel_raw--> [stall_stop_node] --/cmd_vel--> [motor_driver]
                                               ^
                                   /odom (micro-ROS 車輪エンコーダ)

ふだんは指令を素通しする。前進を指令しているのに /odom の実速度が 0 の
状態が stall_time 続いたら、recovery_duration だけ後退してから素通しへ戻る。
壁や芝生で車輪ごと止まると、速度 PI は出力上限 (345 カウント) に
張り付いたまま押し続ける (2026-10-19 の bag で 1〜10s が 15 回)。
それを止めて抜け出すのが目的。

壁に密着した車が前進しながら曲がるのは幾何的に無理なので、MPPI の指令を
聞き続けても抜けられない。後退時は切り返しと同じく舵を逆に切る。

後退しても前進できないまま max_recoveries 回続いたら、停止してラッチする。
解除は reset サービス。解除しても、再び走り出すまでは判定しない。

    ros2 service call /stall_stop_node/reset std_srvs/srv/Trigger

判定の中身は stall_stop.py。空転 (車輪は回っているが車体は止まっている)
は検知しない。

指令が途絶えたとき・/odom が途絶えたときの停止はここでは持たない。
どちらも motor_driver が自分で止める (cmd_timeout / odom_lost_stop)。
後退中も /cmd_vel_raw を受けるたびに後退指令を出すので、上流が止まれば
後退も止まる。このノードが落ちても、motor_driver は /cmd_vel が途絶えて中立になる。

既存の safety_node (シム用: スキャンと IMU で判定) とは同時に起動しないこと。
どちらも /cmd_vel に publish する。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from minicar_safety.stall_stop import PASS, RECOVER, STOP, StallConfig, StallDetector

PARAMS = ("cmd_min", "speed_zero", "stall_time", "arm_speed", "odom_timeout",
          "recovery_speed", "recovery_duration", "max_recoveries")


class StallStopNode(Node):
    def __init__(self):
        super().__init__("stall_stop_node")

        self.declare_parameter("in_topic", "/cmd_vel_raw")
        self.declare_parameter("out_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        defaults = StallConfig()
        for name in PARAMS:
            self.declare_parameter(name, getattr(defaults, name))

        self.det = StallDetector(StallConfig(**{
            name: type(getattr(defaults, name))(self.get_parameter(name).value)
            for name in PARAMS
        }))
        self._armed_logged = False

        self.create_subscription(
            Twist, self.get_parameter("in_topic").value, self.on_cmd, 10)
        # motor_driver と同じく sensor_data (best effort)。micro-ROS の
        # publisher が best effort でも黙って届かなくならない。
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(
            Twist, self.get_parameter("out_topic").value, 10)
        # "pass" / "recover" / "stop"。transient local なので、あとから echo しても
        # 今の状態が見える。
        self.pub_state = self.create_publisher(
            String, "/safety/stall_state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.pub_state.publish(String(data=PASS))

        c = self.det.cfg
        self.get_logger().info(
            f"stall_stop_node started. {self.get_parameter('in_topic').value} -> "
            f"{self.get_parameter('out_topic').value}  "
            f"指令 > {c.cmd_min} m/s で実速 < {c.speed_zero} m/s が {c.stall_time}s 続いたら "
            f"{c.recovery_speed} m/s で {c.recovery_duration}s 後退 "
            f"(続けて {c.max_recoveries} 回で停止。走り出して {c.arm_speed} m/s を超えるまでは判定しない)")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry) -> None:
        v = float(msg.twist.twist.linear.x)
        if not math.isfinite(v):
            return
        self.det.update_odom(v, self._now())
        if self.det.armed and not self._armed_logged:
            self._armed_logged = True
            self.get_logger().info(f"走り出した (実速 {v:+.2f} m/s)。停止検知を開始")

    def on_cmd(self, msg: Twist) -> None:
        before = self.det.mode
        out = self.det.update(msg.linear.x, msg.angular.z, self._now())
        if out is None:
            self.pub.publish(msg)
        else:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = out
            self.pub.publish(cmd)

        mode = self.det.mode
        if mode == before:
            if mode == STOP:
                self.get_logger().warn(
                    "STALL で停止中 (reset サービスで解除)", throttle_duration_sec=2.0)
            return
        self.pub_state.publish(String(data=mode))
        c = self.det.cfg
        if mode == RECOVER:
            self.get_logger().error(
                f"STALL: 指令 v={msg.linear.x:+.2f} omega={msg.angular.z:+.3f} なのに "
                f"実速 {self.det.v_odom:+.3f} m/s が {c.stall_time}s 続いた。"
                f"{c.recovery_duration}s 後退する (kappa {self.det.recover_kappa:+.2f}、"
                f"{self.det.recoveries}/{c.max_recoveries} 回目)")
        elif mode == STOP:
            self.get_logger().error(
                f"STALL: 後退を {c.max_recoveries} 回続けても前進できない。停止してラッチする。"
                f"車を戻したら: ros2 service call {self.get_fully_qualified_name()}/reset "
                f"std_srvs/srv/Trigger")
        elif mode == PASS:
            self.get_logger().info("後退を終了、素通しへ復帰")

    def on_reset(self, request, response):
        was = self.det.mode
        self.det.reset()
        self._armed_logged = False
        self.pub_state.publish(String(data=PASS))
        response.success = True
        response.message = ("解除した" if was == STOP else f"{was} だった") + "。走り出すまで判定しない"
        self.get_logger().info(f"reset: {response.message}")
        return response

    def stop(self) -> None:
        self.pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = StallStopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
