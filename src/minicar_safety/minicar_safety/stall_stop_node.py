#!/usr/bin/env python3
"""停止検知フィルタ — 指令チェーンの最終段 (実機用)。

    controller_server --/cmd_vel_raw--> [stall_stop_node] --/cmd_vel--> [motor_driver]
                                               ^
                                   /odom (micro-ROS 車輪エンコーダ)

ふだんは指令を素通しする。前進を指令しているのに /odom の実速度が 0 の
状態が stall_time 続いたら、以後ゼロ指令に差し替えてラッチする。
壁や芝生で車輪ごと止まると、速度 PI は出力上限 (345 カウント) に
張り付いたまま押し続ける (2026-10-19 の bag で 1〜10s が 15 回)。
それを止めるのが目的。

判定の中身は stall_stop.py。空転 (車輪は回っているが車体は止まっている)
は検知しない。

解除は reset サービス。解除しても、再び走り出すまでは判定しない。

    ros2 service call /stall_stop_node/reset std_srvs/srv/Trigger

指令が途絶えたとき・/odom が途絶えたときの停止はここでは持たない。
どちらも motor_driver が自分で止める (cmd_timeout / odom_lost_stop)。
このノードが落ちても、motor_driver は /cmd_vel が途絶えて中立になる。

既存の safety_node (シム用: スキャンと IMU で判定し後退で脱出) とは
同時に起動しないこと。どちらも /cmd_vel に publish する。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from minicar_safety.stall_stop import StallConfig, StallDetector


class StallStopNode(Node):
    def __init__(self):
        super().__init__("stall_stop_node")

        self.declare_parameter("in_topic", "/cmd_vel_raw")
        self.declare_parameter("out_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        defaults = StallConfig()
        for name in ("cmd_min", "speed_zero", "stall_time", "arm_speed", "odom_timeout"):
            self.declare_parameter(name, getattr(defaults, name))

        self.det = StallDetector(StallConfig(**{
            name: float(self.get_parameter(name).value)
            for name in ("cmd_min", "speed_zero", "stall_time", "arm_speed", "odom_timeout")
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
        # 停止中か。transient local なので、あとから echo しても今の状態が見える。
        self.pub_state = self.create_publisher(
            Bool, "/safety/stall_stop",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, "~/reset", self.on_reset)
        self.pub_state.publish(Bool(data=False))

        c = self.det.cfg
        self.get_logger().info(
            f"stall_stop_node started. {self.get_parameter('in_topic').value} -> "
            f"{self.get_parameter('out_topic').value}  "
            f"指令 > {c.cmd_min} m/s で実速 < {c.speed_zero} m/s が {c.stall_time}s 続いたら停止 "
            f"(走り出して {c.arm_speed} m/s を超えるまでは判定しない)")

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
        was_tripped = self.det.tripped
        now = self._now()
        if not self.det.update(msg.linear.x, now):
            self.pub.publish(msg)
            return

        # ゼロ指令。motor_driver は |v| < v_eps で舵を保持し、スロットルは中立。
        self.pub.publish(Twist())
        if not was_tripped:
            self.pub_state.publish(Bool(data=True))
            self.get_logger().error(
                f"STALL: 指令 v={msg.linear.x:+.2f} m/s なのに実速 {self.det.v_odom:+.3f} m/s が "
                f"{self.det.stalled_for(now):.1f}s 続いた。停止してラッチする。"
                f"車を戻したら: ros2 service call {self.get_fully_qualified_name()}/reset "
                f"std_srvs/srv/Trigger")
        else:
            self.get_logger().warn(
                "STALL で停止中 (reset サービスで解除)", throttle_duration_sec=2.0)

    def on_reset(self, request, response):
        was = self.det.tripped
        self.det.reset()
        self._armed_logged = False
        self.pub_state.publish(Bool(data=False))
        response.success = True
        response.message = ("解除した" if was else "停止していなかった") + "。走り出すまで判定しない"
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
