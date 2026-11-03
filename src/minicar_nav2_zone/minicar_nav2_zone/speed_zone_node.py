#!/usr/bin/env python3
"""Speed Zone — 自己位置に応じて MPPI の速度帯(vx_min..vx_max)を切り替える。

    TF map->base_link -> [speed_zone] --SetParametersAtomically-->
        controller_server (FollowPath.vx_max / FollowPath.vx_min)

ヘアピン(slow_zones の矩形)の中では slow_vx_min..slow_vx_max、それ以外では
fast_vx_min..fast_vx_max で走らせる。経路の形は raceline_manager がこれまでどおり
送り、このノードは速度帯だけを触る。

=== なぜ動的パラメータで切り替えるか ===

Humble の MPPI は FollowPath.vx_max / vx_min を動的パラメータとして受け付け、
変わると Optimizer を reset する(制御列をゼロに戻す)。
controller_server の /speed_limit は vx_min と wz_max も同じ比率で縮めるので使わない
(3.0 -> 2.0 だと wz_max も 2/3 になり、v 2.0 / r_min 0.83 に要る wz 2.4 が出せず
ヘアピンで曲がれない)。

vx_max と vx_min は SetParametersAtomically で 1 回に送る。別々に送ると
reset が 2 回走り、途中で vx_min > vx_max の瞬間もできる。

=== 自己位置 ===

raceline_manager と同じく TF map -> base_link を引く。
"""

import math

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from minicar_nav2_zone.speed_zone import ZoneSwitch, is_slow, parse_zones


def _double(name, value):
    return Parameter(name=name, value=ParameterValue(
        type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))


class SpeedZoneNode(Node):
    def __init__(self):
        super().__init__("speed_zone")

        self.declare_parameter("fast_vx_max", 3.0)
        self.declare_parameter("fast_vx_min", 2.5)
        self.declare_parameter("slow_vx_max", 2.0)
        self.declare_parameter("slow_vx_min", 1.5)
        self.declare_parameter("slow_zones", [0.0])
        self.declare_parameter("preview_distance", 1.25)
        self.declare_parameter("min_hold_s", 0.5)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("controller_server", "/controller_server")
        self.declare_parameter("controller_id", "FollowPath")

        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.bands = {
            False: (float(g("fast_vx_min")), float(g("fast_vx_max"))),
            True: (float(g("slow_vx_min")), float(g("slow_vx_max"))),
        }
        for slow, (lo, hi) in self.bands.items():
            if not lo <= hi:
                raise ValueError(
                    f"{'slow' if slow else 'fast'} の速度帯は vx_min <= vx_max であること: "
                    f"{lo}, {hi}")
        self.zones = parse_zones(g("slow_zones"))
        if not self.zones:
            raise ValueError("slow_zones が空(4 個ずつ [x_min, x_max, y_min, y_max])")
        self.preview = float(g("preview_distance"))
        self.map_frame = str(g("map_frame"))
        self.base_frame = str(g("base_frame"))
        self.controller_id = str(g("controller_id"))
        self.switch = ZoneSwitch(float(g("min_hold_s")))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        server = str(g("controller_server")).rstrip("/")
        self.client = self.create_client(
            SetParametersAtomically, f"{server}/set_parameters_atomically")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_slow = self.create_publisher(Bool, "/speed_zone/slow", latched)
        self.pub_markers = self.create_publisher(
            MarkerArray, "/speed_zone/markers", latched)
        self._publish_markers()

        self._applied = None     # controller_server に反映済みの状態
        self._pending = None     # 応答待ちの future

        self.create_timer(1.0 / float(g("rate_hz")), self.on_timer)

        fast, slow = self.bands[False], self.bands[True]
        self.get_logger().info(
            f"speed_zone started: 低速域 {len(self.zones)} 個 / "
            f"fast vx {fast[0]}..{fast[1]} / slow vx {slow[0]}..{slow[1]} / "
            f"先読み {self.preview}m / -> {server}")

    # ------------------------------------------------------------------
    def _pose(self):
        """map 座標の (x, y, yaw)。TF がまだ無ければ None。"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.0))
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.map_frame}->{self.base_frame} 待ち: {e}",
                throttle_duration_sec=2.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def on_timer(self):
        pose = self._pose()
        if pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.switch.update(is_slow(self.zones, *pose, self.preview), now)
        if self.switch.slow != self._applied and self._pending is None:
            self._send(self.switch.slow)

    def _send(self, slow):
        if not self.client.service_is_ready():
            self.get_logger().info(
                "controller_server の set_parameters_atomically を待っている",
                throttle_duration_sec=2.0)
            return
        lo, hi = self.bands[slow]
        req = SetParametersAtomically.Request()
        req.parameters = [_double(f"{self.controller_id}.vx_max", hi),
                          _double(f"{self.controller_id}.vx_min", lo)]
        self._pending = self.client.call_async(req)
        self._pending.add_done_callback(lambda f: self._on_done(f, slow))

    def _on_done(self, future, slow):
        self._pending = None
        try:
            result = future.result().result
        except Exception as e:  # noqa: BLE001  応答が取れなければ次周期に再送
            self.get_logger().error(f"速度帯の設定に失敗(再送する): {e}")
            return
        if not result.successful:
            self.get_logger().error(
                f"速度帯の設定を controller_server が拒否(再送する): {result.reason}",
                throttle_duration_sec=2.0)
            return
        self._applied = slow
        lo, hi = self.bands[slow]
        self.get_logger().info(
            f"[ZONE] {'slow' if slow else 'fast'} vx {lo}..{hi}")
        self.pub_slow.publish(Bool(data=slow))

    def _publish_markers(self):
        arr = MarkerArray()
        for i, r in enumerate(self.zones):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.ns = "slow_zones"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = 0.5 * (r.x_min + r.x_max)
            m.pose.position.y = 0.5 * (r.y_min + r.y_max)
            m.pose.orientation.w = 1.0
            m.scale.x = r.x_max - r.x_min
            m.scale.y = r.y_max - r.y_min
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 0.3
            arr.markers.append(m)
        self.pub_markers.publish(arr)


def main() -> None:
    rclpy.init()
    node = SpeedZoneNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # rclpy は SIGTERM を ExternalShutdownException に変えて投げてくるので、
        # 拾わないと終了のたびに traceback が出る。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
