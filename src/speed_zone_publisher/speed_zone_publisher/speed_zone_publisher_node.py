#!/usr/bin/env python3
"""自己位置(TF map -> base_link)とエリア定義(zones.yaml)から、今の区間の目標速度を
/target_speed (std_msgs/Float32) に 50Hz で出す。加減速の制限は my_mppi_controller 側が持つ。
TF が取れないときは出さない(MPPI 側がタイムアウトで止まる)。エリアは Marker で表示する。
"""

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from speed_zone_publisher.zones import load_zones, target_speed


class SpeedZonePublisher(Node):
    def __init__(self):
        super().__init__("speed_zone_publisher")
        self.declare_parameter("zones_yaml", "")
        self.default_speed, self.zones = load_zones(self.get_parameter("zones_yaml").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_speed = self.create_publisher(Float32, "/target_speed", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/speed_zone_publisher/markers", 1)
        self.create_timer(1.0 / 50.0, self.on_timer)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            f"エリア {len(self.zones)} 個 / default_speed {self.default_speed} m/s")

    def on_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.0))
        except TransformException as e:
            self.get_logger().warn(f"TF map->base_link 待ち: {e}", throttle_duration_sec=2.0)
            return
        t = tf.transform.translation
        speed = target_speed(self.default_speed, self.zones, t.x, t.y)
        self.pub_speed.publish(Float32(data=float(speed)))

    def publish_markers(self):
        arr = MarkerArray()
        for i, z in enumerate(self.zones):
            line = Marker()
            line.header.frame_id = "map"
            line.ns = "zones"
            line.id = 2 * i
            line.type = Marker.LINE_STRIP
            line.scale.x = 0.03
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.4, 0.0, 1.0
            line.pose.orientation.w = 1.0
            line.points = [Point(x=x, y=y) for x, y in z.polygon + z.polygon[:1]]
            text = Marker()
            text.header.frame_id = "map"
            text.ns = "zone_names"
            text.id = 2 * i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.scale.z = 0.2
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.pose.position.x = sum(p[0] for p in z.polygon) / len(z.polygon)
            text.pose.position.y = sum(p[1] for p in z.polygon) / len(z.polygon)
            text.pose.orientation.w = 1.0
            text.text = f"{z.name} {z.speed:.1f}"
            arr.markers += [line, text]
        self.pub_markers.publish(arr)


def main() -> None:
    rclpy.init()
    node = SpeedZonePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
