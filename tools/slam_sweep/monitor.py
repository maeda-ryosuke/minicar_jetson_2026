#!/usr/bin/env python3
"""sim time 1秒ごとに map->odom / map->base_link / odom->base_link をcsvへ記録する。"""
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
import tf2_ros


def _pose(t):
    q, p = t.transform.rotation, t.transform.translation
    return p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Node('sweep_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    out = open(sys.argv[1], 'w', newline='')
    w = csv.writer(out)
    w.writerow(['t', 'mo_x', 'mo_y', 'mo_yaw', 'mb_x', 'mb_y', 'mb_yaw', 'ob_x', 'ob_y', 'ob_yaw'])
    last = [None]

    def tick():
        now = node.get_clock().now().nanoseconds * 1e-9
        if now == 0 or (last[0] is not None and now - last[0] < 1.0):
            return
        try:
            row = [now]
            for parent, child in (('map', 'odom'), ('map', 'base_link'), ('odom', 'base_link')):
                row += _pose(buf.lookup_transform(parent, child, Time()))
        except tf2_ros.TransformException:
            return
        last[0] = now
        w.writerow([f'{v:.4f}' for v in row])
        out.flush()

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    out.close()


if __name__ == '__main__':
    main()
