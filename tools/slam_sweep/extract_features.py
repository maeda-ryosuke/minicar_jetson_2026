#!/usr/bin/env python3
"""bagから原因分析用の時系列を npz に抜き出す(コンテナ内で実行)。

    python3 /tools/slam_sweep/extract_features.py /bags/<bag> /replay_params/sweep/features.npz
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    bag, out = sys.argv[1], sys.argv[2]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/odom', '/odometry/filtered']))
    scan_t, ranges, odom_t, ekf = [], [], [], []
    angle_min = angle_inc = None
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if topic == '/scan':
            angle_min, angle_inc = m.angle_min, m.angle_increment
            scan_t.append(stamp)
            ranges.append(np.asarray(m.ranges, dtype=np.float32))
        elif topic == '/odom':
            odom_t.append(t * 1e-9)
        else:
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ekf.append((stamp, m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                        m.twist.twist.linear.x, m.twist.twist.angular.z))
    np.savez_compressed(out, scan_t=np.array(scan_t), ranges=np.stack(ranges),
                        angle_min=angle_min, angle_inc=angle_inc,
                        odom_recv_t=np.array(odom_t), ekf=np.array(ekf))


if __name__ == '__main__':
    main()
