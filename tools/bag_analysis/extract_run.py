#!/usr/bin/env python3
"""実走bagから自己位置・経路の分析用データを npz に抜き出す(コンテナ内で実行)。

    python3 /tools/bag_analysis/extract_run.py /bags/<bag> /bags/<bag>/analysis/run.npz
"""
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/scan', '/odom', '/odometry/filtered', '/tf', '/pose', '/map',
          '/transformed_global_plan', '/cmd_vel_raw', '/raceline/lap']
TF_PAIRS = {('map', 'odom'): 'tf_map_odom', ('odom', 'base_link'): 'tf_odom_base'}


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def odom_row(m):
    p = m.pose.pose
    return (stamp(m.header), p.position.x, p.position.y, yaw(p.orientation),
            m.twist.twist.linear.x, m.twist.twist.angular.z)


def main():
    bag, out = sys.argv[1], Path(sys.argv[2])
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    d = {k: [] for k in ('scan_t', 'ranges', 'odom', 'ekf', 'pose', 'cmd', 'lap', *TF_PAIRS.values())}
    path_t, path_frame, path_n, path_xy = [], [], [], []
    tf_other = set()
    angle_min = angle_inc = None
    last_map = None
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        if topic == '/scan':
            angle_min, angle_inc = m.angle_min, m.angle_increment
            d['scan_t'].append(stamp(m.header))
            d['ranges'].append(np.asarray(m.ranges, dtype=np.float32))
        elif topic == '/odom':
            d['odom'].append(odom_row(m))
        elif topic == '/odometry/filtered':
            d['ekf'].append(odom_row(m))
        elif topic == '/tf':
            for tr in m.transforms:
                key = TF_PAIRS.get((tr.header.frame_id, tr.child_frame_id))
                if key is None:
                    tf_other.add((tr.header.frame_id, tr.child_frame_id))
                    continue
                v = tr.transform.translation
                d[key].append((stamp(tr.header), v.x, v.y, yaw(tr.transform.rotation)))
        elif topic == '/pose':
            p = m.pose.pose
            d['pose'].append((stamp(m.header), p.position.x, p.position.y, yaw(p.orientation)))
        elif topic == '/map':
            last_map = m
        elif topic == '/transformed_global_plan':
            if not m.poses:
                continue
            path_t.append(stamp(m.header) if m.header.stamp.sec else t * 1e-9)
            path_frame.append(m.header.frame_id)
            path_n.append(len(m.poses))
            path_xy.extend((p.pose.position.x, p.pose.position.y) for p in m.poses)
        elif topic == '/cmd_vel_raw':
            d['cmd'].append((t * 1e-9, m.linear.x, m.angular.z))
        elif topic == '/raceline/lap':
            d['lap'].append((t * 1e-9, m.data))
    info = last_map.info
    grid = np.asarray(last_map.data, dtype=np.int8).reshape(info.height, info.width)
    o = info.origin
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out, scan_t=np.array(d['scan_t']), ranges=np.stack(d['ranges']), angle_min=angle_min, angle_inc=angle_inc,
        odom=np.array(d['odom']), ekf=np.array(d['ekf']), pose=np.array(d['pose']).reshape(-1, 4),
        tf_map_odom=np.array(d['tf_map_odom']).reshape(-1, 4), tf_odom_base=np.array(d['tf_odom_base']).reshape(-1, 4),
        cmd=np.array(d['cmd']).reshape(-1, 3), lap=np.array(d['lap']).reshape(-1, 2),
        path_t=np.array(path_t), path_frame=np.array(path_frame), path_n=np.array(path_n, dtype=int),
        path_xy=np.array(path_xy).reshape(-1, 2),
        map=grid, map_res=info.resolution, map_origin=np.array([o.position.x, o.position.y, yaw(o.orientation)]))
    for k, v in d.items():
        if k != 'ranges':
            print(f'{k}: {len(v)}')
    print(f'path: {len(path_t)} msgs, frames {sorted(set(path_frame))}')
    print(f'map: {info.width}x{info.height} res {info.resolution} origin ({o.position.x:.2f}, {o.position.y:.2f})')
    print(f'other tf: {sorted(tf_other)}')


if __name__ == '__main__':
    main()
