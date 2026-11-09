#!/usr/bin/env python3
"""元の bag と再生記録(run_replay.sh の replay_bag)から、解析用のデータを npz に抜き出す
(replay コンテナ内で実行)。図とレポートはホストの analyze.py が作る。

    python3 /tools/bag_analysis/bicycle_replay/extract.py /bags/20261105_fast4

出力: <bag>/analysis/bicycle/data.npz
  - 自己位置: map->odom と odom->base_link(元の bag の TF)
  - 速度: /odometry/filtered、元の MPPI の /cmd_vel_raw、新しい MPPI の /cmd_vel_raw、/target_speed
  - 軌道: 新旧の /trajectories(最適軌道と候補軌道、odom 座標)、新しい /transformed_global_plan
  - 地図: /map
時刻はすべて bag の時刻 [s](再生記録は --use-sim-time で記録しているので同じ時間軸)。
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(uri, topics):
    """(topic, 受信時刻 [s], msg) を順に返す。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    while reader.has_next():
        topic, data, t = reader.read_next()
        yield topic, t * 1e-9, deserialize_message(data, get_message(types[topic]))


def trajectories(t, msg, out):
    """MarkerArray を最適軌道 (N, 2) と候補軌道の点 (M, 2) に分けて out に足す。"""
    opt = [(m.pose.position.x, m.pose.position.y) for m in msg.markers
           if m.ns == "Optimal Trajectory"]
    cand = [(m.pose.position.x, m.pose.position.y) for m in msg.markers
            if m.ns == "Candidate Trajectories"]
    out["t"].append(t)
    out["opt"].append(np.array(opt, dtype=np.float32).reshape(-1, 2))
    out["cand"].append(np.array(cand, dtype=np.float32).reshape(-1, 2))


def main():
    bag = Path(sys.argv[1])
    out_dir = bag / "analysis/bicycle"
    replay = out_dir / "replay_bag"

    d = {k: [] for k in ["map_odom", "odom_base", "odom_v", "orig_cmd", "new_cmd", "target"]}
    orig_traj = {"t": [], "opt": [], "cand": []}
    new_traj = {"t": [], "opt": [], "cand": []}
    plan = {"t": [], "pts": []}
    grid = None

    for topic, t, m in read(bag, ["/tf", "/odometry/filtered", "/cmd_vel_raw",
                                  "/trajectories", "/map"]):
        if topic == "/tf":
            for tr in m.transforms:
                row = (t, tr.transform.translation.x, tr.transform.translation.y,
                       yaw(tr.transform.rotation))
                if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                    d["map_odom"].append(row)
                elif tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                    d["odom_base"].append(row)
        elif topic == "/odometry/filtered":
            d["odom_v"].append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif topic == "/cmd_vel_raw":
            d["orig_cmd"].append((t, m.linear.x, m.angular.z))
        elif topic == "/trajectories":
            trajectories(t, m, orig_traj)
        elif topic == "/map":
            info = m.info
            grid = (np.array(m.data, dtype=np.int8).reshape(info.height, info.width),
                    info.resolution, info.origin.position.x, info.origin.position.y)

    for topic, t, m in read(replay, ["/target_speed", "/cmd_vel_raw", "/trajectories",
                                     "/transformed_global_plan"]):
        if topic == "/target_speed":
            d["target"].append((t, m.data))
        elif topic == "/cmd_vel_raw":
            d["new_cmd"].append((t, m.linear.x, m.angular.z))
        elif topic == "/trajectories":
            trajectories(t, m, new_traj)
        elif topic == "/transformed_global_plan":
            plan["t"].append(t)
            plan["pts"].append(np.array(
                [(p.pose.position.x, p.pose.position.y) for p in m.poses],
                dtype=np.float32).reshape(-1, 2))

    obj = lambda xs: np.array(xs + [None], dtype=object)[:-1]  # noqa: E731  形の違う配列の列
    np.savez_compressed(
        out_dir / "data.npz",
        **{k: np.array(v, dtype=np.float64).reshape(-1, len(v[0]) if v else 2)
           for k, v in d.items()},
        orig_traj_t=np.array(orig_traj["t"]), orig_traj_opt=obj(orig_traj["opt"]),
        orig_traj_cand=obj(orig_traj["cand"]),
        new_traj_t=np.array(new_traj["t"]), new_traj_opt=obj(new_traj["opt"]),
        new_traj_cand=obj(new_traj["cand"]),
        plan_t=np.array(plan["t"]), plan_pts=obj(plan["pts"]),
        grid=grid[0], grid_meta=np.array(grid[1:]))
    # コンテナは root で書くので、ホストから扱えるよう bag と同じ所有者にする
    st = bag.stat()
    os.chown(out_dir / "data.npz", st.st_uid, st.st_gid)
    print(f"{out_dir / 'data.npz'}: "
          + ", ".join(f"{k} {len(v)}" for k, v in d.items())
          + f", 新 trajectories {len(new_traj['t'])}, 旧 trajectories {len(orig_traj['t'])}")


if __name__ == "__main__":
    main()
