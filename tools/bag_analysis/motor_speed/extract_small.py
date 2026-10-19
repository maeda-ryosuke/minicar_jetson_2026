#!/usr/bin/env python3
"""bag から速度解析に要る小さいトピックだけを抜き出す (Jetson ホストで実行、ROS 不要)。

    cd ~/workspace/workspace_2026/minicar_jetson_2026
    python3 tools/bag_analysis/motor_speed/extract_small.py            # 一番新しい bag
    python3 tools/bag_analysis/motor_speed/extract_small.py <bag名> ...
    gzip -f ~/bag_small/*.db3

出力は ~/bag_small/<元のファイル名>.db3。/scan や /tf を落とすので数 MB になる。
元の bag は読み取り専用で開く。PC へは scp jetson@<IP>:~/bag_small/*.gz .\\bags\\small\\
"""
import glob
import os
import sqlite3
import sys

TOPICS = ('/cmd_vel_raw', '/cmd_vel', '/odom', '/odometry/filtered', '/imu',
          '/motor/pwm_count', '/motor/cmd_normalized', '/motor/steer_angle',
          '/motor/speed_ctrl', '/rosout', '/amcl_pose')

names = [f'bags/{n}' for n in sys.argv[1:]] or [
    max((d for d in glob.glob('bags/*') if os.path.isdir(d)), key=os.path.getmtime)]
os.makedirs(os.path.expanduser('~/bag_small'), exist_ok=True)
for name in names:
    print('対象:', name)
    for src in sorted(glob.glob(f'{name}/*.db3')):
        dst = os.path.expanduser(f'~/bag_small/{os.path.basename(src)}')
        if os.path.exists(dst):
            os.remove(dst)
        c = sqlite3.connect(f'file:{dst}', uri=True)
        c.execute(f"ATTACH DATABASE 'file:{os.path.abspath(src)}?mode=ro' AS s")
        for (sql,) in c.execute(
                "SELECT sql FROM s.sqlite_master WHERE sql IS NOT NULL").fetchall():
            c.execute(sql)
        for (t,) in c.execute("SELECT name FROM s.sqlite_master "
                              "WHERE type='table' AND name!='messages'").fetchall():
            c.execute(f"INSERT INTO main.{t} SELECT * FROM s.{t}")
        q = ','.join('?' * len(TOPICS))
        n = c.execute(f"INSERT INTO main.messages SELECT m.* FROM s.messages m "
                      f"JOIN s.topics t ON m.topic_id = t.id WHERE t.name IN ({q})",
                      TOPICS).rowcount
        c.commit()
        c.close()
        print(f'  {src}: {n} msgs -> {dst} ({os.path.getsize(dst) / 1e6:.1f} MB)')
