#!/usr/bin/env python3
"""地図とscanだけでどこまで合わせられるか(一致率の上限の目安)を出す(ホストで実行)。

    python3 tools/bag_analysis/oracle.py bags/<bag> [--every 10]

走行時の自己位置のまわり(±0.6 m、±12deg)で姿勢を総当たりし、各scanの一致率が最大になる姿勢を探す。
1scanずつ独立に最大化するので、通路の前後方向のように拘束の弱い向きへは都合よく滑る。
実際の推定器が届く値ではなく、上限の目安として使う。
  - 上限も低い区間: 地図かscanの問題で、どの照合法でも直せない。
  - 上限は高いのに推定器が低い区間: 推定器(パラメータ)の問題。
結果は <bag>/analysis/oracle.json (時刻, 一致率の上限, 最良姿勢と走行時の自己位置の差)。
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from pose_error import FOV, INLIER, LX, LYAW, RMAX, Grid, compose, interp_pose, uniq


def best(m, lx, ly, pose, span, step, aspan, astep):
    """pose のまわりで一致率が最大の (一致率, dx, dy, dth) を返す。"""
    offs = np.arange(-span, span + 1e-9, step)
    dx, dy = [a.ravel() for a in np.meshgrid(offs, offs)]
    out = (-1, 0, 0, 0)
    for dth in np.arange(-aspan, aspan + 1e-9, astep):
        th = pose[2] + dth
        c, s = math.cos(th), math.sin(th)
        px = pose[0] + c * lx - s * ly
        py = pose[1] + s * lx + c * ly
        j = ((px[None] + dx[:, None] - m.ox) / m.res).astype(int)
        i = ((py[None] + dy[:, None] - m.oy) / m.res).astype(int)
        ok = (i >= 0) & (i < m.H) & (j >= 0) & (j < m.W)
        d = np.where(ok, m.dt[np.clip(i, 0, m.H - 1), np.clip(j, 0, m.W - 1)], 1.0)
        score = (d <= INLIER).mean(axis=1)
        k = int(np.argmax(score))
        if score[k] > out[0]:
            out = (float(score[k]), float(dx[k]), float(dy[k]), float(dth))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--map', default='maps/track_v1.yaml')
    ap.add_argument('--every', type=int, default=10, help='何scanごとに評価するか(scanは約10Hz)')
    args = ap.parse_args()
    bag = Path(args.bag)
    f = np.load(bag / 'analysis/run.npz')
    t_bag = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information'][
        'starting_time']['nanoseconds_since_epoch'] * 1e-9
    m = Grid(f, args.map)
    mo, ob = uniq(f['tf_map_odom']), uniq(f['tf_odom_base'])
    st, R = f['scan_t'], f['ranges']
    ang = f['angle_min'] + np.arange(R.shape[1]) * f['angle_inc']
    fov = np.abs(ang) <= math.radians(FOV)
    ang = ang + LYAW
    idx = [k for k in range(0, len(st), args.every) if max(mo[0, 0], ob[0, 0]) <= st[k] <= min(mo[-1, 0], ob[-1, 0])]
    res = {'t': [], 'match': [], 'dx': [], 'dy': [], 'dth': []}
    for k in idx:
        tt = np.array([st[k]])
        pose = [float(a[0]) for a in compose(interp_pose(mo, tt), interp_pose(ob, tt))]
        r = R[k]
        ok = fov & np.isfinite(r) & (r >= 0.1) & (r <= RMAX)
        lx, ly = LX + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        s0, dx, dy, dth = best(m, lx, ly, pose, 0.6, 0.1, math.radians(12), math.radians(3))
        p1 = (pose[0] + dx, pose[1] + dy, pose[2] + dth)
        s1, ex, ey, eth = best(m, lx, ly, p1, 0.1, 0.025, math.radians(3), math.radians(0.75))
        res['t'].append(round(float(st[k] - t_bag), 2))
        res['match'].append(round(s1 * 100, 1))
        res['dx'].append(round(dx + ex, 3))
        res['dy'].append(round(dy + ey, 3))
        res['dth'].append(round(math.degrees(dth + eth), 2))
    a = np.array(res['match'])
    off = np.hypot(res['dx'], res['dy'])
    print(f'上限の一致率 中央 {np.median(a):.1f}% p10 {np.percentile(a, 10):.1f}%  '
          f'走行時の自己位置との差 中央 {np.median(off):.2f} m / 向き中央 {np.median(res["dth"]):+.1f}° ({len(a)} scan)')
    (bag / 'analysis/oracle.json').write_text(json.dumps(res))


if __name__ == '__main__':
    main()
