#!/usr/bin/env python3
"""北東ヘアピンの出口から先だけを、上の通路の高さ(y 一定)の直線へ差し替える。

    python tools/raceline/splice_top_straight.py \\
        --raceline maps/raceline_center.csv --out maps/raceline_center_gate.csv

ROS 不要(numpy / scipy / Pillow / PyYAML)。ホストの python で叩く想定。

=== なぜ要るか ===

raceline_center.csv はヘアピンの出口(x≈2.2, y≈4.48)から北の板 7-8 の
仕切りの端(x≈-0.7, y≈4.42)へ、約 3m かけて斜めに寄っていく。
コーナーを出たらすぐ仕切りの下の通路の高さ(y≈4.08)へ移り、そこから
直線で走らせたい。smooth_raceline.py --straights だと全周を解き直すので
ヘアピン(芝生のコーナー)の通り方まで変わる。

=== やっていること ===

上の通路を -x 向きに走る点だけ y を書き換える(x は元の線のまま)。

    y = y_c + w(x) * (--y - y_c)

w は --x-start で 0、--x-start - --length で 1 になる C2 の smoothstep
(曲率が段にならない)。仕切りの西側(--x-back-start -> --x-back-end)で
同じ形で 0 へ戻し、元の線へつなぐ。元の線はそこで既に y≈4.08 なので
戻りの曲率はほぼ 0。--x-start より東(ヘアピン)と --x-back-end より
西は 1 点も動かさない。最後に 0.05m 等間隔に取り直す。

=== 失敗時挙動 ===

次のときは非 0 で終了し、CSV を書かない。
* 書き換えた区間の |kappa|(0.25m で均した値)が 1/--r-target を超える
* 書き換えた区間の壁余裕が half_width + --wall-margin を下回る
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from relax_corners import (  # noqa: E402
    ROOT, WallField, kappa_avg, read_raceline_csv, resample_closed, write_raceline_csv,
    yaw_of)
from smooth_raceline import add_walls  # noqa: E402


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t ** 3 * (10.0 - 15.0 * t + 6.0 * t * t)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--raceline", default=os.path.join(ROOT, "maps/raceline_center.csv"))
    ap.add_argument("--map", default=os.path.join(ROOT, "maps/track_v1.yaml"))
    ap.add_argument("--extra-walls", default=os.path.join(ROOT, "maps/track_v1_extra_walls.csv"))
    ap.add_argument("--vehicle", default=os.path.join(
        ROOT, "src/minicar_bringup/config/vehicle_params.yaml"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--y", type=float, default=4.08, help="揃える通路の高さ[m]")
    ap.add_argument("--x-start", type=float, default=2.16,
                    help="書き換えの始まり[m]。既定は center の向きが 180deg になる所")
    ap.add_argument("--length", type=float, default=1.8, help="--y へ移り切るまでの x の長さ[m]")
    ap.add_argument("--x-back-start", type=float, default=-0.3)
    ap.add_argument("--x-back-end", type=float, default=-1.2)
    ap.add_argument("--y-min", type=float, default=3.9, help="上の通路とみなす y の下限[m]")
    ap.add_argument("--r-target", type=float, default=0.9)
    ap.add_argument("--wall-margin", type=float, default=0.10)
    ap.add_argument("--min-blob", type=int, default=20)
    a = ap.parse_args(argv)

    with open(a.vehicle, encoding="utf-8") as f:
        hw = float(yaml.safe_load(f)["vehicle"]["half_width"])
    field = WallField(a.map, a.min_blob)
    add_walls(field, a.extra_walls)

    _, x, y, _, _ = read_raceline_csv(a.raceline, closed=True)
    P = np.c_[x, y]
    going_west = np.r_[np.diff(P[:, 0]), P[0, 0] - P[-1, 0]] < 0
    x = P[:, 0]
    sel = (P[:, 1] > a.y_min) & going_west & (x < a.x_start) & (x > a.x_back_end)
    w = (smoothstep((a.x_start - x) / a.length)
         * (1.0 - smoothstep((a.x_back_start - x) / (a.x_back_start - a.x_back_end))))
    P[sel, 1] += w[sel] * (a.y - P[sel, 1])
    Q = resample_closed(P)

    k = kappa_avg(Q)
    win = (Q[:, 1] > a.y_min) & (Q[:, 0] < a.x_start) & (Q[:, 0] > a.x_back_end)
    clear = field(Q[win])
    reach = Q[win & (np.abs(Q[:, 1] - a.y) < 0.02), 0].max()
    print(f"書き換え区間: kappa {k[win].min():+.2f}..{k[win].max():+.2f} "
          f"(最小 R {1 / np.abs(k[win]).max():.2f} m) / y={a.y}±0.02 に入る x {reach:.2f} m / "
          f"壁余裕 最小 {clear.min():.3f} m")
    print(f"全周: 最小 R {1 / np.abs(k).max():.3f} m / 点数 {len(Q)}")

    if np.abs(k[win]).max() > 1.0 / a.r_target:
        sys.exit(f"NG: R {1 / np.abs(k[win]).max():.3f} m < 目標 {a.r_target} m")
    if clear.min() < hw + a.wall_margin:
        sys.exit(f"NG: 壁余裕 {clear.min():.3f} m < {hw + a.wall_margin:.3f} m")

    write_raceline_csv(a.out, Q[:, 0], Q[:, 1], yaw_of(Q))
    print(f"書き出し: {a.out}")


if __name__ == "__main__":
    main()
