#!/usr/bin/env python3
"""レースラインの指定区間を、曲率の上限を守ったまま壁から均等に離す。

    python tools/raceline/balance_clearance.py \\
        --s0 0.3 --s1 9.5 --out maps/raceline.csv --png hairpin1.png

ROS 不要(numpy / scipy / Pillow / PyYAML)。ホストの python で叩く想定。
既定の --raceline は maps/raceline.csv。直す前の線は
`git show 54b14f7:maps/raceline.csv` にある。

=== なぜ要るか ===

relax_corners.py は「元の線から離れない」ことを優先してきつい所だけを
引き直すので、R は満たしても壁際を通ったままになる。ヘアピン1
(南東、s 0.3〜9.5m)は入口で南の壁 0.28m、出口で北の壁 0.25m
(車体中心から)を通り、実車が出口側の壁に当たって復帰できなくなった。
東側の空きは使い切れていなかった。

=== やっていること ===

区間の点を法線方向へ alpha だけずらす(relax_corners.solve_window と同じ
B スプライン表現。両端で値も傾きも 0 なので区間の外とはつながる)。

    目的 = ∫kappa^2 ds
         + --w-over  * ∫max(0, |kappa| - 1/r_target)^2 ds
         + --w-clear * ∫max(0, clear_target - 壁までの距離)^2 ds

元の線に留まる項は入れない。clear_target を通路で取れる値より大きく
置くと、どの壁からも同じだけ離れたところで釣り合う。

track_v1 のヘアピン1は南の壁から北の壁まで約 2.85m しかなく、R 1.0m で
回ると余裕は各側 0.33m 前後が上限(R 0.95m で 0.38m)。

=== 失敗時挙動 ===

次のときは非 0 で終了し、CSV も PNG も書かない。
* 区間の中の |kappa| 最大が 1/r_target を超える
* 区間の中の壁余裕が元の線より小さくなる
"""

import argparse
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import optimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from relax_corners import (  # noqa: E402
    DS, ROOT, WallField, kappa_avg, menger, read_raceline_csv, resample_closed,
    spline_basis, tangents, write_raceline_csv, yaw_of)


def solve(P, idx, k_lim, field, clear_target, w_over, w_clear, knot_m=0.3):
    n = len(P)
    ext = np.r_[(idx[0] - 1) % n, idx, (idx[-1] + 1) % n]
    C = P[ext]
    t = tangents(P)[ext]
    nrm = np.c_[-t[:, 1], t[:, 0]]                          # 左向き法線
    B = spline_basis(len(idx), knot_m / DS)

    def fun(c):
        a = B @ c
        Q = C.copy()
        Q[1:-1] += a[:, None] * nrm[1:-1]
        k = menger(Q)
        ds = 0.5 * np.linalg.norm(Q[2:] - Q[:-2], axis=1)
        over = np.maximum(0.0, np.abs(k) - k_lim)
        short = np.maximum(0.0, clear_target - field(Q[1:-1]))
        return float(np.sum(k * k * ds) + w_over * np.sum(over * over * ds)
                     + w_clear * np.sum(short * short) * DS)

    res = optimize.minimize(fun, np.zeros(B.shape[1]), method="L-BFGS-B",
                            options={"maxiter": 30000, "maxfun": 10 ** 7})
    Q = P.copy()
    Q[idx] = C[1:-1] + (B @ res.x)[:, None] * nrm[1:-1]
    return Q, res


def section(P, s0, s1):
    s = np.arange(len(P)) * DS
    return np.nonzero((s >= s0) & (s < s1))[0]


def draw_png(path, field, P0, P1):
    sc = 3
    img = Image.fromarray(field.raw).convert("RGB")
    img = img.resize((field.w * sc, field.h * sc), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def xy(q):
        c, r = field.pix(q)
        return list(zip(c * sc, r * sc))

    d.line(xy(np.vstack([P0, P0[:1]])), fill=(150, 150, 150), width=2)
    d.line(xy(np.vstack([P1, P1[:1]])), fill=(0, 160, 0), width=3)
    img.save(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--raceline", default=os.path.join(ROOT, "maps/raceline.csv"))
    ap.add_argument("--map", default=os.path.join(ROOT, "maps/track_v1.yaml"))
    ap.add_argument("--vehicle", default=os.path.join(
        ROOT, "src/minicar_bringup/config/vehicle_params.yaml"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--png", default=None)
    ap.add_argument("--s0", type=float, required=True, help="区間の始まり(弧長[m])")
    ap.add_argument("--s1", type=float, required=True, help="区間の終わり(弧長[m])")
    ap.add_argument("--r-target", type=float, default=1.0)
    ap.add_argument("--clear-target", type=float, default=0.46,
                    help="壁から車体中心までの目標[m]。取れる値より大きめに置く")
    ap.add_argument("--w-over", type=float, default=1e6)
    ap.add_argument("--w-clear", type=float, default=3e4)
    ap.add_argument("--min-blob", type=int, default=20)
    a = ap.parse_args(argv)

    with open(a.vehicle, encoding="utf-8") as f:
        veh = yaml.safe_load(f)
    r_min = float(veh["limits"]["r_min"])
    if a.r_target < r_min:
        raise SystemExit(f"--r-target {a.r_target} < r_min {r_min}")
    k_lim = 1.0 / a.r_target

    field = WallField(a.map, a.min_blob)
    _, x, y, _, _ = read_raceline_csv(a.raceline, closed=True)
    P0 = np.c_[x, y]
    idx = section(P0, a.s0, a.s1)
    # 上限は罰金なのでわずかに超える。解く側だけ 2% 内側を狙う
    P, res = solve(P0, idx, k_lim * 0.98, field, a.clear_target, a.w_over, a.w_clear)
    print(f"s={a.s0:.2f}〜{a.s1:.2f}m: {res.message}")
    P1 = resample_closed(P)

    i0, i1 = section(P0, a.s0, a.s1), section(P1, a.s0, a.s1)
    k0, k1 = kappa_avg(P0)[i0], kappa_avg(P1)[i1]
    c0, c1 = float(field(P0[i0]).min()), float(field(P1[i1]).min())
    kall = float(np.abs(kappa_avg(P1)).max())
    print(f"区間の最小 R   {1 / np.abs(k0).max():.3f} -> {1 / np.abs(k1).max():.3f} m"
          f" (全周 {1 / kall:.3f} m)")
    print(f"区間の壁余裕   {c0:.3f} -> {c1:.3f} m (車体中心)")
    print(f"周長 {np.linalg.norm(np.roll(P0, -1, 0) - P0, axis=1).sum():.2f} -> "
          f"{np.linalg.norm(np.roll(P1, -1, 0) - P1, axis=1).sum():.2f} m")

    if np.abs(k1).max() > k_lim * 1.005:
        raise SystemExit(f"NG: R {1 / np.abs(k1).max():.3f} m < 目標 {a.r_target} m")
    if c1 < c0:
        raise SystemExit("NG: 壁余裕が元の線より小さくなった")

    write_raceline_csv(a.out, P1[:, 0], P1[:, 1], yaw_of(P1))
    print(f"書き出し: {a.out}")
    if a.png:
        draw_png(a.png, field, P0, P1)
        print(f"図: {a.png}")


if __name__ == "__main__":
    main()
