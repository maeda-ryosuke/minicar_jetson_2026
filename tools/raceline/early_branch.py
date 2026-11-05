#!/usr/bin/env python3
"""北東ヘアピンの出口を低くし、上の通路の分岐(北の板 7-8)の下へ早く移る線を作る。

    python tools/raceline/early_branch.py \\
        --raceline maps/raceline_center_gate.csv --out maps/raceline_early_branch.csv --png eb.png

ROS 不要(numpy / scipy / Pillow / PyYAML)。ホストの python で叩く想定。

=== なぜ要るか ===

raceline_center_gate.csv はヘアピンを y≈4.48(仕切りとほぼ同じ高さ)で抜け、
そこから左 -> 右の S 字で 0.4m 下りて x≈0.8 で y=4.08 に揃う。ヘアピンの
出口で車が膨らむと、下り切る前に仕切りの東端(x≈-0.55, y≈4.42)へ着き、
正面から当たる(2026-11-05 実走)。splice_top_straight.py はヘアピンを
1 点も動かさないので、出口の高さは変えられない。

=== やっていること ===

1. 元の線の弧長 --s-start .. --s-end(既定: 3 本目の通路 x≈0.7 から上の
   通路 x≈-2.4 まで。ヘアピンを含む)だけを窓にする。
2. 窓の中の点を法線方向へ alpha だけずらす。alpha は --knot 間隔の 3 次
   B スプラインで、窓の両端で値・傾き・2 階微分が 0(曲率が段にならない)。
3. 目的関数は smooth_raceline.objective(全周で評価。窓の外は定数)に、
   上の通路を西へ走る点のうち x <= --x-pull の点を y=--y へ引き寄せる項
   (--w-pull)を足したもの。--x-pull を東へ(大きく)するほど早く下りる。
   ヘアピンは R --r-target を守れる範囲で、東へ膨らんで出口が低くなる形
   (電球形)に変わる。広い所で壁から離す項(w_clear)は y=--y への引き寄せと
   喧嘩するので使わない。
4. 0.05m 等間隔に取り直し、窓を弧長で取り直してもう一度解く(--passes)。

--x-pull はヘアピンの頂点(x≈2.9)より東まで掛けること。x-pull 2.0 / 2.5 では
ヘアピンが東へ膨らんだぶん出口の頂点が x≈2.7 へ移り、下り始めがかえって
遅くなった(y=4.08±0.03 に収まるのは x≈0 から)。--w-pull も 1e4 では弱く、
x 1.0〜1.5 で 4.05 まで下がりすぎる。

既定値での結果(元は raceline_center_gate.csv):

                                     center_gate   新
  y=4.08±0.03 に収まる区間の東端 x      0.83        2.08
  仕切り東端の 1.5m 手前からの向きのずれ  11.7deg      2.2deg
  ヘアピンを抜ける高さ(頂点の y)      4.48        4.22
  ヘアピンの最小 R / 東端 x            0.91m/3.35   0.92m/4.02
  ヘアピンの壁余裕の最小               0.42m       0.29m(東の外壁)
  灰色マットを踏む長さ                 1.05m       0
  周長                                 35.94m      37.24m
  窓の外の点の移動                      -           1.1mm 以内(取り直しの誤差)

東へ膨らむのは灰色マットを避けたからではない(既定で踏んでよい)。R を大きく
取るほど目的関数が下がるので、外壁まで広がる。--x-max 3.4 で元のヘアピンと
同じ大きさに抑えると、入りを低く(y≈2.38)取って出口を y≈4.25 に下げる形に
なり、y=4.08 に収まるのは x≈1.47 から、灰色マットは元と同じ 1.05m 踏む
(maps/raceline_early_branch_gray.csv)。

=== 失敗時挙動 ===

次のときは非 0 で終了し、CSV も PNG も書かない。
* 窓の |kappa|(0.25m で均した値)が 1/--r-target を超える
* 窓の壁余裕が half_width + --wall-margin を下回る
* 避けるマット(--low-mu から --allow-mat を除いたもの)に近づきすぎる
"""

import argparse
import os
import sys
from types import SimpleNamespace

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import optimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from relax_corners import (  # noqa: E402
    DS, ROOT, WallField, dist_to_poly, kappa_avg, read_raceline_csv, resample_closed,
    tangents, write_raceline_csv, yaw_of)
from smooth_raceline import (  # noqa: E402
    MAT_MARGIN, MAT_MARGIN_DEFAULT, W_HARD, Field, add_walls, load_polys, objective, start_at)

# 仕切り(北の板 7-8)の東端。合否には使わず、報告だけ。
DIVIDER_TIP = np.array([-0.55, 4.43])


def window_basis(n, knot_pts):
    """長さ n の窓に置く 3 次 B スプラインの基底 (n, m)。

    台が窓の内側に収まる基底だけを使うので、両端で値も導関数も 0。
    """
    nk = max(5, int(round(n / knot_pts)))
    u = np.arange(n) * nk / (n - 1)
    j = np.arange(2, nk - 1)
    t = np.abs(u[:, None] - j[None])
    return np.where(t < 1, (4 - 6 * t ** 2 + 3 * t ** 3) / 6,
                    np.where(t < 2, (2 - t) ** 3 / 6, 0.0))


def pull_mask(P, a):
    """上の通路を西へ走り、x が --x-back .. --x-pull にある点。"""
    west = tangents(P)[:, 0] < -0.3
    return (west & (P[:, 1] > a.y_min) & (P[:, 0] <= a.x_pull)
            & (P[:, 0] >= a.x_back)).astype(float)


def solve(P, i0, n, a):
    idx = (i0 + np.arange(n)) % len(P)
    t = tangents(P)
    nrm = np.c_[-t[:, 1], t[:, 0]][idx]
    B = window_basis(n, a.knot / DS)
    a.mask = np.zeros(len(P))
    a.pull_masks = [pull_mask(P, a)]

    def fun(c):
        Q = P.copy()
        Q[idx] += (B @ c)[:, None] * nrm
        J, gQ = objective(Q, a)
        if a.x_max is not None:
            # ヘアピンが東へ膨らみすぎないよう、x > --x-max を罰する
            over = np.maximum(0.0, Q[idx, 0] - a.x_max)
            J += W_HARD * DS * np.sum(over * over)
            gQ[idx, 0] += 2 * W_HARD * DS * over
        return J, B.T @ np.sum(gQ[idx] * nrm, axis=1)

    res = optimize.minimize(fun, np.zeros(B.shape[1]), jac=True, method="L-BFGS-B",
                            options={"maxiter": 20000, "maxfun": 10 ** 5,
                                     "ftol": 1e-13, "gtol": 1e-9})
    Q = P.copy()
    Q[idx] += (B @ res.x)[:, None] * nrm
    return Q, res


def window_of(P, q0, q1):
    """q0 に一番近い点から q1 に一番近い点まで(進行方向)の (先頭, 点数)。"""
    i0 = int(np.argmin(np.linalg.norm(P - q0, axis=1)))
    i1 = int(np.argmin(np.linalg.norm(P - q1, axis=1)))
    return i0, (i1 - i0) % len(P) + 1


def report(name, P, i0, n, field, mats, a):
    idx = (i0 + np.arange(n)) % len(P)
    W = P[idx]
    k = kappa_avg(P)[idx]
    clr = field(W)
    top = (W[:, 1] > a.y_min) & (tangents(P)[idx, 0] < -0.3)
    # 仕切りの東端から東へたどり、y が --y から 3cm 以上外れる手前までの x
    east = top & (W[:, 0] >= DIVIDER_TIP[0])
    order = np.argsort(W[east, 0])
    off = np.abs(W[east, 1][order] - a.y) >= 0.03
    xs = W[east, 0][order]
    reach = xs[np.argmax(off) - 1] if off.any() and off.argmax() > 0 else (
        xs[-1] if not off.any() else float("nan"))
    # 仕切りの東端を通り過ぎるときの線の y と、東端からの距離
    near_tip = top & (np.abs(W[:, 0] - DIVIDER_TIP[0]) < 0.03)
    tip = np.linalg.norm(W - DIVIDER_TIP, axis=1)[top].min()
    # 仕切りの東端の 1.5m 手前から先端までの、向きの最大のずれ(真西から)
    app = top & (W[:, 0] < DIVIDER_TIP[0] + 1.5) & (W[:, 0] > DIVIDER_TIP[0])
    yaw = np.degrees(np.abs(np.angle(-np.exp(1j * yaw_of(P)[idx][app])))).max()
    i = int(np.argmax(np.abs(k)))
    print(f"[{name}] 窓: 最小 R {1 / np.abs(k).max():.3f} m (x {W[i, 0]:.2f}, y {W[i, 1]:.2f})"
          f" / 壁余裕 最小 {clr.min():.3f} m (x {W[np.argmin(clr), 0]:.2f})")
    print(f"[{name}] y={a.y}±0.03 に収まる区間の東端 x {reach:.2f} / 仕切り東端での y "
          f"{W[near_tip, 1].mean():.3f} / 東端までの距離 {tip:.3f} m"
          f" / 東端 1.5m 手前からの向きのずれ 最大 {yaw:.1f} deg")
    for nm, (poly, need) in mats.items():
        print(f"[{name}] {nm} 必要距離に対する余り {(dist_to_poly(W, poly) - need).min():+.3f} m")
    return k, clr


def draw_png(path, wf, P0, P1, clear_min):
    X0, X1, Y0, Y1, sc = -3.0, 4.45, 1.8, 5.1, 110
    img = Image.new("RGB", (int((X1 - X0) * sc), int((Y1 - Y0) * sc)), "white")
    d = ImageDraw.Draw(img)

    def px(q):
        q = np.atleast_2d(q)
        return list(zip((q[:, 0] - X0) * sc, (Y1 - q[:, 1]) * sc))

    rr, cc = np.nonzero(wf.occ)
    xy = np.c_[wf.ox + cc * wf.res, wf.oy + (wf.h - 1 - rr) * wf.res]
    for (u, v) in px(xy):
        d.rectangle([u - wf.res * sc / 2, v - wf.res * sc / 2,
                     u + wf.res * sc / 2, v + wf.res * sc / 2], fill=(0, 0, 0))
    for gx in np.arange(-3.0, 4.41, 0.5):
        d.line(px(np.array([[gx, Y0], [gx, Y1]])), fill=(205, 205, 255))
        d.text(px([gx + 0.02, Y0 + 0.12])[0], f"{gx:.1f}", fill=(0, 0, 200))
    for gy in np.arange(2.0, 5.1, 0.2):
        d.line(px(np.array([[X0, gy], [X1, gy]])), fill=(205, 205, 255))
        d.text(px([X0 + 0.02, gy + 0.05])[0], f"{gy:.1f}", fill=(0, 0, 200))
    d.line(px(np.vstack([P0, P0[:1]])), fill=(150, 150, 150), width=2)
    clr = wf(P1)
    pts = px(np.vstack([P1, P1[:1]]))
    for i in range(len(P1)):
        col = (220, 0, 0) if clr[i] < clear_min + 0.05 else (0, 160, 0)
        d.line([pts[i], pts[i + 1]], fill=col, width=3)
    img.save(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--raceline", default=os.path.join(ROOT, "maps/raceline_center_gate.csv"))
    ap.add_argument("--map", default=os.path.join(ROOT, "maps/track_v1.yaml"))
    ap.add_argument("--extra-walls", default=os.path.join(ROOT, "maps/track_v1_extra_walls.csv"))
    ap.add_argument("--low-mu", default=os.path.join(ROOT, "maps/track_v1_low_mu.csv"))
    ap.add_argument("--allow-mat", action="append", default=["gray_mat"],
                    help="踏んでよいマット。既定は gray_mat(center と同じ)")
    ap.add_argument("--vehicle", default=os.path.join(
        ROOT, "src/minicar_bringup/config/vehicle_params.yaml"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--png", default=None)
    ap.add_argument("--s-start", type=float, default=16.5,
                    help="窓の始まり(元の線の弧長[m])。既定は 3 本目の通路 x≈0.7")
    ap.add_argument("--s-end", type=float, default=26.0,
                    help="窓の終わり[m]。既定は上の通路 x≈-2.4")
    ap.add_argument("--y", type=float, default=4.08, help="寄せる高さ[m](仕切りの下の通路)")
    ap.add_argument("--x-pull", type=float, default=3.0,
                    help="ここより西(x が小さい側)を --y へ引き寄せる[m]。大きいほど早く下りる")
    ap.add_argument("--x-back", type=float, default=-2.4, help="引き寄せの西端[m]")
    ap.add_argument("--y-min", type=float, default=3.75, help="上の通路とみなす y の下限[m]")
    ap.add_argument("--w-pull", type=float, default=1e5)
    ap.add_argument("--x-max", type=float, default=None,
                    help="窓の点の x の上限[m](ヘアピンの東端)。既定は無し(外壁まで膨らむ)")
    ap.add_argument("--r-target", type=float, default=0.9)
    ap.add_argument("--wall-margin", type=float, default=0.15)
    ap.add_argument("--clear-near", type=float, default=0.32)
    ap.add_argument("--w-near", type=float, default=1e6)
    ap.add_argument("--w-dk", type=float, default=1000.0)
    ap.add_argument("--w-k", type=float, default=0.1)
    ap.add_argument("--knot", type=float, default=0.3)
    ap.add_argument("--passes", type=int, default=2)
    ap.add_argument("--min-blob", type=int, default=20)
    a = ap.parse_args(argv)
    if os.path.abspath(a.out) == os.path.abspath(a.raceline):
        raise SystemExit("--out が --raceline と同じ。元の線を残すため別の名前にする")

    with open(a.vehicle, encoding="utf-8") as f:
        veh = yaml.safe_load(f)
    hw = float(veh["vehicle"]["half_width"])
    if a.r_target < float(veh["limits"]["r_min"]):
        raise SystemExit(f"--r-target {a.r_target} < r_min {veh['limits']['r_min']}")
    wf = WallField(a.map, a.min_blob)
    add_walls(wf, a.extra_walls)
    field = Field(wf)
    mats = {nm: (p, hw + MAT_MARGIN.get(nm, MAT_MARGIN_DEFAULT))
            for nm, p in load_polys(a.low_mu).items() if nm not in a.allow_mat}

    s, x, y, _, _ = read_raceline_csv(a.raceline, closed=True)
    P0 = np.c_[x, y]
    q0 = P0[np.searchsorted(s, a.s_start)]
    q1 = P0[np.searchsorted(s, a.s_end)]
    o = SimpleNamespace(
        field=field, k_lim=1.0 / a.r_target, clear_min=hw + a.wall_margin,
        clear_target=0.0, w_clear=0.0, clear_near=a.clear_near, w_near=a.w_near,
        w_k=a.w_k, w_dk=a.w_dk, w_over=1e6, w_straight=0.0, mats=list(mats.values()),
        pulls=[(np.array([0.0, a.y]), np.array([0.0, 1.0]), a.w_pull)],
        knot=a.knot, x_pull=a.x_pull, x_back=a.x_back, y_min=a.y_min, y=a.y, x_max=a.x_max)

    print(f"目標 R {a.r_target} m / 壁余裕 下限 {o.clear_min:.3f} m / "
          f"x <= {a.x_pull} を y={a.y} へ (w_pull {a.w_pull:g})")
    i0, n = window_of(P0, q0, q1)
    report("元", P0, i0, n, field, mats, o)

    P = P0
    for it in range(a.passes):
        i0, n = window_of(P, q0, q1)
        k_lim, o.k_lim = o.k_lim, o.k_lim * 0.98
        Q, res = solve(P, i0, n, o)
        o.k_lim = k_lim
        P = start_at(resample_closed(Q), P0[0])
        print(f"  pass {it + 1}: {res.message} (評価 {res.nfev} 回)")
    i0, n = window_of(P, q0, q1)
    k, clr = report("新", P, i0, n, field, mats, o)
    kall = kappa_avg(P)
    print(f"[新] 全周: 最小 R {1 / np.abs(kall).max():.3f} m / 周長 "
          f"{np.linalg.norm(np.roll(P, -1, 0) - P, axis=1).sum():.2f} m")

    if np.abs(k).max() > o.k_lim * 1.005:
        raise SystemExit(f"NG: R {1 / np.abs(k).max():.3f} m < 目標 {a.r_target} m")
    if clr.min() < o.clear_min:
        raise SystemExit("NG: 壁余裕が下限を割った")
    W = P[(i0 + np.arange(n)) % len(P)]
    if any((dist_to_poly(W, p) - need).min() < 0 for p, need in mats.values()):
        raise SystemExit("NG: マットに近づきすぎた")

    write_raceline_csv(a.out, P[:, 0], P[:, 1], yaw_of(P))
    print(f"書き出し: {a.out}")
    if a.png:
        draw_png(a.png, wf, P0, P, o.clear_min)
        print(f"図: {a.png}")


if __name__ == "__main__":
    main()
