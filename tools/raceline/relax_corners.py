#!/usr/bin/env python3
"""レースラインのきついコーナーだけを、実機の最小旋回半径に収まるよう引き直す。

    python tools/raceline/relax_corners.py \\
        --raceline <元の線>.csv --map maps/track_v1.yaml \\
        --out maps/raceline.csv --png raceline_relaxed.png

ROS 不要(numpy / scipy / Pillow / PyYAML)。ホストの python で叩く想定。
既定の --raceline は maps/raceline.csv。直した後の線を入れても、きつい
区間が無いので取り直すだけで形は変わらない。直す前の線は
`git show f45ec7c:maps/raceline.csv` にある。

=== なぜ要るか ===

maps/raceline.csv はシム側(minicar_2026/nodes/raceline_opt.py)で最小曲率
最適化した線を、剛体変換で map 座標へ載せ替えたもの。その最適化には
**曲率の上限が入っていない**。シムの車両は |kappa| 1.74(R 0.575m)まで
曲がれたので問題にならなかったが、実車の実測 r_min は 0.83m しかなく、
ヘアピン 3 つ(0.25m で均した最小 R 0.62 / 0.62 / 0.78m)で曲がり切れない。

=== やっていること ===

1. |kappa| > 1/r_target の区間を探し、前後 --pad[m] を足して窓にする。
2. 窓ごとに 1 回、窓の中の点を法線方向へ alpha だけずらす(solve_window)。
       P_i = C_i + alpha_i * n_i
   alpha は 0.3m 間隔の B スプラインで、窓の両端で値も傾きも 0。
   目的: 曲率の二乗積分 + 1/r_target 超過の罰金 + 元の線からの距離(--stay)
   境界: 壁(SLAM 地図の占有セル)から車体中心まで half_width + --wall-margin
         (任意)--low-mu で渡した領域から half_width + --mat-margin
3. 周期 3 次スプラインで 0.05m 等間隔に取り直す。

窓の外は 1 点も動かさない。直線の通し方やマットの避け方など、シム側で
決めた線の性格はそのまま残る。

=== 失敗時挙動 ===

次のときは非 0 で終了し、CSV も PNG も書かない。
* |kappa| 最大が 1/r_target を超える(通路の幅が足りない)
* 窓の中の壁余裕が half_width + wall_margin を下回る
"""

import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import interpolate, ndimage, optimize

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "src", "minicar_raceline"))
from minicar_raceline.raceline_loop import (  # noqa: E402
    read_raceline_csv, write_raceline_csv)

DS = 0.05     # 出力点間隔[m]。raceline.csv と同じ


# ---------------------------------------------------------------------------
# 地図
# ---------------------------------------------------------------------------
class WallField:
    """SLAM 地図の占有セルまでの距離[m]を引く。

    通路の中の孤立した黒点(スキャンのノイズ)は min_blob 画素未満の
    連結成分として捨てる。raceline_to_map.py(シム側)と同じ扱い。
    """

    def __init__(self, map_yaml, min_blob):
        with open(map_yaml, encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta["image"])))
        p = img.astype(float) / 255.0
        if not meta.get("negate", 0):
            p = 1.0 - p
        occ = p > float(meta["occupied_thresh"])
        lab, n = ndimage.label(occ, structure=np.ones((3, 3)))
        size = ndimage.sum(occ, lab, index=np.arange(1, n + 1))
        keep = np.zeros(n + 1, bool)
        keep[1:] = size >= min_blob
        self.occ = keep[lab]
        self.raw = img
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.h, self.w = img.shape
        self.dist = ndimage.distance_transform_edt(~self.occ) * self.res

    def pix(self, q):
        q = np.atleast_2d(q)
        c = (q[:, 0] - self.ox) / self.res
        r = self.h - 1 - (q[:, 1] - self.oy) / self.res
        return c, r

    def __call__(self, q):
        c, r = self.pix(q)
        return ndimage.map_coordinates(self.dist, [r, c], order=1, cval=0.0)


def load_polys(path):
    polys = {}
    if not path:
        return polys
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            polys.setdefault(r["name"], []).append((float(r["x"]), float(r["y"])))
    return {k: np.array(v) for k, v in polys.items()}


def dist_to_poly(q, poly):
    """多角形までの符号付き距離[m]。内側が負。"""
    q = np.atleast_2d(q)
    a, b = poly, np.roll(poly, -1, axis=0)
    e = b - a
    t = np.clip(((q[:, None, :] - a[None]) * e[None]).sum(-1) / (e * e).sum(-1)[None], 0, 1)
    d = np.linalg.norm(q[:, None, :] - (a[None] + t[..., None] * e[None]), axis=-1).min(1)
    inside = np.zeros(len(q), bool)
    for (x1, y1), (x2, y2) in zip(a, b):
        cross = ((y1 > q[:, 1]) != (y2 > q[:, 1])) & \
            (q[:, 0] < (x2 - x1) * (q[:, 1] - y1) / (y2 - y1 + 1e-12) + x1)
        inside ^= cross
    return np.where(inside, -d, d)


# ---------------------------------------------------------------------------
# 幾何
# ---------------------------------------------------------------------------
def resample_closed(P, ds=DS):
    """周回路を弧長 ds 等間隔に取り直す。

    点の間は周期 3 次スプラインで補間する。折れ線のまま取り直すと、外へ
    ずらして点間隔が 8cm に開いたコーナーで頂点ごとに曲率の山ができる
    (実測: R 1.0m の線が R 0.90m と判定された)。
    """
    Q = np.vstack([P, P[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(Q, axis=0).T))])
    sp = interpolate.CubicSpline(s, Q, bc_type="periodic")
    # 弦長でなく弧長で等間隔にするため、細かく取ってから測り直す
    f = np.linspace(0.0, s[-1], 20 * len(P) + 1)
    F = sp(f)
    sf = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(F, axis=0).T))])
    n = int(round(sf[-1] / ds))
    t = np.interp(np.linspace(0.0, sf[-1], n, endpoint=False), sf, f)
    return sp(t)


def tangents(P):
    d = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def yaw_of(P):
    t = tangents(P)
    return np.arctan2(t[:, 1], t[:, 0])


def kappa_of(P):
    """raceline_manager と同じ式(dyaw/ds)の曲率。"""
    yaw = yaw_of(P)
    dyaw = np.angle(np.exp(1j * (np.roll(yaw, -1) - yaw)))
    dseg = np.linalg.norm(np.roll(P, -1, axis=0) - P, axis=1)
    return dyaw / np.maximum(dseg, 1e-9)


def kappa_avg(P):
    """0.25m(5 点)の移動平均をとった曲率。窓探しと合否はこれで行う。

    CSV は 0.1mm 丸めなので、点ごとの曲率には ±0.02 程度の雑音が乗る。
    また元の線にはスタート付近に数 cm だけ |kappa| 1.17 になる小さな S 字が
    あるが、0.25m で均すと 0.97 で、車の向きはほとんど変わらない。
    点ごとの値で判定すると、これらで窓が開いたり合否が揺れたりする。
    """
    return ndimage.uniform_filter1d(kappa_of(P), 5, mode="wrap")


def windows(kappa, k_lim, pad_pts):
    """|kappa| > k_lim の区間を pad_pts だけ広げ、重なりを併合した (start, length) の列。"""
    n = len(kappa)
    bad = np.abs(kappa) > k_lim
    if not bad.any():
        return []
    grow = np.zeros(n, bool)
    for i in np.nonzero(bad)[0]:
        grow[np.arange(i - pad_pts, i + pad_pts + 1) % n] = True
    if grow.all():
        raise SystemExit("窓が周回全体を覆った。--pad を小さくする")
    start = int(np.argmin(grow))   # 窓の外の点から数え始める
    out, i = [], 0
    while i < n:
        j = (start + i) % n
        if grow[j]:
            k = i
            while k < n and grow[(start + k) % n]:
                k += 1
            out.append(((start + i) % n, k - i))
            i = k
        else:
            i += 1
    return out


# ---------------------------------------------------------------------------
# 1 窓の最適化
# ---------------------------------------------------------------------------
def lateral_bounds(C, nrm, ok, max_d=0.8, step=0.01):
    """各点の法線上で ok(q) が真の連続区間 [lo, hi][m]。

    元の点が既に余裕を割っているとき(0 が ok でない)は、いちばん近い
    ok の区間を返す。0 を含む区間しか許さないと、その点は動けなくなる。
    """
    off = np.round(np.arange(-max_d, max_d + step / 2, step), 6)
    z = len(off) // 2
    lo, hi = np.zeros(len(C)), np.zeros(len(C))
    for i in range(len(C)):
        good = np.array([ok(C[i] + o * nrm[i]) for o in off])
        if not good.any():
            continue
        j = z if good[z] else int(np.nonzero(good)[0][np.argmin(np.abs(np.nonzero(good)[0] - z))])
        a = j
        while a > 0 and good[a - 1]:
            a -= 1
        b = j
        while b < len(off) - 1 and good[b + 1]:
            b += 1
        lo[i], hi[i] = off[a], off[b]
    return lo, hi


def menger(Q):
    """3点を通る円の曲率(符号つき、左旋回が正)。両端を除いた len(Q)-2 点ぶん。"""
    a = Q[1:-1] - Q[:-2]
    b = Q[2:] - Q[1:-1]
    c = Q[2:] - Q[:-2]
    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    den = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1) * np.linalg.norm(c, axis=1)
    return 2.0 * cross / np.maximum(den, 1e-12)


def spline_basis(m, spacing_pts):
    """窓の m 点に対する 3 次 B スプラインの基底 (m, nc)。

    両端の 2 本を落としてあるので、alpha も傾きも窓の両端で 0 になり、
    窓の外の線と向きまでつながる。
    """
    nc = max(6, int(round(m / spacing_pts)) + 3)
    inner = np.linspace(0.0, 1.0, nc - 2)
    knots = np.r_[[0.0] * 3, inner, [1.0] * 3]
    u = np.linspace(0.0, 1.0, m)
    B = interpolate.BSpline.design_matrix(u, knots, 3).toarray()
    return B[:, 2:-2]


def solve_window(P, idx, k_lim, ok, stay, weight=1e5, knot_m=0.3):
    """idx の点だけを法線方向へずらし、曲率を小さくする。

    目的 = ∫kappa^2 ds + weight * ∫max(0, |kappa| - k_lim)^2 ds
           + stay * ∫alpha^2 ds + 横方向の境界の罰金
    曲率は点の位置からそのまま測る(Menger)ので近似はない。

    stay は元の線に留まる強さ。0 にすると純粋な最小曲率になり、窓の中で
    通路いっぱいまで膨らむ(線が壁際の余裕ぎりぎりを通る)。

    alpha は knot_m[m] 間隔の B スプラインで持つ。点ごとに自由にすると、
    5cm 間隔の点の曲率は 1mm の動きにも鋭く反応する硬い問題になり、
    L-BFGS-B が罰金を消したところで止まって通路の幅を使い切らない
    (実測: 西ヘアピンで外壁まで 0.7m 余らせたまま R 0.98m で停止)。
    曲率の上限を alpha の線形近似で硬い制約にするのも、0.5m 動かすここでは
    近似が崩れて解けなかった(SLSQP: "Inequality constraints incompatible")。
    """
    n = len(P)
    ext = np.r_[(idx[0] - 1) % n, idx, (idx[-1] + 1) % n]
    C = P[ext]
    t = tangents(P)[ext]
    nrm = np.c_[-t[:, 1], t[:, 0]]                          # 左向き法線
    m = len(idx)

    lo, hi = lateral_bounds(C[1:-1], nrm[1:-1], ok)
    B = spline_basis(m, knot_m / DS)

    def fun(c):
        a = B @ c
        Q = C.copy()
        Q[1:-1] += a[:, None] * nrm[1:-1]
        k = menger(Q)
        ds = 0.5 * np.linalg.norm(Q[2:] - Q[:-2], axis=1)
        over = np.maximum(0.0, np.abs(k) - k_lim)
        out = np.maximum(0.0, a - hi) + np.maximum(0.0, lo - a)
        return float(np.sum(k * k * ds) + weight * np.sum(over * over * ds)
                     + stay * np.sum(a * a) * DS + 1e4 * np.sum(out * out))

    res = optimize.minimize(fun, np.zeros(B.shape[1]), method="L-BFGS-B",
                            options={"maxiter": 20000, "maxfun": 10 ** 7,
                                     "ftol": 1e-15, "gtol": 1e-10})
    Q = P.copy()
    Q[idx] = C[1:-1] + (B @ res.x)[:, None] * nrm[1:-1]
    return Q, res


def relax(P0, k_lim, ok, pad, stay):
    """きつい区間ごとに窓を切り、元の線を基準に 1 回ずつ解く。

    解いた線を基準にして解き直すことはしない。最小曲率は外へ膨らむほど
    得をするので、解き直すたびに線が外へずれていく(実測: 周長 35.5 -> 39.6m)。
    """
    wins = windows(kappa_avg(P0), k_lim, int(round(pad / DS)))
    P = P0.copy()
    moved = np.zeros(len(P0), bool)
    for st, ln in wins:
        idx = (st + np.arange(ln)) % len(P)
        # 上限は罰金なのでわずかに超える。解く側だけ 2% 内側を狙う
        P, res = solve_window(P, idx, k_lim * 0.98, ok, stay)
        moved[idx] = True
        print(f"  窓 s={st * DS:.2f}m から {ln * DS:.2f}m: {res.message}")
    return P, moved


# ---------------------------------------------------------------------------
def draw_png(path, field, P0, P1, k_lim, polys):
    sc = 3
    img = Image.fromarray(field.raw).convert("RGB")
    img = img.resize((field.w * sc, field.h * sc), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def xy(q):
        c, r = field.pix(q)
        return list(zip(c * sc, r * sc))

    for poly in polys.values():
        d.polygon(xy(poly), outline=(0, 150, 220))
    d.line(xy(np.vstack([P0, P0[:1]])), fill=(150, 150, 150), width=2)
    k1 = np.abs(kappa_avg(P1))
    pts = xy(np.vstack([P1, P1[:1]]))
    for i in range(len(P1)):
        col = (220, 0, 0) if k1[i] > k_lim * 1.005 else (0, 160, 0)
        d.line([pts[i], pts[i + 1]], fill=col, width=3)
    img.save(path)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--raceline", default=os.path.join(ROOT, "maps/raceline.csv"))
    ap.add_argument("--map", default=os.path.join(ROOT, "maps/track_v1.yaml"))
    ap.add_argument("--low-mu", default="",
                    help="避ける領域の輪郭 CSV (name,i,x,y / map 座標)。既定は無し")
    ap.add_argument("--vehicle", default=os.path.join(
        ROOT, "src/minicar_bringup/config/vehicle_params.yaml"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--png", default=None)
    ap.add_argument("--r-target", type=float, default=1.0,
                    help="線に許す最小半径[m]。r_min ちょうどにすると MPPI が"
                         "修正舵を切る余地がなくなるので、上乗せしておく")
    ap.add_argument("--wall-margin", type=float, default=0.15,
                    help="half_width に上乗せする壁との余裕[m]")
    ap.add_argument("--mat-margin", type=float, default=0.03,
                    help="half_width に上乗せするマットとの余裕[m]")
    ap.add_argument("--pad", type=float, default=2.5,
                    help="きつい区間の前後に足して動かしてよい長さ[m]")
    ap.add_argument("--stay", type=float, default=20.0,
                    help="元の線に留まる強さ。0 で純粋な最小曲率(通路いっぱいに膨らむ)")
    ap.add_argument("--min-blob", type=int, default=20)
    a = ap.parse_args(argv)

    with open(a.vehicle, encoding="utf-8") as f:
        veh = yaml.safe_load(f)
    hw = float(veh["vehicle"]["half_width"])
    r_min = float(veh["limits"]["r_min"])
    if a.r_target < r_min:
        raise SystemExit(f"--r-target {a.r_target} < r_min {r_min}")
    k_lim = 1.0 / a.r_target
    need = hw + a.wall_margin

    field = WallField(a.map, a.min_blob)
    polys = load_polys(a.low_mu)

    def ok(q):
        if field(q)[0] < need:
            return False
        return all(dist_to_poly(q, p)[0] >= hw + a.mat_margin for p in polys.values())

    _, x, y, _, _ = read_raceline_csv(a.raceline, closed=True)
    P0 = np.c_[x, y]
    print(f"r_min {r_min} m / 目標 R {a.r_target} m / 壁から車体中心 {need:.3f} m")
    P, moved = relax(P0, k_lim, ok, a.pad, a.stay)
    clr = field(P)
    P1 = resample_closed(P)

    kmax = float(np.abs(kappa_avg(P1)).max())
    kraw = float(np.abs(kappa_of(P1)).max())
    clr_min = float(clr[moved].min()) if moved.any() else float("inf")
    print(f"動かした点 {moved.sum()} / 全 {len(P0)} 点")
    print(f"|kappa| 最大 {kmax:.3f} 1/m (R {1 / kmax:.3f} m) "
          f"/ 0.25m で均さない点ごとの最大 {kraw:.3f}")
    print(f"窓の中の壁余裕 最小 {clr_min:.3f} m (要求 {need:.3f})")
    for name, p in polys.items():
        print(f"{name} 縁から最小 {dist_to_poly(P1, p).min():.3f} m")
    print(f"周長 {np.linalg.norm(np.roll(P1, -1, 0) - P1, axis=1).sum():.2f} m "
          f"(元 {np.linalg.norm(np.roll(P0, -1, 0) - P0, axis=1).sum():.2f} m)")

    if kmax > k_lim * 1.005:
        raise SystemExit(f"NG: R {1 / kmax:.3f} m < 目標 {a.r_target} m。通路幅が足りない")
    if clr_min < need - 0.01:
        raise SystemExit("NG: 窓の中で壁余裕が要求を割った")

    write_raceline_csv(a.out, P1[:, 0], P1[:, 1], yaw_of(P1))
    print(f"書き出し: {a.out}")
    if a.png:
        draw_png(a.png, field, P0, P1, k_lim, polys)
        print(f"図: {a.png}")


if __name__ == "__main__":
    main()
