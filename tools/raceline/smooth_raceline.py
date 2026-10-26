#!/usr/bin/env python3
"""レースラインを全周まとめて引き直し、くねりを取って壁から離す。

    python tools/raceline/smooth_raceline.py --out maps/raceline_smooth.csv --png smooth.png

ROS 不要(numpy / scipy / Pillow / PyYAML)。ホストの python で叩く想定。
既定の --raceline は maps/raceline.csv(中心線を作る種に使うだけで、形は
引き継がない)。--out は既存の線と別の名前にすること(上書きすると比べられない)。

=== なぜ要るか ===

maps/raceline.csv はシムの最小曲率線を map へ載せ、きつい所だけを
relax_corners.py / balance_clearance.py で部分的に直したもの。

* 元の線は小さな S 字や曲率の段差を何か所も持っている(s≈2, 7, 15,
  25〜28, 38m)。窓の外を動かさない部分修正では取れない。速度を上げると、
  そのたびに舵を左右へ振ることになる。
* 壁際を通る区間が残っている(車体中心から壁まで 0.21〜0.30m が
  s≈12, 15, 20, 24, 28m)。MPPI はコーナーで内側へ寄るので、速度を
  上げると壁の端を刈りにいく。

=== やっていること ===

1. 初期値として通路の中心線を作る(centerline)。元の線から始めると、その
   S 字や壁際の癖を残したまま局所解に落ちる(実測: 曲率変化の総量が
   20.6 -> 17.8 にしか下がらなかった。中心線からだと 10.0)。
2. 線の各点を法線方向へ alpha だけずらす。alpha は全周を --knot[m] 間隔で
   覆う周期 3 次 B スプライン(区間の継ぎ目がない)。

    目的 = --w-k     * ∫kappa^2 ds                         (大きな R で回る)
         + --w-dk    * ∫(dkappa/ds)^2 ds                   (くねらない。舵を急に振らない)
         + --w-over  * ∫max(0, |kappa| - 1/r_target)^2 ds
         + --w-clear * ∫max(0, clear_target - 壁距離)^2 ds  (広い所で壁から離す)
         + --w-near  * ∫max(0, clear_near - 壁距離)^2 ds    (狭い所で真ん中を通す)
         + 1e6 * ∫max(0, clear_min - 壁距離)^2 ds          (壁余裕の下限)
         + 1e6 * ∫max(0, マットまでの必要距離 - マット距離)^2 ds

   壁距離は SLAM 地図の占有セル(+ --extra-walls)までの距離なので、壁の端
   (支柱や板の角)もそのまま含まれる。clear_target は通路で取れる値より
   大きめに置く。clear_near の段が無いと、狭い通路の出口(s≈28m、幅 0.6m)
   で次のコーナーの内側の角へ 4cm 寄った(0.239m。真ん中なら 0.29m)。
3. 0.05m 等間隔に取り直し、それを初期値にもう一度解く(--passes)。法線の
   向きが初期の線に依存するので、1 回だと大きく動いた所で近似が粗い。

勾配は解析的に出している(数値微分だと 1 回 8 分かかった)。曲率は
kappa = (d1 x d2) / |d1|^3(d1, d2 は点列の中心差分)。

=== 通路の形から決まる限界(track_v1) ===

* ヘアピン 3 か所は隣り合う通路の中心の間隔(1.35〜1.53m)が 2R より狭いので、
  R 1.0m で回ると壁余裕は内外とも 0.3m 前後が上限。R 1.1m は取れない
  (--r-target 1.1 では解けない)。
* s≈27〜30m(北の板 7-8 と内壁の間)は幅 0.57〜0.70m で、真ん中を通っても
  壁余裕は 0.28〜0.33m。

=== 地図に無いもの ===

* --extra-walls: オレンジ物体(支柱21-24 の壁列の一部)は LiDAR にほとんど
  写らず、地図の中段の壁がそこで途切れている。壁線として足さないと線が
  物体を突き抜ける近道を通る。
* --low-mu: 水色 / 灰色マット。シムの方針(minicar_2026/nodes/raceline_opt.py)
  と同じく踏まない。水色だけ余裕を大きく取るのもシムに合わせてある。
どちらも minicar_2026 側のコース図座標を、raceline_to_map.py と同じ剛体変換で
map へ載せたもの(CSV の冒頭に変換を書いてある)。

=== 失敗時挙動 ===

次のときは非 0 で終了し、CSV も PNG も書かない。
* |kappa| 最大が 1/r_target を超える
* 壁余裕が clear_min を下回る
* マットまでの距離が要求を下回る
"""

import argparse
import csv
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import interpolate, ndimage, optimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from relax_corners import (  # noqa: E402
    DS, ROOT, WallField, dist_to_poly, kappa_avg, read_raceline_csv,
    resample_closed, tangents, write_raceline_csv, yaw_of)

# マットごとの余裕[m](half_width に上乗せ)。シムの MAT_MARGIN_BY_NAME と同じ。
# 水色は脇の通路に余裕があり、MPPI の横偏差でシムで乗り上げた実績がある。
MAT_MARGIN = {"blue_mat": 0.18}
MAT_MARGIN_DEFAULT = 0.03


def read_csv_rows(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def load_polys(path):
    polys = {}
    for r in read_csv_rows(path) if path else []:
        polys.setdefault(r["name"], []).append((float(r["x"]), float(r["y"])))
    return {k: np.array(v) for k, v in polys.items()}


def add_walls(field, path):
    """線分の壁を占有セルとして焼き込み、距離場を作り直す。"""
    for r in read_csv_rows(path) if path else []:
        a = np.array([float(r["x1"]), float(r["y1"])])
        b = np.array([float(r["x2"]), float(r["y2"])])
        n = int(np.ceil(np.linalg.norm(b - a) / (field.res / 4))) + 1
        c, row = field.pix(a + np.linspace(0, 1, n)[:, None] * (b - a))
        c, row = np.round(c).astype(int), np.round(row).astype(int)
        ok = (c >= 0) & (c < field.w) & (row >= 0) & (row < field.h)
        field.occ[row[ok], c[ok]] = True
    field.dist = ndimage.distance_transform_edt(~field.occ) * field.res


def periodic_basis(n, nc):
    """周長を nc 等分した周期 3 次 B スプラインの基底 (n, nc)。"""
    u = np.arange(n) * nc / n
    t = np.abs((u[:, None] - np.arange(nc)[None] + nc / 2) % nc - nc / 2)
    return np.where(t < 1, (4 - 6 * t ** 2 + 3 * t ** 3) / 6,
                    np.where(t < 2, (2 - t) ** 3 / 6, 0.0))


def poly_dist_grad(q, poly):
    """多角形までの符号付き距離(内側が負)と、その q についての勾配。"""
    a, b = poly, np.roll(poly, -1, axis=0)
    e = b - a
    t = np.clip(((q[:, None, :] - a[None]) * e[None]).sum(-1) / (e * e).sum(-1)[None], 0, 1)
    foot = a[None] + t[..., None] * e[None]
    dv = q[:, None, :] - foot
    dn = np.linalg.norm(dv, axis=-1)
    j = np.argmin(dn, axis=1)
    i = np.arange(len(q))
    d = dn[i, j]
    g = dv[i, j] / np.maximum(d, 1e-9)[:, None]
    inside = dist_to_poly(q, poly) < 0
    return np.where(inside, -d, d), np.where(inside[:, None], -g, g)


class Field:
    """壁距離と、その勾配を map 座標で引く。

    双 3 次スプラインで補間する。線形補間だと勾配が格子ごとに飛び、
    L-BFGS-B の直線探索が失敗する(ABNORMAL で途中終了する)。
    """

    def __init__(self, wf):
        self.wf = wf
        self.sp = interpolate.RectBivariateSpline(
            np.arange(wf.h), np.arange(wf.w), wf.dist, kx=3, ky=3)

    def __call__(self, q, grad=False):
        c, r = self.wf.pix(q)
        d = self.sp.ev(r, c)
        if not grad:
            return d
        return d, np.c_[self.sp.ev(r, c, dy=1), -self.sp.ev(r, c, dx=1)] / self.wf.res


def objective(Q, a):
    """目的関数と dJ/dQ。Q は周回で、点の並びを間隔 DS のパラメータとみなす。

    kappa = (d1 x d2) / |d1|^3 はパラメータの取り方によらない曲率の式で、
    d1, d2 は中心差分。弧長は |d1| DS。
    """
    h = DS
    Qp, Qm = np.roll(Q, -1, axis=0), np.roll(Q, 1, axis=0)
    d1 = (Qp - Qm) / (2 * h)
    d2 = (Qp - 2 * Q + Qm) / (h * h)
    m = np.linalg.norm(d1, axis=1)
    cr = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]
    k = cr / m ** 3
    ds = m * h

    kn = np.roll(k, -1)
    over = np.maximum(0.0, np.abs(k) - a.k_lim)
    clr, gclr = a.field(Q, grad=True)
    short = np.maximum(0.0, a.clear_target - clr)
    near = np.maximum(0.0, a.clear_near - clr)
    # 下限は罰金なのでわずかに割る。解く側だけ 1cm 内側を狙う
    hard = np.maximum(0.0, a.clear_min + 0.01 - clr)

    J = (a.w_k * np.sum(k * k * ds) + a.w_dk * np.sum((kn - k) ** 2) / h
         + a.w_over * np.sum(over * over * ds)
         + h * np.sum(a.w_clear * short ** 2 + a.w_near * near ** 2 + 1e6 * hard ** 2))
    gQ = (-h * (2 * a.w_clear * short + 2 * a.w_near * near + 2e6 * hard))[:, None] * gclr
    for p, need in a.mats:
        dm, gm = poly_dist_grad(Q, p)
        sm = np.maximum(0.0, need + 0.01 - dm)
        J += 1e6 * h * np.sum(sm * sm)
        gQ += (-2e6 * h * sm)[:, None] * gm

    # J の k, ds についての偏微分
    dJ_dk = (2 * a.w_k * k * ds + 2 * a.w_dk * ((k - kn) - (np.roll(k, 1) - k)) / h
             + 2 * a.w_over * over * np.sign(k) * ds)
    dJ_dm = h * (a.w_k * k * k + a.w_over * over * over)
    # k = cr / m^3, m = |d1|
    u = d1 / m[:, None]
    dk_dd1 = np.c_[d2[:, 1], -d2[:, 0]] / (m ** 3)[:, None] - (3 * k / m)[:, None] * u
    dk_dd2 = np.c_[-d1[:, 1], d1[:, 0]] / (m ** 3)[:, None]
    g1 = dJ_dk[:, None] * dk_dd1 + dJ_dm[:, None] * u
    g2 = dJ_dk[:, None] * dk_dd2
    # d1_i = (Q_{i+1} - Q_{i-1}) / 2h,  d2_i = (Q_{i+1} - 2Q_i + Q_{i-1}) / h^2
    gQ += (np.roll(g1, 1, axis=0) - np.roll(g1, -1, axis=0)) / (2 * h)
    gQ += (np.roll(g2, 1, axis=0) - 2 * g2 + np.roll(g2, -1, axis=0)) / (h * h)
    return float(J), gQ


def solve(C, a, nc):
    t = tangents(C)
    nrm = np.c_[-t[:, 1], t[:, 0]]                          # 左向き法線
    B = periodic_basis(len(C), nc)

    def fun(c):
        Q = C + (B @ c)[:, None] * nrm
        J, gQ = objective(Q, a)
        return J, B.T @ np.sum(gQ * nrm, axis=1)

    res = optimize.minimize(fun, np.zeros(nc), jac=True, method="L-BFGS-B",
                            options={"maxiter": 50000, "maxfun": 10 ** 6,
                                     "ftol": 1e-15, "gtol": 1e-9})
    return C + (B @ res.x)[:, None] * nrm, res


def centerline(P, field, iters=4):
    """各点の法線上で壁から一番遠い点へ寄せ、なめらかにつないだ線。初期値に使う。"""
    off = np.arange(-1.0, 1.0 + 1e-9, 0.01)
    z = len(off) // 2
    for _ in range(iters):
        t = tangents(P)
        nrm = np.c_[-t[:, 1], t[:, 0]]
        D = np.array([field(P + o * nrm) for o in off])
        alpha = np.zeros(len(P))
        for i in range(len(P)):
            free = D[:, i] > 0.1                # 隣の通路まで飛ばないよう、連続区間だけ見る
            lo = hi = z
            while lo > 0 and free[lo - 1]:
                lo -= 1
            while hi < len(off) - 1 and free[hi + 1]:
                hi += 1
            alpha[i] = off[lo + np.argmax(D[lo:hi + 1, i])]
        alpha = ndimage.gaussian_filter1d(alpha, 0.6 / DS, mode="wrap")
        P = resample_closed(P + alpha[:, None] * nrm)
        P = resample_closed(ndimage.gaussian_filter1d(P, 0.3 / DS, axis=0, mode="wrap"))
    return P


def start_at(P, q):
    """q に最も近い点が先頭になるよう並べ替える(s=0 の位置を保つ)。"""
    return np.roll(P, -int(np.argmin(np.linalg.norm(P - q, axis=1))), axis=0)


def stats(P, field, mats):
    k = kappa_avg(P)
    sg = np.sign(k[np.abs(k) > 0.15])
    dk = np.diff(np.r_[k, k[:1]]) / DS
    return {
        "length": float(np.linalg.norm(np.roll(P, -1, 0) - P, axis=1).sum()),
        "kmax": float(np.abs(k).max()),
        "k2": float(np.sum(k * k) * DS),
        "dk": float(np.sum(np.abs(dk)) * DS),
        "flips": int(np.sum(sg != np.roll(sg, 1))),
        "clr": field(P),
        "mat": {n: float((dist_to_poly(P, p) - need).min()) for n, (p, need) in mats.items()},
    }


def draw_png(path, field, P0, P1, clear_min, polys, walls_csv):
    sc = 3
    img = Image.fromarray(field.raw).convert("RGB")
    img = img.resize((field.w * sc, field.h * sc), Image.NEAREST)
    d = ImageDraw.Draw(img)

    def xy(q):
        c, r = field.pix(q)
        return list(zip(c * sc, r * sc))

    for poly in polys.values():
        d.polygon(xy(poly), outline=(0, 150, 220))
    for r in read_csv_rows(walls_csv) if walls_csv else []:
        d.line(xy(np.array([[float(r["x1"]), float(r["y1"])],
                            [float(r["x2"]), float(r["y2"])]])), fill=(230, 120, 0), width=3)
    d.line(xy(np.vstack([P0, P0[:1]])), fill=(150, 150, 150), width=2)
    clr = field(P1)
    pts = xy(np.vstack([P1, P1[:1]]))
    for i in range(len(P1)):
        col = (220, 0, 0) if clr[i] < clear_min + 0.05 else (0, 160, 0)
        d.line([pts[i], pts[i + 1]], fill=col, width=3)
    for i in range(0, len(P1), 40):
        d.text(pts[i], f"{i * DS:.0f}", fill=(0, 0, 255))
    img.save(path)


def report(name, st, need):
    c = st["clr"]
    i = int(np.argmin(c))
    print(f"[{name}] 周長 {st['length']:.2f} m / 最小 R {1 / st['kmax']:.3f} m"
          f" / ∫k^2 {st['k2']:.2f} / ∫|dk/ds| {st['dk']:.2f} / 曲率の符号反転 {st['flips']} 回")
    print(f"[{name}] 壁余裕 最小 {c.min():.3f} m (s={i * DS:.1f}m) / "
          f"{need + 0.05:.2f}m 未満の長さ {np.sum(c < need + 0.05) * DS:.1f} m / "
          f"0.40m 未満 {np.sum(c < 0.40) * DS:.1f} m")
    for n, v in st["mat"].items():
        print(f"[{name}] {n} 必要距離に対する余り {v:+.3f} m")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--raceline", default=os.path.join(ROOT, "maps/raceline.csv"))
    ap.add_argument("--map", default=os.path.join(ROOT, "maps/track_v1.yaml"))
    ap.add_argument("--extra-walls", default=os.path.join(ROOT, "maps/track_v1_extra_walls.csv"),
                    help="地図に足す壁線 CSV (name,x1,y1,x2,y2 / map 座標)。'' で無し")
    ap.add_argument("--low-mu", default=os.path.join(ROOT, "maps/track_v1_low_mu.csv"),
                    help="踏まない領域の輪郭 CSV (name,i,x,y / map 座標)。'' で無し")
    ap.add_argument("--vehicle", default=os.path.join(
        ROOT, "src/minicar_bringup/config/vehicle_params.yaml"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--png", default=None)
    ap.add_argument("--r-target", type=float, default=1.0,
                    help="線に許す最小半径[m]。r_min 0.83 に 2 割の余裕(relax_corners と同じ)")
    ap.add_argument("--wall-margin", type=float, default=0.15,
                    help="壁余裕の下限 = half_width + これ[m]")
    ap.add_argument("--clear-target", type=float, default=0.45,
                    help="壁から車体中心までの目標[m]。取れる値より大きめに置く")
    ap.add_argument("--w-clear", type=float, default=1e4)
    ap.add_argument("--clear-near", type=float, default=0.32,
                    help="これより壁に近いと急に強く押し返す[m]。狭い通路で真ん中を通らせる")
    ap.add_argument("--w-near", type=float, default=1e6)
    ap.add_argument("--w-dk", type=float, default=300.0,
                    help="曲率変化率の重み。大きいほどくねらず、コーナーの入りが緩やかになる")
    ap.add_argument("--w-k", type=float, default=1.0,
                    help="曲率の二乗積分の重み。大きいほど通路幅を使って R を稼ぐ(壁に寄る)")
    ap.add_argument("--w-over", type=float, default=1e6)
    ap.add_argument("--knot", type=float, default=0.4, help="横ずれ B スプラインの節点間隔[m]")
    ap.add_argument("--passes", type=int, default=3)
    ap.add_argument("--min-blob", type=int, default=20)
    a = ap.parse_args(argv)
    if os.path.abspath(a.out) == os.path.abspath(a.raceline):
        raise SystemExit("--out が --raceline と同じ。元の線を残すため別の名前にする")

    with open(a.vehicle, encoding="utf-8") as f:
        veh = yaml.safe_load(f)
    hw = float(veh["vehicle"]["half_width"])
    r_min = float(veh["limits"]["r_min"])
    if a.r_target < r_min:
        raise SystemExit(f"--r-target {a.r_target} < r_min {r_min}")
    a.k_lim = 1.0 / a.r_target
    a.clear_min = hw + a.wall_margin

    wf = WallField(a.map, a.min_blob)
    add_walls(wf, a.extra_walls)
    a.field = Field(wf)
    polys = load_polys(a.low_mu)
    mats = {n: (p, hw + MAT_MARGIN.get(n, MAT_MARGIN_DEFAULT)) for n, p in polys.items()}
    a.mats = list(mats.values())

    _, x, y, _, _ = read_raceline_csv(a.raceline, closed=True)
    P0 = np.c_[x, y]
    print(f"目標 R {a.r_target} m / 壁余裕 下限 {a.clear_min:.3f} m, 目標 {a.clear_target} m")
    report("元", stats(P0, a.field, mats), a.clear_min)

    P = start_at(centerline(P0, a.field), P0[0])
    for it in range(a.passes):
        nc = int(round(len(P) * DS / a.knot))
        # 上限は罰金なのでわずかに超える。解く側だけ 2% 内側を狙う
        k_lim, a.k_lim = a.k_lim, a.k_lim * 0.98
        Q, res = solve(P, a, nc)
        a.k_lim = k_lim
        P = start_at(resample_closed(Q), P0[0])
        print(f"  pass {it + 1}: {res.message} (評価 {res.nfev} 回)")
    st = stats(P, a.field, mats)
    report("新", st, a.clear_min)

    if st["kmax"] > a.k_lim * 1.005:
        raise SystemExit(f"NG: R {1 / st['kmax']:.3f} m < 目標 {a.r_target} m")
    if st["clr"].min() < a.clear_min:
        raise SystemExit("NG: 壁余裕が下限を割った")
    if any(v < 0.0 for v in st["mat"].values()):
        raise SystemExit("NG: マットに近づきすぎた")

    write_raceline_csv(a.out, P[:, 0], P[:, 1], yaw_of(P))
    print(f"書き出し: {a.out}")
    if a.png:
        draw_png(a.png, wf, P0, P, a.clear_min, polys, a.extra_walls)
        print(f"図: {a.png}")


if __name__ == "__main__":
    main()
