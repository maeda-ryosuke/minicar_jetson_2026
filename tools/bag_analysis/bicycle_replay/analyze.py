#!/usr/bin/env python3
"""extract.py の data.npz から、PLAN.md Step 8 の確認項目を判定し、図を描く(ホストで実行)。

    python3 tools/bag_analysis/bicycle_replay/analyze.py bags/20261105_fast4 \
      --zones tools/bag_analysis/bicycle_replay/zones_fast4.yaml

出力は <bag>/analysis/bicycle/ の results.json と図(png)。
  1. target_speed: bag の自己位置と zones.yaml から計算した期待値と、記録した /target_speed の一致
  2. MPPI の v: /target_speed へ accel_max / decel_max のランプになっているか
  3. MPPI の δ: atan(ω·L/v) が ±steer_max に収まっているか
  4. 軌道: 最適軌道が自己位置から始まり、経路に沿い、曲率が tan(steer_max)/L 以下で、NaN がないか
"""
import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src/speed_zone_publisher"))
from speed_zone_publisher.zones import load_zones, target_speed  # noqa: E402

# my_mppi_params.yaml と vehicle_params.yaml の値(判定の基準)
L = 0.257
STEER_MAX = 0.30
ACCEL_MAX = 2.0
DECEL_MAX = 2.0
MARGIN = 0.5
CTRL_DT = 1.0 / 30.0
MODEL_DT = 0.1
KAPPA_MAX = math.tan(STEER_MAX) / L

# 色(dataviz の参照パレット、ライト)。新旧の MPPI と目標速度・実速度
C_NEW, C_ORIG, C_TARGET, C_ACTUAL = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6"
C_ZONE, C_INK, C_MUTED, C_GRID = "#eda100", "#0b0b0b", "#52514e", "#e4e3df"
plt.rcParams.update({
    "font.family": ["Noto Sans CJK JP", "DejaVu Sans"],
    "font.size": 9, "axes.edgecolor": C_MUTED, "axes.labelcolor": C_INK,
    "xtick.color": C_MUTED, "ytick.color": C_MUTED, "axes.grid": True,
    "grid.color": C_GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "legend.frameon": False, "lines.linewidth": 1.6,
    "savefig.dpi": 150, "savefig.bbox": "tight", "figure.facecolor": "#fcfcfb",
    "axes.facecolor": "#fcfcfb",
})


def latest(table, t):
    """時刻 t 以前で最新の行(TF の lookup に Time() を渡したときと同じ)。"""
    i = np.searchsorted(table[:, 0], t, side="right") - 1
    return table[max(i, 0)]


def compose(a, b):
    """2次元の座標変換 a∘b。どちらも (x, y, yaw)。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def to_map(map_odom_row, pts):
    c, s = math.cos(map_odom_row[3]), math.sin(map_odom_row[3])
    return np.column_stack([map_odom_row[1] + c * pts[:, 0] - s * pts[:, 1],
                            map_odom_row[2] + s * pts[:, 0] + c * pts[:, 1]])


def dist_to_polyline(pts, line):
    """各点から折れ線までの距離。"""
    a, b = line[:-1], line[1:]
    ab = b - a
    ab2 = np.maximum((ab ** 2).sum(1), 1e-12)
    ap = pts[:, None, :] - a[None]
    u = np.clip((ap * ab[None]).sum(2) / ab2[None], 0, 1)
    proj = a[None] + u[..., None] * ab[None]
    return np.sqrt(((pts[:, None, :] - proj) ** 2).sum(2)).min(1)


def within_plan(o, plan):
    """最適軌道のうち、経路の長さの内側にある点(走った距離が経路の全長 - 0.2 m 以下)。
    経路は raceline_manager が lookahead_m(4 m)ぶんしか送らないので、高速では予測の
    終わりが経路の先へはみ出し、そこで経路の終点からの距離が伸びる"""
    plan_len = np.linalg.norm(np.diff(plan, axis=0), axis=1).sum()
    run = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(o, axis=0), axis=1))]
    return o[run <= plan_len - 0.2]


def curvature(p):
    """隣り合う区間の向きの変化 ÷ 後ろの区間の長さ。1つ目の向きの変化は実測のヨーレートで
    積分されている(制御で決まらない)ので除く。区間が 1 cm 未満の所も除く。"""
    seg = np.diff(p, axis=0)
    length = np.linalg.norm(seg, axis=1)
    heading = np.arctan2(seg[:, 1], seg[:, 0])
    dh = np.angle(np.exp(1j * np.diff(heading)))
    ok = (length[:-1] > 0.01) & (length[1:] > 0.01)
    ok[0] = False
    return np.abs(dh[ok] / length[1:][ok])


def dist_to_zone_edge(zones, x, y):
    """点から最も近いゾーンの辺までの距離。"""
    best = np.inf
    for z in zones:
        poly = np.array(z.polygon + z.polygon[:1])
        best = min(best, dist_to_polyline(np.array([[x, y]]), poly)[0])
    return best


def zone_patches(ax, zones):
    for z in zones:
        poly = np.array(z.polygon + z.polygon[:1])
        ax.fill(poly[:, 0], poly[:, 1], color=C_ZONE, alpha=0.12, lw=0)
        ax.plot(poly[:, 0], poly[:, 1], color=C_ZONE, lw=1.2)
        ax.text(poly[:-1, 0].mean(), poly[:-1, 1].max() + 0.1, f"{z.name} {z.speed:.1f} m/s",
                ha="center", va="bottom", fontsize=8, color=C_INK, clip_on=True)


def draw_map(ax, d, extent):
    grid, (res, ox, oy) = d["grid"], d["grid_meta"]
    img = np.where(grid == 100, 0.35, np.where(grid == 0, 1.0, 0.85))
    h, w = grid.shape
    ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower",
              extent=[ox, ox + w * res, oy, oy + h * res], interpolation="nearest")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.grid(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--zones", required=True)
    ap.add_argument("--eval-window", type=float, nargs=2, metavar=("START", "END"),
                    help="経路追従を判定する区間 [s](bag の先頭から)。既定は走っている区間全体。"
                         "自己位置が飛んだ後などを除くときに使う")
    args = ap.parse_args()
    out = Path(args.bag) / "analysis/bicycle"
    d = dict(np.load(out / "data.npz", allow_pickle=True))
    default_speed, zones = load_zones(args.zones)

    mo, ob = d["map_odom"], d["odom_base"]
    t0 = ob[0, 0]
    pose_map = np.array([(r[0], *compose(latest(mo, r[0])[1:], r[1:])) for r in ob])
    odom_v = d["odom_v"]
    v_meas = lambda t: np.interp(t, odom_v[:, 0], odom_v[:, 1])  # noqa: E731
    moving = odom_v[odom_v[:, 1] > 0.3, 0]
    move_t = (moving.min(), moving.max())
    eval_t = (t0 + args.eval_window[0], t0 + args.eval_window[1]) if args.eval_window else move_t
    res = {"bag": Path(args.bag).name, "duration_s": float(ob[-1, 0] - t0),
           "moving_s": [float(move_t[0] - t0), float(move_t[1] - t0)],
           "eval_s": [float(eval_t[0] - t0), float(eval_t[1] - t0)],
           "v_max_actual": float(odom_v[:, 1].max())}

    # ---- 1. target_speed --------------------------------------------------------
    tgt = d["target"]
    exp = np.array([target_speed(default_speed, zones,
                                 *compose(latest(mo, t)[1:], latest(ob, t)[1:])[:2])
                    for t in tgt[:, 0]])
    match = np.abs(exp - tgt[:, 1]) < 1e-4
    # 食い違いはゾーンの縁をまたぐ瞬間だけのはず(TF の時刻のずれ)。縁からの距離で確かめる
    edge = [dist_to_zone_edge(zones, *compose(latest(mo, t)[1:], latest(ob, t)[1:])[:2])
            for t in tgt[~match, 0]]
    res["target"] = {"n": int(len(tgt)), "match_rate": float(match.mean()),
                     "n_mismatch": int((~match).sum()),
                     "mismatch_max_edge_dist_m": float(max(edge)) if edge else 0.0,
                     "n_transitions": int((np.diff(tgt[:, 1]) != 0).sum()),
                     "rate_hz": float((len(tgt) - 1) / (tgt[-1, 0] - tgt[0, 0])),
                     "values": sorted({round(float(v), 3) for v in tgt[:, 1]})}
    # ノードと解析で見ている TF の時刻は、map->odom(AMCL、10 Hz)の1回ぶんと
    # /target_speed の1周期(50 Hz)だけずれうる。その間に進む距離までを縁のずれとして許す
    res["target"]["edge_tol_m"] = float(res["v_max_actual"] * (0.1 + 0.02))
    res["target"]["pass"] = bool(
        res["target"]["mismatch_max_edge_dist_m"] <= res["target"]["edge_tol_m"])

    # ---- 2. v のランプ ------------------------------------------------------------
    cmd = d["new_cmd"]
    v = cmd[:, 1]
    dv = np.diff(v)
    tgt_at = np.array([latest(tgt, t)[1] if t >= tgt[0, 0] else 0.0 for t in cmd[:, 0]])
    # 実速度は controller が見た値と時刻が少しずれる(急停止では 50 ms で 1 m/s 変わる)ので、
    # 指令時刻と 50 ms 前の大きいほうを使い、さらに 0.1 m/s の余裕を見る
    vm = np.maximum(v_meas(cmd[:, 0]), v_meas(cmd[:, 0] - 0.05))
    capped = v[1:] <= vm[1:] + MARGIN + ACCEL_MAX * CTRL_DT + 0.1
    # controller_server が FollowPath の中断時に自分で出す 0 指令(MPPI の出力ではない)。
    # 直後に MPPI が前回の値で戻るので、前後と比べて見分ける
    injected = np.r_[False, (v[1:-1] == 0.0) & (v[:-2] > 0.0) & (v[2:] > 0.0), False]
    after_inj = np.r_[False, injected[:-1]][1:]
    up_ok = (dv <= ACCEL_MAX * CTRL_DT + 1e-4) | after_inj
    # 下げは decel_max まで。ただし起点が「実速度 + margin」で切られたとき(実速度が急に
    # 落ちたとき)は、それに合わせて下がる。0 は controller_server の 0 指令かタイムアウト
    down_ok = (dv >= -DECEL_MAX * CTRL_DT - 1e-4) | capped | (v[1:] == 0.0)
    cap_ok = (dv <= 0) | capped | after_inj
    # 目標を超えて行き過ぎない(ランプは目標で止まる)
    over = v - np.maximum(tgt_at, np.r_[v[0], v[:-1]])
    res["ramp"] = {"n": int(len(v)), "max_up_per_cycle": float(dv.max()),
                   "max_down_per_cycle": float(-dv[v[1:] > 0].min()) if (v[1:] > 0).any() else 0.0,
                   "limit_per_cycle": ACCEL_MAX * CTRL_DT,
                   "n_up_violation": int((~up_ok).sum()), "n_down_violation": int((~down_ok).sum()),
                   "n_restart_cap_violation": int((~cap_ok).sum()),
                   "n_down_by_restart_cap": int(((dv < -DECEL_MAX * CTRL_DT - 1e-4) & capped
                                                 & (v[1:] > 0)).sum()),
                   "n_injected_zero": int(injected.sum()),
                   "max_over_target": float(over.max())}
    res["ramp"]["pass"] = bool(up_ok.all() and down_ok.all() and cap_ok.all()
                               and over.max() <= 1e-4)

    # ---- 3. δ -------------------------------------------------------------------
    mv = v > 0.05
    delta = np.full(len(v), np.nan)
    delta[mv] = np.arctan(cmd[mv, 2] * L / v[mv])
    oc = d["orig_cmd"]
    omv = oc[:, 1] > 0.05
    odelta = np.full(len(oc), np.nan)
    odelta[omv] = np.arctan(oc[omv, 2] * L / oc[omv, 1])
    res["delta"] = {"max_abs": float(np.nanmax(np.abs(delta))), "steer_max": STEER_MAX,
                    "orig_max_abs": float(np.nanmax(np.abs(odelta)))}
    res["delta"]["pass"] = bool(res["delta"]["max_abs"] <= STEER_MAX + 1e-4)

    # ---- 4. 軌道 ----------------------------------------------------------------
    tt, opt, cand = d["new_traj_t"], d["new_traj_opt"], d["new_traj_cand"]
    pt, pp = d["plan_t"], d["plan_pts"]
    start_err, dev_max, dev_mean, kmax, spread, finite = [], [], [], [], [], True
    robot_off = []  # 車(元の走行)が経路から離れている距離
    for t, o, c in zip(tt, opt, cand):
        finite &= bool(np.isfinite(o).all() and np.isfinite(c).all())
        r = latest(ob, t)
        # 最適軌道の1点目は 自己位置 + 実速度·model_dt(Humble の積分の作り)
        start_err.append(float(np.hypot(o[0, 0] - r[1], o[0, 1] - r[2])
                               - abs(v_meas(t)) * MODEL_DT))
        plan = pp[np.argmin(np.abs(pt - t))]
        inside = within_plan(o, plan) if len(plan) > 1 else o[:0]
        dist = dist_to_polyline(inside, plan) if len(inside) else np.array([np.nan])
        robot_off.append(float(dist_to_polyline(np.array([r[1:3]]), plan)[0])
                         if len(plan) > 1 else np.nan)
        dev_max.append(float(dist.max()))
        dev_mean.append(float(dist.mean()))
        k = curvature(o)
        kmax.append(float(k.max()) if len(k) else 0.0)
        ends = c.reshape(-1, 5, 2)[:, -1] if len(c) % 5 == 0 and len(c) else c
        spread.append(float(np.linalg.norm(ends - o[-1], axis=1).std()) if len(ends) else 0.0)
    start_err, dev_max, dev_mean, kmax, spread, robot_off = map(
        np.array, (start_err, dev_max, dev_mean, kmax, spread, robot_off))
    mov = (tt >= eval_t[0]) & (tt <= eval_t[1])
    # 開ループなので車は元の走行のまま動く。車が経路の上(0.3 m 以内)にいる時刻だけで
    # 「最適軌道が経路に沿うか」を見る
    on = mov & (robot_off <= 0.3)
    res["traj"] = {"n": int(len(tt)), "finite": finite,
                   "max_start_err_m": float(np.abs(start_err[mov]).max()),
                   "n_on_path": int(on.sum()), "n_eval": int(mov.sum()),
                   "dev_max_median_m": float(np.median(dev_max[on])),
                   "dev_max_p95_m": float(np.percentile(dev_max[on], 95)),
                   "dev_mean_median_m": float(np.median(dev_mean[on])),
                   "kappa_max": float(kmax[mov].max()), "kappa_limit": KAPPA_MAX,
                   "kappa_max_all": float(kmax.max()),
                   "spread_median_m": float(np.median(spread[mov]))}
    res["traj"]["pass"] = bool(finite and res["traj"]["max_start_err_m"] <= 0.15
                               and res["traj"]["kappa_max"] <= KAPPA_MAX * 1.05
                               and res["traj"]["dev_max_p95_m"] <= 0.3)

    # 元の MPPI の最適軌道の経路ずれ(参考)
    ot, oo = d["orig_traj_t"], d["orig_traj_opt"]
    odev = []
    for t, o in zip(ot, oo):
        k = int(np.argmin(np.abs(tt - t)))
        if eval_t[0] <= t <= eval_t[1] and len(o) and robot_off[k] <= 0.3:
            # 元の走行の経路は bag に残っていないので、新しい経路(同じレースライン)で測る
            plan = pp[np.argmin(np.abs(pt - t))]
            inside = within_plan(o, plan)
            if len(inside):
                odev.append(float(dist_to_polyline(inside, plan).max()))
    res["traj"]["orig_dev_max_median_m"] = float(np.median(odev)) if odev else None
    res["traj"]["orig_dev_max_p95_m"] = float(np.percentile(odev, 95)) if odev else None
    (out / "results.json").write_text(json.dumps(res, ensure_ascii=False, indent=2))

    # ======== 図 ===================================================================
    rel = lambda t: np.asarray(t) - t0  # noqa: E731
    pm = pose_map
    pad = 1.0
    extent = (min(pm[:, 1].min(), min(min(p[0] for p in z.polygon) for z in zones)) - pad,
              max(pm[:, 1].max(), max(max(p[0] for p in z.polygon) for z in zones)) + pad,
              min(pm[:, 2].min(), min(min(p[1] for p in z.polygon) for z in zones)) - pad,
              max(pm[:, 2].max(), max(max(p[1] for p in z.polygon) for z in zones)) + pad)

    # 図1: 地図・ゾーン・走行軌跡(/target_speed で色分け)
    fig, ax = plt.subplots(figsize=(8, 8 * (extent[3] - extent[2]) / (extent[1] - extent[0]) + 0.6))
    draw_map(ax, d, extent)
    zone_patches(ax, zones)
    tgt_on_path = np.array([latest(tgt, t)[1] if t >= tgt[0, 0] else np.nan for t in pm[:, 0]])
    speeds = sorted({default_speed, *[z.speed for z in zones]})
    blues = ["#86b6ef", "#2a78d6", "#0d3b75"][-len(speeds):] if len(speeds) <= 3 else None
    for s_, col in zip(speeds, blues or [C_NEW] * len(speeds)):
        m = np.abs(tgt_on_path - s_) < 1e-4
        ax.scatter(pm[m, 1], pm[m, 2], s=4, color=col, label=f"/target_speed = {s_:.1f} m/s",
                   zorder=3)
    ax.scatter(pm[np.isnan(tgt_on_path), 1], pm[np.isnan(tgt_on_path), 2], s=4, color=C_MUTED,
               label="受信前", zorder=3)
    ax.plot(pm[0, 1], pm[0, 2], "o", color=C_INK, ms=6)
    ax.annotate("スタート", (pm[0, 1], pm[0, 2]), xytext=(6, 6), textcoords="offset points",
                fontsize=8)
    ax.set_xlabel("x [m](map)")
    ax.set_ylabel("y [m](map)")
    ax.legend(loc="upper left", fontsize=8, markerscale=3, frameon=True, framealpha=0.85)
    fig.savefig(out / "fig_map.png")
    plt.close(fig)

    # 図2: 速度の時系列
    fig, ax = plt.subplots(figsize=(9, 3.4))
    ax.step(rel(tgt[:, 0]), exp, where="post", color=C_INK, lw=1, ls=(0, (4, 3)),
            label="期待値(自己位置 + zones.yaml)")
    ax.step(rel(tgt[:, 0]), tgt[:, 1], where="post", color=C_TARGET, label="/target_speed")
    ax.plot(rel(cmd[:, 0]), v, color=C_NEW, label="新 MPPI の v")
    ax.plot(rel(oc[:, 0]), oc[:, 1], color=C_ORIG, lw=1.2, alpha=0.9, label="元の MPPI の v(bag)")
    ax.plot(rel(odom_v[:, 0]), odom_v[:, 1], color=C_ACTUAL, lw=1.2, label="実速度(/odometry/filtered)")
    ax.axvspan(*rel(eval_t), color=C_GRID, alpha=0.5, lw=0, zorder=0)
    ax.set_xlabel("時刻 [s](bag の先頭から)")
    ax.set_ylabel("速度 [m/s]")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=3, fontsize=8)
    fig.savefig(out / "fig_speed.png")
    plt.close(fig)

    # 図3: δ の時系列
    fig, ax = plt.subplots(figsize=(9, 2.8))
    ax.axhspan(-STEER_MAX, STEER_MAX, color=C_GRID, alpha=0.5, lw=0, zorder=0)
    for y in (-STEER_MAX, STEER_MAX):
        ax.axhline(y, color=C_MUTED, lw=1, ls=(0, (4, 3)))
    ax.text(rel(cmd[-1, 0]), STEER_MAX, " steer_max", va="bottom", ha="right", fontsize=8,
            color=C_MUTED)
    ax.plot(rel(cmd[:, 0]), delta, color=C_NEW, label="新 MPPI の δ 指令")
    ax.plot(rel(oc[:, 0]), odelta, color=C_ORIG, lw=1.2, alpha=0.9,
            label="元の MPPI の δ(atan(ωL/v)、bag)")
    ax.set_xlabel("時刻 [s](bag の先頭から)")
    ax.set_ylabel("δ [rad]")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.25), ncol=2, fontsize=8)
    fig.savefig(out / "fig_delta.png")
    plt.close(fig)

    # 図4: 軌道の指標の時系列
    fig, axs = plt.subplots(2, 1, figsize=(9, 4.6), sharex=True)
    axs[0].plot(rel(tt), dev_max, color=C_NEW, label="最大")
    axs[0].plot(rel(tt), dev_mean, color=C_NEW, lw=1, alpha=0.5, label="平均")
    axs[0].set_ylabel("経路からの距離 [m]")
    axs[0].legend(loc="upper right", ncol=2, fontsize=8)
    axs[1].plot(rel(tt), kmax, color=C_NEW)
    axs[1].axhline(KAPPA_MAX, color=C_MUTED, lw=1, ls=(0, (4, 3)))
    axs[1].text(rel(tt[0]), KAPPA_MAX, f" tan(steer_max)/L = {KAPPA_MAX:.2f}", va="bottom",
                fontsize=8, color=C_MUTED)
    axs[1].set_ylabel("最適軌道の最大曲率 [1/m]")
    axs[1].set_xlabel("時刻 [s](bag の先頭から)")
    for a in axs:
        a.axvspan(*rel(eval_t), color=C_GRID, alpha=0.5, lw=0, zorder=0)
    fig.savefig(out / "fig_traj_metrics.png")
    plt.close(fig)

    # 図5: スナップショット(走り出し・ゾーン境界の手前・境界の後・停止後)
    exp_full = np.array([target_speed(default_speed, zones, x, y) for _, x, y, _ in pm])
    snaps = [eval_t[0] + 0.8]
    for i in np.nonzero(np.diff(exp_full) != 0)[0] + 1:
        if eval_t[0] + 1.0 < pm[i, 0] < eval_t[1] - 0.3 and len(snaps) < 5:
            snaps += [pm[i, 0] - 0.25, pm[i, 0] + 0.25]
    snaps.append(eval_t[1] - 0.05)
    snaps = sorted({round(s_, 1) for s_ in snaps})[:6]
    cols = 3 if len(snaps) > 4 else 2
    rows = math.ceil(len(snaps) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(4.2 * cols, 4.0 * rows))
    axs = np.atleast_1d(axs).ravel()
    for ax, ts in zip(axs, snaps):
        k = int(np.argmin(np.abs(tt - ts)))
        t = tt[k]
        r_mo = latest(mo, t)
        o = to_map(r_mo, opt[k])
        c = to_map(r_mo, cand[k]).reshape(-1, 5, 2) if len(cand[k]) % 5 == 0 else None
        plan = to_map(r_mo, pp[int(np.argmin(np.abs(pt - t)))])
        ko = int(np.argmin(np.abs(ot - t)))
        oo_map = to_map(latest(mo, ot[ko]), oo[ko]) if len(oo[ko]) else None
        pose = compose(r_mo[1:], latest(ob, t)[1:])
        half = 2.2
        draw_map(ax, d, (pose[0] - half, pose[0] + half, pose[1] - half, pose[1] + half))
        zone_patches(ax, zones)
        if c is not None:
            for tr in c[::2]:
                ax.plot(tr[:, 0], tr[:, 1], color="#8fb3e3", lw=0.6, alpha=0.6)
        ax.plot(plan[:, 0], plan[:, 1], color=C_INK, lw=1, ls=(0, (4, 3)), label="経路")
        if oo_map is not None:
            ax.plot(oo_map[:, 0], oo_map[:, 1], color=C_ORIG, lw=1.6, label="元の最適軌道")
        ax.plot(o[:, 0], o[:, 1], color=C_NEW, lw=2.2, label="新 最適軌道")
        ax.annotate("", xy=(pose[0] + 0.35 * math.cos(pose[2]), pose[1] + 0.35 * math.sin(pose[2])),
                    xytext=pose[:2], arrowprops=dict(arrowstyle="-|>", color=C_INK, lw=1.5))
        tv = latest(tgt, t)[1] if t >= tgt[0, 0] else float("nan")
        kc = int(np.argmin(np.abs(cmd[:, 0] - t)))
        ax.set_title(f"t = {t - t0:.1f} s  実速度 {v_meas(t):.2f} / target {tv:.1f} / "
                     f"v {cmd[kc, 1]:.2f} m/s", fontsize=8)
        ax.set_xticks([])
        ax.set_yticks([])
    for ax in axs[len(snaps):]:
        ax.axis("off")
    h, lab = axs[0].get_legend_handles_labels()
    h.append(plt.Line2D([], [], color="#8fb3e3", lw=1))
    lab.append("候補軌道(一部)")
    fig.legend(h, lab, loc="lower center", ncol=4, fontsize=8, bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out / "fig_snapshots.png")
    plt.close(fig)
    res["snapshots_s"] = [float(s - t0) for s in snaps]
    (out / "results.json").write_text(json.dumps(res, ensure_ascii=False, indent=2))
    print(json.dumps(res, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
