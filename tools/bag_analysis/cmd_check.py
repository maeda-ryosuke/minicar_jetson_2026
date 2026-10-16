#!/usr/bin/env python3
"""追従経路とのずれ・cmd_vel の角速度・モータの操舵出力を同じ時間軸で並べる(ホストで実行)。

    python3 tools/bag_analysis/cmd_check.py bags/<bag> [--raceline maps/raceline.csv] [--lat-th 0.15]

先に extract_run.py で <bag>/analysis/run.npz を作っておく。モータ出力 (/motor/steer_angle,
/motor/cmd_normalized, /motor/pwm_count) は run.npz に無いので bag の db3 から直接読む。
速度は見ない。出力は <bag>/analysis/cmd/ に図と、その生データの csv。

  - 自己位置: map->odom と odom->base_link の合成 (pose_error.py と同じ)
  - 横ずれ: 経路の最近点からの距離。経路の進行方向に対して左を正
  - 角度ずれ: 自己位置の yaw - 経路の yaw。左向きを正
  - 経路に沿うのに必要な w: EKF の速度 x 経路の曲率 (参考線)
  - 操舵出力: steer_angle は motor_driver が決めた舵角 delta [rad]、u_s は正規化指令 (+1=左)、
    pwm はサーボに書いたカウント。全部 左を正 / 左のカウントが小さい。
  - 実舵角: EKF の w と v から bicycle model で逆算した atan(w L / v)。指令舵角と比べて左右の効きを見る
    (w どうしの比較は指令速度と実速度の差で歪むので、舵角で比べる)
"""
import argparse
import csv
import json
import sqlite3
import struct
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'Noto Sans CJK TC', 'sans-serif']
from matplotlib.colors import TwoSlopeNorm
import numpy as np
from PIL import Image
import yaml

DT = 0.05             # 解析の時間刻み[s]
WHEELBASE = 0.257     # [m] vehicle_params.yaml と同じ値
DELTA_MAX = 0.3003    # [rad] steer_map の端 (u=±1)
V_MIN = 0.15          # 実舵角を逆算するのに使う最低速度[m/s]。低速では w/v が暴れる
MARK_EVERY = 5.0      # 地図に cmd の w を出す間隔 [s]
H_K = 5               # 実曲率の中心差分の半幅 [サンプル] (= 0.25s)
DS_MIN = 0.03         # この距離[m]未満しか進んでいない窓は曲率を出さない (ほぼ停止)
V_EPS = 0.05          # [m/s] motor_driver の v_eps。これ未満では w/v を使わず直前の舵を保持する
K_MAX = np.tan(DELTA_MAX) / WHEELBASE   # 舵角上限での曲率 [1/m] (= 1/最小旋回半径 0.83m)

INK, MUTED = '#0b0b0b', '#52514e'
C_SLAM, C_EKF, C_PATH, C_LOW = '#2a78d6', '#eb6834', '#8a8984', '#e34948'
C_CMD, C_NEED = '#c03a8a', '#1f9e6e'


# ---- pose_error.py と同じ処理(別作業で pose_error.py が変わっても壊れないよう複製)
def uniq(a):
    """時刻が重複・逆行する行を落とす(slam_toolboxは同じstampのmap->odomを何度も出す)。"""
    keep = np.r_[np.diff(a[:, 0]) > 0, True]
    return a[keep]


def interp_pose(a, t):
    return (np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]),
            np.interp(t, a[:, 0], np.unwrap(a[:, 3])))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    c, s = np.cos(ath), np.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


def wrap_deg(a):
    return np.degrees((a + np.pi) % (2 * np.pi) - np.pi)


def lap_split(px, py, rl):
    """追従経路上の進みを積算し、経路の始点(index 0)を前向きに通過した時刻で区切る。"""
    n = len(rl)
    win = 40
    idx = np.empty(len(px), dtype=int)
    k = int(np.argmin(np.hypot(rl[:, 0] - px[0], rl[:, 1] - py[0])))
    for i, (x, y) in enumerate(zip(px, py)):
        cand = (k + np.arange(-win, win + 1)) % n
        d = np.hypot(rl[cand, 0] - x, rl[cand, 1] - y)
        if d.min() > 0.8:
            cand = np.arange(n)
            d = np.hypot(rl[:, 0] - x, rl[:, 1] - y)
        k = int(cand[np.argmin(d)])
        idx[i] = k
    step = (np.diff(idx) + n // 2) % n - n // 2
    prog = idx[0] + np.r_[0, np.cumsum(step)]
    lap = np.floor(prog / n).astype(int)
    lap -= lap[0]
    return lap, idx


class Grid:
    """保存済み地図(.pgm)を描画用に読む。"""
    def __init__(self, map_yaml):
        info = yaml.safe_load(Path(map_yaml).read_text())
        img = np.array(Image.open(Path(map_yaml).parent / info['image']))[::-1]
        p = (255 - img) / 255.0
        self.g = np.where(p > info['occupied_thresh'], 100, np.where(p < info['free_thresh'], 0, -1)).astype(np.int8)
        res = float(info['resolution'])
        ox, oy = info['origin'][:2]
        H, W = self.g.shape
        self.extent = (ox, ox + W * res, oy, oy + H * res)

    def image(self):
        img = np.full(self.g.shape, 0.86)
        img[self.g == 0] = 1.0
        img[self.g >= 50] = 0.25
        return img


def read_motor(db3):
    """/motor/* を db3 から読む。CDR の Float64 / Float64MultiArray(要素2つ) を決め打ちで解く。"""
    con = sqlite3.connect(f'file:{db3}?mode=ro', uri=True)   # 読むだけ。bag 再生と同時でも可
    ids = dict(con.execute('select name, id from topics'))
    out = {}
    for name in ('/motor/steer_angle', '/motor/cmd_normalized', '/motor/pwm_count'):
        if name not in ids:
            continue
        rows = con.execute('select timestamp, data from messages where topic_id=? order by timestamp', (ids[name],))
        a = []
        for ts, b in rows:
            if name == '/motor/steer_angle':
                a.append((ts * 1e-9, struct.unpack_from('<d', b, 4)[0]))
            else:
                # header4 | dim len4 | data_offset4 | data len4 | pad4 | double x2
                a.append((ts * 1e-9, *struct.unpack_from('<2d', b, 20)))
        out[name] = np.array(a)
    con.close()
    return out


def path_curvature(rl):
    ds = np.hypot(*np.diff(np.r_[rl[:, :2], rl[:1, :2]], axis=0).T)
    k = np.angle(np.exp(1j * np.diff(np.r_[rl[:, 2], rl[0, 2]]))) / ds
    n = 5   # 前後5点で平滑化(閉路)
    return np.convolve(np.r_[k[-n:], k, k[:n]], np.ones(2 * n + 1) / (2 * n + 1), 'valid')


def segments(mask, gap=4):
    i = np.where(mask)[0]
    if not len(i):
        return []
    return np.split(i, np.where(np.diff(i) > gap)[0] + 1)


def style(ax):
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.grid(color='#e6e5e0', lw=0.6)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--raceline', default='maps/raceline.csv')
    ap.add_argument('--map', default='maps/track_v1.yaml')
    ap.add_argument('--pwm', default='src/minicar_motor/config/pwm_params.json', help='操舵カウントの基準線(読むだけ)')
    ap.add_argument('--lat-th', type=float, default=0.15, help='この横ずれ[m]を超えた区間を塗る')
    ap.add_argument('--out', default='', help='出力先(既定 <bag>/analysis/cmd)')
    args = ap.parse_args()
    bag = Path(args.bag)
    f = np.load(bag / 'analysis' / 'run.npz')
    out = Path(args.out) if args.out else bag / 'analysis' / 'cmd'
    out.mkdir(parents=True, exist_ok=True)
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    t_bag = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    rl = np.loadtxt(args.raceline, delimiter=',', skiprows=1)
    kappa = path_curvature(rl)
    m = Grid(args.map)
    motor = read_motor(next(bag.glob('*.db3')))

    mo, ob, ekf, cmd = uniq(f['tf_map_odom']), uniq(f['tf_odom_base']), uniq(f['ekf']), f['cmd']
    t = np.arange(max(mo[0, 0], ob[0, 0]), min(mo[-1, 0], ob[-1, 0]), DT)
    px, py, pyaw = compose(interp_pose(mo, t), interp_pose(ob, t))
    lap, idx = lap_split(px, py, rl)
    tr = t - t_bag
    hx, hy = np.cos(rl[idx, 2]), np.sin(rl[idx, 2])
    lat = hx * (py - rl[idx, 1]) - hy * (px - rl[idx, 0])
    head = wrap_deg(pyaw - rl[idx, 2])
    v_ekf = np.interp(t, ekf[:, 0], ekf[:, 4])
    w_ekf = np.interp(t, ekf[:, 0], ekf[:, 5])
    w_need = v_ekf * kappa[idx]
    # cmd/motor は区分一定の指令なので、その時刻で最後に出ていた値を使う
    def hold(a, col):
        k = np.clip(np.searchsorted(a[:, 0], t, side='right') - 1, 0, len(a) - 1)
        return a[k, col]
    w_cmd = hold(cmd, 2)
    v_cmd = hold(cmd, 1)
    # 指令曲率 w/v。舵角はこれで決まる (delta = atan(w/v * L))
    k_cmd = np.where(v_cmd >= V_EPS, w_cmd / np.where(v_cmd >= V_EPS, v_cmd, 1.0), np.nan)
    nan = np.full_like(t, np.nan)
    delta = hold(motor['/motor/steer_angle'], 1) if '/motor/steer_angle' in motor else nan
    u_s = hold(motor['/motor/cmd_normalized'], 1) if '/motor/cmd_normalized' in motor else nan
    pwm_s = hold(motor['/motor/pwm_count'], 1) if '/motor/pwm_count' in motor else nan
    pwm_ref = {}
    if Path(args.pwm).exists():
        pwm_ref = json.loads(Path(args.pwm).read_text())['pwm_steering']
    big = np.abs(lat) > args.lat_th
    # 舵角で比べる(速度に依らない)。実舵角は EKF の動きから bicycle model で逆算
    moving = v_ekf > V_MIN
    d_path = np.arctan(kappa[idx] * WHEELBASE)
    d_act = np.where(moving, np.arctan(w_ekf * WHEELBASE / np.maximum(v_ekf, 1e-3)), np.nan)
    k_model = np.tan(delta) / WHEELBASE    # ドライバが出した舵角から bicycle model で期待される曲率
    # 実際に走った曲率: /odometry/filtered の姿勢から、向きの変化 / 走った距離 (中心差分 ±H_K)
    # 曲率は座標系に依らないので、SLAM の補正ジャンプが無い EKF の連続軌跡で求める
    ex, ey, eyaw = interp_pose(ekf, t)
    arc = np.r_[0.0, np.cumsum(np.hypot(np.diff(ex), np.diff(ey)))]
    k_odom = np.full_like(t, np.nan)
    ds = arc[2 * H_K:] - arc[:-2 * H_K]
    dth = np.radians(wrap_deg(eyaw[2 * H_K:] - eyaw[:-2 * H_K]))
    k_odom[H_K:-H_K] = np.where(ds >= DS_MIN, dth / np.maximum(ds, 1e-9), np.nan)
    k_twist = np.where(moving, w_ekf / np.maximum(v_ekf, 1e-3), np.nan)   # 確認用: twist の瞬時値
    k_path = kappa[idx]

    # ---- 生データ
    cols = ['t', 'lap', 'x', 'y', 'yaw_deg', 'path_idx', 'lat_m', 'head_err_deg',
            'w_cmd', 'w_ekf', 'w_need', 'steer_angle_rad', 'u_steer', 'pwm_steer', 'delta_path_rad', 'delta_act_rad',
            'v_cmd', 'kappa_cmd', 'kappa_path', 'kappa_model',
            'kappa_odom', 'kappa_ekf_twist']
    data = np.c_[tr, lap + 1, px, py, np.degrees(pyaw), idx, lat, head, w_cmd, w_ekf, w_need, delta, u_s, pwm_s,
                 d_path, d_act, v_cmd, k_cmd, k_path, k_model, k_odom, k_twist]
    fmt = ['%.2f', '%d', '%.4f', '%.4f', '%.2f', '%d', '%.4f', '%.2f', '%.4f', '%.4f', '%.4f', '%.4f', '%.4f', '%.0f',
           '%.4f', '%.4f', '%.4f', '%.4f', '%.4f', '%.4f', '%.4f', '%.4f']
    np.savetxt(out / 'cmd_timeseries.csv', data, delimiter=',', header=','.join(cols), comments='', fmt=fmt)

    # ---- 時系列(周ごと)
    laps = np.unique(lap)
    for L in laps:
        s = lap == L
        fig, axs = plt.subplots(6, 1, figsize=(15, 14), dpi=120, sharex=True)
        x = tr[s]
        axs[0].plot(x, lat[s], color=C_SLAM, lw=1.2)
        axs[0].axhline(0, color=INK, lw=0.6)
        axs[0].set_ylabel('横ずれ [m]\n(+:経路の左)', fontsize=9, color=MUTED)
        axs[1].plot(x, head[s], color=C_SLAM, lw=1.2)
        axs[1].axhline(0, color=INK, lw=0.6)
        axs[1].set_ylabel('角度ずれ [deg]\n(+:経路より左向き)', fontsize=9, color=MUTED)
        axs[2].plot(x, w_need[s], color=C_NEED, lw=1.2, ls=(0, (4, 2)), label='経路に沿うのに必要な w (v_ekf × 曲率)')
        axs[2].plot(x, w_ekf[s], color=C_EKF, lw=1.0, label='実際の w (EKF)')
        axs[2].plot(x, w_cmd[s], color=C_CMD, lw=1.4, label='cmd_vel の w')
        axs[2].axhline(0, color=INK, lw=0.6)
        axs[2].set_ylabel('角速度 [rad/s]\n(+:左)', fontsize=9, color=MUTED)
        axs[2].legend(fontsize=8, loc='upper right', ncol=3, frameon=False)
        axs[3].plot(x, k_path[s], color=C_NEED, lw=1.0, ls=(0, (4, 2)), alpha=0.7, label='経路の曲率 κ')
        axs[3].plot(x, k_model[s], color=INK, lw=1.4, label='期待: tan(steer_angle)/L (ドライバ出力)')
        axs[3].plot(x, k_odom[s], color=C_EKF, lw=1.4, label='実際: /odometry/filtered の Δyaw/Δs')
        for y_ in (K_MAX, -K_MAX):
            axs[3].axhline(y_, color=MUTED, lw=0.6, ls=':')
        axs[3].axhline(0, color=INK, lw=0.6)
        axs[3].set_ylim(-2.0, 2.0)
        axs[3].set_ylabel('曲率 [1/m]\n(+:左)', fontsize=9, color=MUTED)
        axs[3].legend(fontsize=8, loc='upper right', ncol=3, frameon=False)
        axs[4].plot(x, d_path[s], color=C_NEED, lw=1.2, ls=(0, (4, 2)), label='経路が要る舵角 atan(κL)')
        axs[4].plot(x, delta[s], color=INK, lw=1.2, label='指令舵角 steer_angle')
        axs[4].plot(x, d_act[s], color=C_EKF, lw=1.0, label=f'実舵角 atan(w_ekf·L/v_ekf) (v>{V_MIN})')
        for y_ in (DELTA_MAX, -DELTA_MAX):
            axs[4].axhline(y_, color=MUTED, lw=0.6, ls=':')
        axs[4].axhline(0, color=INK, lw=0.6)
        axs[4].set_ylim(-0.45, 0.45)
        axs[4].set_ylabel('舵角 [rad]\n(+:左)', fontsize=9, color=MUTED)
        axs[4].legend(fontsize=8, loc='upper right', ncol=3, frameon=False)
        axs[5].plot(x, pwm_s[s], color=INK, lw=1.0)
        for k_, c_ in (('left', C_SLAM), ('center', MUTED), ('right', C_LOW)):
            if k_ in pwm_ref:
                axs[5].axhline(pwm_ref[k_], color=c_, lw=0.8, ls='--')
                axs[5].annotate(f'{k_} {pwm_ref[k_]}', (x[0], pwm_ref[k_]), fontsize=7, color=c_,
                                xytext=(2, 2), textcoords='offset points')
        axs[5].invert_yaxis()   # 左(カウント小)を上に
        axs[5].set_ylabel('操舵 pwm カウント\n(上:左)', fontsize=9, color=MUTED)
        for ax in axs:
            style(ax)
            for g in segments(big & s):
                ax.axvspan(tr[g[0]], tr[g[-1]], color=C_LOW, alpha=0.10, lw=0)
        for g in segments(np.isnan(k_odom) & s, gap=1):
            axs[3].axvspan(tr[g[0]], tr[g[-1]] + DT, color=MUTED, alpha=0.18, lw=0)
        axs[0].set_title(f'{bag.name}  周 {L + 1}  赤帯 = |横ずれ| > {args.lat_th} m   灰帯(4段目) = 停止中で実曲率なし', fontsize=10, loc='left', color=INK)
        axs[-1].set_xlabel('bag開始からの時間 [s] (ros2 bag play --start-offset に使える)', color=MUTED)
        fig.tight_layout()
        fig.savefig(out / f'cmd_timeseries_lap{L + 1}.png')
        plt.close(fig)

    # ---- 地図: 5s ごとの cmd w
    marks = []
    for L in laps:
        s = np.where(lap == L)[0]
        t0 = np.ceil(tr[s[0]] / MARK_EVERY) * MARK_EVERY
        for tm in np.arange(t0, tr[s[-1]], MARK_EVERY):
            marks.append(s[np.argmin(np.abs(tr[s] - tm))])
    marks = np.array(marks)
    with open(out / 'cmd_w_5s.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['t', 'lap', 'x', 'y', 'yaw_deg', 'lat_m', 'head_err_deg', 'w_cmd', 'w_ekf', 'w_need',
                     'steer_angle_rad', 'u_steer', 'pwm_steer'])
        for i in marks:
            wr.writerow([f'{tr[i]:.2f}', lap[i] + 1, f'{px[i]:.4f}', f'{py[i]:.4f}', f'{np.degrees(pyaw[i]):.2f}',
                         f'{lat[i]:.4f}', f'{head[i]:.2f}', f'{w_cmd[i]:.4f}', f'{w_ekf[i]:.4f}', f'{w_need[i]:.4f}',
                         f'{delta[i]:.4f}', f'{u_s[i]:.4f}', f'{pwm_s[i]:.0f}'])

    img = m.image()
    wmax = max(np.nanmax(np.abs(w_cmd)), 1e-3)
    norm = TwoSlopeNorm(0, -wmax, wmax)
    cmap = plt.get_cmap('coolwarm_r')   # 左(+)=青, 右(-)=赤
    for L in laps:
        s = lap == L
        mk = marks[lap[marks] == L]
        fig, ax = plt.subplots(figsize=(13, 8.4), dpi=130)
        ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', extent=m.extent, interpolation='nearest')
        ax.plot(np.r_[rl[:, 0], rl[0, 0]], np.r_[rl[:, 1], rl[0, 1]], color=C_PATH, lw=1.5, ls=(0, (4, 3)),
                label='追従path (raceline.csv)')
        ax.plot(px[s], py[s], color=C_SLAM, lw=1.6, alpha=0.8, label='自己位置 (map→base_link)')
        ax.quiver(px[mk], py[mk], np.cos(pyaw[mk]), np.sin(pyaw[mk]), w_cmd[mk], cmap=cmap, norm=norm,
                  scale=28, width=0.004, headwidth=4, zorder=4)
        sc = ax.scatter(px[mk], py[mk], c=w_cmd[mk], cmap=cmap, norm=norm, s=40, edgecolors=INK, linewidths=0.6,
                        zorder=5, label=f'{MARK_EVERY:.0f}s ごとの cmd_vel w (矢印=車の向き)')
        for i in mk:
            ax.annotate(f'{tr[i]:.0f}s\n{w_cmd[i]:+.2f}', (px[i], py[i]), xytext=(5, 4), textcoords='offset points',
                        fontsize=7, color=INK, zorder=6)
        cb = fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.01)
        cb.set_label('cmd_vel w [rad/s]  (+:左 / -:右)', fontsize=8, color=MUTED)
        ax.set_title(f'{bag.name}  周 {L + 1}  t = {tr[s][0]:.0f}–{tr[s][-1]:.0f} s', fontsize=11, color=INK, loc='left')
        ax.set_xlabel('x [m] (map)', color=MUTED)
        ax.set_ylabel('y [m] (map)', color=MUTED)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.legend(loc='upper left', fontsize=8, frameon=True)
        fig.tight_layout()
        fig.savefig(out / f'cmd_overlay_lap{L + 1}.png')
        plt.close(fig)

    # ---- 指令曲率 w/v 単独(周ごと)。舵角上限まで切る指令が出ているか
    SAT = 0.98 * K_MAX    # これ以上を「上限まで切る指令」とみなす
    for L in laps:
        s_ = lap == L
        x = tr[s_]
        kc = k_cmd[s_]
        fig, ax = plt.subplots(figsize=(16, 4.8), dpi=120)
        ax.plot(x, kc, color=C_CMD, lw=1.4, label='cmd_vel_raw の w/v')
        ax.fill_between(x, K_MAX, np.where(kc >= SAT, kc, np.nan), color=C_SLAM, alpha=0.35, lw=0,
                        label='左の上限以上 (ドライバで頭打ち)')
        ax.fill_between(x, -K_MAX, np.where(kc <= -SAT, kc, np.nan), color=C_LOW, alpha=0.35, lw=0,
                        label='右の上限以上 (ドライバで頭打ち)')
        for y_ in (K_MAX, -K_MAX):
            ax.axhline(y_, color=INK, lw=0.8, ls=':')
        ax.annotate(f'左の上限 +{K_MAX:.2f} (δ=+{DELTA_MAX:.2f} rad, R=0.83 m)', (x[0], K_MAX), xytext=(2, 3),
                    textcoords='offset points', fontsize=8, color=INK)
        ax.annotate(f'右の上限 -{K_MAX:.2f}', (x[0], -K_MAX), xytext=(2, -11), textcoords='offset points',
                    fontsize=8, color=INK)
        ax.axhline(0, color=INK, lw=0.6)
        for g in segments((v_cmd < V_EPS) & s_, gap=1):
            ax.axvspan(tr[g[0]], tr[g[-1]] + DT, color=MUTED, alpha=0.18, lw=0)
        ax.set_ylim(-2.0, 2.0)
        ax.set_ylabel('w/v [1/m]  (+:左)', color=MUTED)
        ax.set_xlabel('bag開始からの時間 [s]', color=MUTED)
        mv = s_ & np.isfinite(k_cmd)
        pl = (k_cmd[mv] >= SAT).mean() * 100
        pr = (k_cmd[mv] <= -SAT).mean() * 100
        ax.set_title(f'{bag.name}  周 {L + 1}  cmd_vel_raw の w/v   上限まで切る指令: 左 {pl:.1f}% / 右 {pr:.1f}% の時間'
                     f'   灰帯 = v_cmd < {V_EPS} (w/v 未定義)', fontsize=10, loc='left', color=INK)
        ax.legend(fontsize=8, loc='upper right', frameon=False)
        style(ax)
        fig.tight_layout()
        fig.savefig(out / f'cmd_curvature_lap{L + 1}.png')
        plt.close(fig)
    old = out / 'cmd_curvature.png'
    if old.exists():
        old.unlink()
    print(f'上限まで切る指令 (|w/v| >= {SAT:.2f})')
    with open(out / 'cmd_saturation.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['side', 't_start', 't_end', 'lap', 'duration_s', 'w_over_v_peak'])
        for side, m_ in (('left', k_cmd >= SAT), ('right', k_cmd <= -SAT)):
            for g in segments(m_, gap=2):
                pk = k_cmd[g][np.argmax(np.abs(k_cmd[g]))]
                dur = tr[g[-1]] - tr[g[0]] + DT
                wr.writerow([side, f'{tr[g[0]]:.2f}', f'{tr[g[-1]]:.2f}', lap[g[0]] + 1, f'{dur:.2f}', f'{pk:.3f}'])
                if dur >= 0.3:
                    print(f'  {"左" if side == "left" else "右"} {tr[g[0]]:7.2f}-{tr[g[-1]]:7.2f}s 周{lap[g[0]] + 1} '
                          f'{dur:5.2f}s  最大 {pk:+.2f}')
    q = np.isfinite(k_cmd) & np.isfinite(k_model)
    dk = np.abs(np.clip(k_cmd[q], -K_MAX, K_MAX) - k_model[q])
    print(f'w/v (±{K_MAX:.2f} で頭打ち) と tan(steer_angle)/L の差: 中央値 {np.median(dk):.4f}, 95% {np.percentile(dk, 95):.4f} [1/m]'
          f'  (v_cmd < {V_EPS} の時間 {(v_cmd < V_EPS).mean() * 100:.1f}%)\n')

    # ---- 期待曲率 vs 実曲率
    # 距離スケール: 1周の走行距離を EKF と SLAM 自己位置で比べる (EKF の距離はホイール由来)
    sl = np.r_[0.0, np.cumsum(np.hypot(np.diff(px), np.diff(py)))]
    for L in laps:
        i = np.where(lap == L)[0]
        de, dsl = arc[i[-1]] - arc[i[0]], sl[i[-1]] - sl[i[0]]
        print(f'周{L + 1} 走行距離 EKF {de:.2f} m / SLAM {dsl:.2f} m  比 {de / dsl:.3f}')
    q = np.isfinite(k_odom) & np.isfinite(k_twist)
    print(f'Δyaw/Δs と twist の w/v の差: 中央値 {np.median(np.abs(k_odom[q] - k_twist[q])):.3f} [1/m]')
    print('期待曲率 tan(steer_angle)/L で分けた実曲率 (遅れ 0 / 0.5s)')
    for lag_ in (0, 10):
        km = np.r_[np.full(lag_, np.nan), k_model[:len(t) - lag_]]
        q = np.isfinite(km) & np.isfinite(k_odom)
        rows_ = []
        for lo, hi in ((-1.3, -1.1), (-1.1, -0.6), (-0.6, -0.15), (-0.15, 0.15), (0.15, 0.6), (0.6, 1.1), (1.1, 1.3)):
            b = q & (km >= lo) & (km < hi)
            if b.sum():
                rows_.append(f'[{lo:+.2f},{hi:+.2f}) n={b.sum():4d} 期待 {km[b].mean():+.2f} 実際 {np.median(k_odom[b]):+.2f}')
        print(f'  遅れ {lag_ * DT:.1f}s')
        for r_ in rows_:
            print('    ' + r_)
    print()

    # ---- 指令舵角 vs 実舵角(左右の効き)
    # サーボと車体の応答遅れぶん指令をずらして比べる。相関が最大になる遅れを 0〜1.5s で探す
    # (相関の山はなだらかで遅れの推定は粗い。左右の傾き・端での実舵角は遅れ 0〜0.7s でほぼ変わらない)
    ok = moving & np.isfinite(delta) & np.isfinite(d_act)
    lags = np.arange(0, 31)
    cc = [np.corrcoef(delta[:len(t) - k][ok[k:]], d_act[k:][ok[k:]])[0, 1] for k in lags]
    lag = int(lags[np.argmax(cc)])
    dc = np.r_[np.full(lag, np.nan), delta[:len(t) - lag]]   # lag だけ前の指令
    ok &= np.isfinite(dc)
    fit_rows = []
    bins = [-0.31, -0.2, -0.1, -0.03, 0.03, 0.1, 0.2, 0.31]
    for lo, hi in zip(bins[:-1], bins[1:]):
        q = ok & (dc >= lo) & (dc < hi)
        if q.sum():
            fit_rows.append(['bin', f'{lo:+.2f}', f'{hi:+.2f}', int(q.sum()), f'{dc[q].mean():+.4f}',
                             f'{d_act[q].mean():+.4f}', f'{(d_act[q] / dc[q]).mean() if lo >= 0.03 or hi <= -0.03 else float("nan"):.3f}'])
    side_fit = {}
    for side, q in (('left', ok & (dc > 0.03)), ('right', ok & (dc < -0.03))):
        a, b = np.polyfit(dc[q], d_act[q], 1)
        side_fit[side] = (a, b)
        fit_rows.append([f'fit_{side}', '', '', int(q.sum()), f'slope={a:.3f}', f'intercept={b:+.4f}', ''])
    sat = ok & (np.abs(dc) >= DELTA_MAX - 1e-3)
    for side, q in (('left', sat & (dc > 0)), ('right', sat & (dc < 0))):
        if q.sum():
            fit_rows.append([f'saturated_{side}', '', '', int(q.sum()), f'{dc[q].mean():+.4f}',
                             f'{np.median(d_act[q]):+.4f}', 'median of actual'])
    with open(out / 'steer_fit.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow([f'# lag={lag * DT:.2f}s, v_ekf>{V_MIN}'])
        wr.writerow(['kind', 'cmd_lo', 'cmd_hi', 'n', 'cmd_mean', 'act_mean', 'act/cmd'])
        wr.writerows(fit_rows)
    np.savetxt(out / 'steer_scatter.csv', np.c_[tr[ok], lap[ok] + 1, dc[ok], d_act[ok], d_path[ok]], delimiter=',',
               header='t,lap,delta_cmd_lagged_rad,delta_act_rad,delta_path_rad', comments='',
               fmt=['%.2f', '%d', '%.4f', '%.4f', '%.4f'])
    fig, ax = plt.subplots(figsize=(7.5, 7), dpi=130)
    ax.scatter(dc[ok], d_act[ok], s=3, alpha=0.25, color=np.where(dc[ok] > 0, C_SLAM, C_LOW), lw=0)
    xx = np.linspace(-0.32, 0.32, 2)
    ax.plot(xx, xx, color=INK, lw=0.8, ls='--', label='実舵角 = 指令舵角')
    for side, col, rng in (('left', C_SLAM, (0.03, 0.32)), ('right', C_LOW, (-0.32, -0.03))):
        a, b = side_fit[side]
        xs = np.linspace(*rng, 2)
        ax.plot(xs, a * xs + b, color=col, lw=2, label=f'{"左" if side == "left" else "右"}: 実 = {a:.2f}×指令 {b:+.3f}')
    for y_ in (DELTA_MAX, -DELTA_MAX):
        ax.axvline(y_, color=MUTED, lw=0.6, ls=':')
    ax.set_xlim(-0.35, 0.35)
    ax.set_ylim(-0.45, 0.45)
    ax.set_aspect('equal')
    ax.set_xlabel(f'指令舵角 steer_angle [rad] ({lag * DT:.2f}s 前)  +:左', color=MUTED)
    ax.set_ylabel(f'実舵角 atan(w_ekf·L/v_ekf) [rad]  (v>{V_MIN} m/s)', color=MUTED)
    ax.set_title(f'{bag.name}  指令舵角 vs 実舵角', fontsize=11, color=INK, loc='left')
    style(ax)
    ax.legend(fontsize=8, loc='upper left', frameon=False)
    fig.tight_layout()
    fig.savefig(out / 'steer_scatter.png')
    plt.close(fig)
    print(f'指令舵角 vs 実舵角 (遅れ {lag * DT:.2f}s で比較)')
    for r in fit_rows:
        print('  ' + '  '.join(str(c) for c in r))
    print()

    # ---- 横ずれ大の区間
    rows = []
    for g in segments(big & (v_ekf > 0.05)):
        if len(g) < 3:
            continue
        i = g[np.argmax(np.abs(lat[g]))]
        rows.append([f'{tr[g[0]]:.2f}', f'{tr[g[-1]]:.2f}', lap[i] + 1, f'{px[i]:.3f}', f'{py[i]:.3f}',
                     f'{lat[i]:.3f}', f'{head[g][np.argmax(np.abs(head[g]))]:.1f}',
                     f'{w_cmd[g].mean():.3f}', f'{w_need[g].mean():.3f}', f'{w_ekf[g].mean():.3f}',
                     f'{np.nanmean(delta[g]):.3f}', f'{np.nanmean(pwm_s[g]):.0f}'])
    hdr = ['t_start', 't_end', 'lap', 'x_at_max', 'y_at_max', 'lat_max_m', 'head_err_max_deg',
           'w_cmd_mean', 'w_need_mean', 'w_ekf_mean', 'steer_angle_mean', 'pwm_steer_mean']
    with open(out / 'lat_segments.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(hdr)
        wr.writerows(rows)
    print(f'横ずれ > {args.lat_th} m の区間 {len(rows)} 件')
    print(f'{"時刻[s]":>15} {"周":>2} {"横ずれ":>7} {"角度ずれ":>7} {"cmd w":>7} {"必要w":>7} {"実w":>7} {"δ":>7} {"pwm":>5}')
    for r in rows:
        print(f'{r[0]:>7}-{r[1]:>7} {r[2]:>2} {r[5]:>7} {r[6]:>7} {r[7]:>7} {r[8]:>7} {r[9]:>7} {r[10]:>7} {r[11]:>5}')
    print(f'\n出力: {out}/ (cmd_timeseries_lap*.png, cmd_curvature_lap*.png, cmd_overlay_lap*.png, steer_scatter.png, *.csv)')


if __name__ == '__main__':
    main()
