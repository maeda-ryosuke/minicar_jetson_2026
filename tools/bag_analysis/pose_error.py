#!/usr/bin/env python3
"""実走bagの自己位置を周回ごとに地図・追従経路と重ねて、誤差の出た場所を探す(ホストで実行)。

    python3 tools/bag_analysis/pose_error.py bags/<bag> [--raceline maps/raceline.csv]
    python3 tools/bag_analysis/pose_error.py bags/<bag> --tf-csv <再生結果>/tf.csv --out <出力先> --label AMCL

先に extract_run.py で <bag>/analysis/run.npz を作っておく。
--tf-csv を渡すと、自己位置を bag 再生で得た map->odom (monitor.py の tf.csv) と bag の odom->base_link の
合成に差し替える(AMCL などの再生結果を同じ指標で評価する)。周回の区切りは常に走行時の自己位置で決めるので、
条件の間で同じ区間を比べられる。要約は <出力先>/summary.json。

正解軌跡は無いので次の3つで見る。
  - SLAM自己位置: map->odom (slam_toolbox) と odom->base_link (EKF) の合成。
  - EKFのみ: 各周の開始時のSLAM姿勢に、EKFの移動量だけを積んだ軌跡。SLAMとの差がその周のドリフト。
  - 地図との一致率: 各scanをSLAM姿勢で地図に重ね、壁から5cm以内に入った点の割合。
    下がった所はSLAM自体が誤っている。
周回は追従経路(raceline.csv)上の進みで数え、経路の始点を通過したところで区切る
(raceline_manager と同じ数え方)。bag記録前から走っていれば最初の周は途中から始まる。
地図は既定で maps/track_v1.yaml を使う。bagの /map は、localization中のscanが足されて
壁が二重になった所があり、一致率の基準にならない。
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'Noto Sans CJK TC', 'sans-serif']
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

LX, LYAW = 0.332, -0.0349066   # base_link -> laser_frame (bagの/tf_staticと一致)
FOV, RMAX = 120.0, 2.0          # 一致率に使うscan: 前方±deg・距離[m]。SLAMの入力(scan_filter)と揃える
INLIER = 0.05                   # 地図の壁に一致したとみなす距離[m]
CAP = 0.3
LOW = 0.40                      # これ未満の一致率を「低一致」として印を付ける
JUMP_M, JUMP_DEG = 0.05, 2.0    # 0.2秒で map->odom がこれ以上動いたら補正ジャンプ
DT = 0.05                       # 解析の時間刻み[s]

INK, MUTED = '#0b0b0b', '#52514e'
C_SLAM, C_EKF, C_PATH = '#2a78d6', '#eb6834', '#8a8984'
C_JUMP, C_LOW = '#4a3aa7', '#e34948'


def uniq(a):
    """時刻が重複・逆行する行を落とす(slam_toolboxは同じstampのmap->odomを何度も出す)。"""
    t = a[:, 0]
    keep = np.r_[np.diff(t) > 0, True]
    return a[keep]


def interp_pose(a, t):
    return (np.interp(t, a[:, 0], a[:, 1]), np.interp(t, a[:, 0], a[:, 2]),
            np.interp(t, a[:, 0], np.unwrap(a[:, 3])))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    c, s = np.cos(ath), np.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


def inverse(a):
    x, y, th = a
    c, s = np.cos(th), np.sin(th)
    return -c * x - s * y, s * x - c * y, -th


def wrap_deg(a):
    return np.degrees((a + np.pi) % (2 * np.pi) - np.pi)


class Grid:
    def __init__(self, f, map_yaml=None):
        if map_yaml:
            # 保存済み地図(.pgm: 0=壁, 254=空き, 205=未知)を OccupancyGrid と同じ値・向きに直す
            info = yaml.safe_load(Path(map_yaml).read_text())
            img = np.array(Image.open(Path(map_yaml).parent / info['image']))[::-1]
            p = (255 - img) / 255.0
            self.g = np.where(p > info['occupied_thresh'], 100, np.where(p < info['free_thresh'], 0, -1)).astype(np.int8)
            self.res = float(info['resolution'])
            self.ox, self.oy = info['origin'][:2]
        else:
            self.g = f['map']
            self.res = float(f['map_res'])
            self.ox, self.oy = f['map_origin'][:2]
        self.H, self.W = self.g.shape
        self.dt = ndimage.distance_transform_edt(self.g < 50) * self.res
        self.extent = (self.ox, self.ox + self.W * self.res, self.oy, self.oy + self.H * self.res)

    def match(self, P):
        j = ((P[:, 0] - self.ox) / self.res).astype(int)
        i = ((P[:, 1] - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.H) & (j >= 0) & (j < self.W)
        d = np.full(len(P), CAP)
        d[ok] = np.minimum(self.dt[i[ok], j[ok]], CAP)
        return (d <= INLIER).mean()

    def image(self):
        img = np.full(self.g.shape, 0.86)       # 未知
        img[self.g == 0] = 1.0                  # 空き
        img[self.g >= 50] = 0.25                # 壁
        return img


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
    lat = np.hypot(rl[idx, 0] - px, rl[idx, 1] - py)
    return lap, idx, lat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--raceline', default='maps/raceline.csv')
    ap.add_argument('--map', default='maps/track_v1.yaml',
                    help='localizationが照合に使った地図。空文字ならbagの最後の/map(走行中のscanが足されたもの)')
    ap.add_argument('--tf-csv', default='', help='再生で得た map->odom (monitor.py の tf.csv)')
    ap.add_argument('--out', default='', help='出力先(既定 <bag>/analysis)')
    ap.add_argument('--label', default='SLAM', help='図の凡例に出す自己位置の名前')
    args = ap.parse_args()
    bag = Path(args.bag)
    f = np.load(bag / 'analysis/run.npz')
    out = Path(args.out) if args.out else bag / 'analysis'
    out.mkdir(parents=True, exist_ok=True)
    name = args.label
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    t_bag = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
    rl = np.loadtxt(args.raceline, delimiter=',', skiprows=1)
    m = Grid(f, args.map)

    mo_run, ob = uniq(f['tf_map_odom']), uniq(f['tf_odom_base'])
    mo = mo_run
    if args.tf_csv:
        raw = np.genfromtxt(args.tf_csv, delimiter=',', names=True)
        mo = uniq(np.c_[raw['t'], raw['mo_x'], raw['mo_y'], raw['mo_yaw']])
    ekf = uniq(f['ekf'])
    t = np.arange(max(mo[0, 0], ob[0, 0], mo_run[0, 0]), min(mo[-1, 0], ob[-1, 0], mo_run[-1, 0]), DT)
    MO, OB = interp_pose(mo, t), interp_pose(ob, t)
    slam = compose(MO, OB)
    v = np.interp(t, ekf[:, 0], np.abs(ekf[:, 4]))
    wz = np.degrees(np.interp(t, ekf[:, 0], np.abs(ekf[:, 5])))
    run_pose = compose(interp_pose(mo_run, t), OB)      # 走行時の自己位置。周回の区切りに使う
    lap, _, _ = lap_split(run_pose[0], run_pose[1], rl)
    _, ridx, lat = lap_split(slam[0], slam[1], rl)
    tr = t - t_bag

    # 各周の開始SLAM姿勢にEKFの移動量だけを積む
    ekf_only = [np.empty_like(t) for _ in range(3)]
    for L in np.unique(lap):
        s = np.where(lap == L)[0]
        k0 = s[0]
        rel = compose(inverse((OB[0][k0], OB[1][k0], OB[2][k0])), (OB[0][s], OB[1][s], OB[2][s]))
        e = compose((slam[0][k0], slam[1][k0], slam[2][k0]), rel)
        for c in range(3):
            ekf_only[c][s] = e[c]
    drift = np.hypot(ekf_only[0] - slam[0], ekf_only[1] - slam[1])
    drift_yaw = np.abs(wrap_deg(ekf_only[2] - slam[2]))

    # 補正ジャンプ: 0.2秒ごとの map->odom の変化
    k = int(round(0.2 / DT))
    dmo = np.r_[np.zeros(k), np.hypot(MO[0][k:] - MO[0][:-k], MO[1][k:] - MO[1][:-k])]
    dmy = np.r_[np.zeros(k), np.degrees(np.abs(MO[2][k:] - MO[2][:-k]))]
    jump = (dmo > JUMP_M) | (dmy > JUMP_DEG)
    jump_i = [i for i in np.where(jump)[0] if not jump[max(0, i - 2 * k):i].any()]   # 連続は先頭だけ

    # 地図との一致率(scan時刻のSLAM姿勢で)
    st = f['scan_t']
    ang = f['angle_min'] + np.arange(f['ranges'].shape[1]) * f['angle_inc']
    fov = np.abs(ang) <= math.radians(FOV)
    ang = ang + LYAW
    sel = (st >= t[0]) & (st <= t[-1])
    st, R = st[sel], f['ranges'][sel]
    sp = (np.interp(st, t, slam[0]), np.interp(st, t, slam[1]), np.interp(st, t, slam[2]))
    match = np.empty(len(st))
    for i in range(len(st)):
        r = R[i]
        ok = fov & np.isfinite(r) & (r >= 0.1) & (r <= RMAX)
        lx, ly = LX + r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])
        c, s = math.cos(sp[2][i]), math.sin(sp[2][i])
        match[i] = m.match(np.c_[sp[0][i] + c * lx - s * ly, sp[1][i] + s * lx + c * ly])
    slap = lap[np.clip(np.searchsorted(t, st), 0, len(t) - 1)]
    sv = np.interp(st, t, v)
    low = (match < LOW) & (sv > 0.05)

    # ---- 周回ごとの重ね描き
    img = m.image()
    laps = np.unique(lap)
    summary = {'label': name, 'tf_csv': args.tf_csv, 'laps': []}
    print(f'[{name}] 周回数 {len(laps)} (bag開始 {t_bag:.1f}, 経路始点 ({rl[0, 0]:.2f}, {rl[0, 1]:.2f}))')
    print(f'{"周":>2} {"時刻[s]":>13} {"ドリフト最大":>10} {"周末":>6} {"yaw最大":>7} {"一致率中央":>8} {"低一致":>6} {"ジャンプ":>5} {"経路横ずれ平均":>10}')
    for L in laps:
        s = lap == L
        ss = slap == L
        a, b = tr[s][0], tr[s][-1]
        J = [i for i in jump_i if lap[i] == L]
        fig, ax = plt.subplots(figsize=(11, 7.2), dpi=130)
        ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower', extent=m.extent, interpolation='nearest')
        ax.plot(np.r_[rl[:, 0], rl[0, 0]], np.r_[rl[:, 1], rl[0, 1]], color=C_PATH, lw=1.5, ls=(0, (4, 3)), label='追従path (raceline.csv)')
        ax.plot(ekf_only[0][s], ekf_only[1][s], color=C_EKF, lw=2, label=f'EKFのみ (周の開始で{name}に合わせる)')
        ax.plot(slam[0][s], slam[1][s], color=C_SLAM, lw=2, label=f'{name}自己位置 (map→base_link)')
        ax.plot(*rl[0, :2], marker='s', ms=8, mfc='white', mec=INK, ls='none', label='経路始点(周の区切り)')
        if J:
            ax.plot(slam[0][J], slam[1][J], marker='D', ms=8, mfc='none', mec=C_JUMP, mew=2, ls='none',
                    label=f'{name}補正ジャンプ (0.2s で >{JUMP_M * 100:.0f}cm or >{JUMP_DEG:.0f}°)')
            for i in J:
                ax.annotate(f'{tr[i]:.0f}s', (slam[0][i], slam[1][i]), xytext=(6, 6), textcoords='offset points',
                            fontsize=8, color=INK)
        lw_ = low & ss
        if lw_.any():
            ax.plot(sp[0][lw_], sp[1][lw_], marker='o', ms=8, mfc='none', mec=C_LOW, mew=1.5, ls='none',
                    label=f'地図との一致率 < {LOW * 100:.0f}%')
        ax.set_title(f'{bag.name} {name} ({Path(args.map).stem or "bag /map"})  周 {L + 1}   t = {a:.0f}–{b:.0f} s   '
                     f'ドリフト最大 {drift[s].max():.2f} m / 一致率中央 {np.median(match[ss]) * 100:.0f}%',
                     fontsize=11, color=INK, loc='left')
        ax.set_xlabel('x [m] (map)', color=MUTED)
        ax.set_ylabel('y [m] (map)', color=MUTED)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=8, frameon=False)
        fig.tight_layout()
        fig.savefig(out / f'overlay_lap{L + 1}.png')
        plt.close(fig)
        mv = ss & (sv > 0.05)
        summary['laps'].append({'lap': int(L + 1), 't': [round(float(a), 1), round(float(b), 1)],
                                'match_med': round(float(np.median(match[mv])) * 100, 1),
                                'match_p10': round(float(np.percentile(match[mv], 10)) * 100, 1),
                                'low_pct': round(float((match[mv] < LOW).mean()) * 100, 1),
                                'drift_max': round(float(drift[s].max()), 3), 'jumps': len(J),
                                'lat_mean': round(float(lat[s & (v > 0.05)].mean()), 3)})
        print(f'{L + 1:>2} {a:6.1f}-{b:6.1f} {drift[s].max():9.2f}m {drift[s][-1]:5.2f}m {drift_yaw[s].max():6.1f}° '
              f'{np.median(match[ss]) * 100:8.0f}% {lw_.sum():6d} {len(J):5d} {lat[s & (v > 0.05)].mean():9.3f}m')

    # ---- 時系列
    fig, axs = plt.subplots(5, 1, figsize=(13, 10), dpi=120, sharex=True)
    rows = [(drift, 'EKFのみとの差 [m]', C_EKF), (drift_yaw, 'yaw差 [deg]', C_EKF),
            (None, '地図との一致率 [%]', C_SLAM), (v, '速度 [m/s]', MUTED), (wz, '|角速度| [deg/s]', MUTED)]
    for ax, (y, lab, col) in zip(axs, rows):
        if y is None:
            ax.plot(st - t_bag, match * 100, color=col, lw=1)
            ax.axhline(LOW * 100, color=C_LOW, lw=1, ls=':')
        else:
            ax.plot(tr, y, color=col, lw=1.2)
        ax.set_ylabel(lab, fontsize=9, color=MUTED)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(color='#e6e5e0', lw=0.6)
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
        for L in laps[1:]:
            ax.axvline(tr[lap == L][0], color=INK, lw=0.8, ls='--')
    for i in jump_i:
        axs[0].axvline(tr[i], color=C_JUMP, lw=0.8, alpha=0.6)
    axs[0].set_title(f'{bag.name} {name}  破線=周の区切り  紫線=補正ジャンプ', fontsize=10, loc='left', color=INK)
    axs[-1].set_xlabel('bag開始からの時間 [s] (ros2 bag play --start-offset に使える)', color=MUTED)
    fig.tight_layout()
    fig.savefig(out / 'timeseries.png')
    plt.close(fig)

    # ---- 確認候補の一覧
    mv = sv > 0.05
    summary.update({
        'match_med': round(float(np.median(match[mv])) * 100, 1),
        'match_p10': round(float(np.percentile(match[mv], 10)) * 100, 1),
        'low_pct': round(float((match[mv] < LOW).mean()) * 100, 1),
        'jumps': len(jump_i), 'jumps_10cm': int(sum(dmo[i:i + 2 * k].max() > 0.10 for i in jump_i)),
        'max_jump_m': round(float(dmo.max()), 3), 'max_jump_deg': round(float(dmy.max()), 2),
        'lat_mean': round(float(lat[v > 0.05].mean()), 3), 'drift_max': round(float(drift.max()), 3),
        'series': {'t': [round(float(x), 2) for x in (st - t_bag)[::2]],
                   'match': [round(float(x) * 100, 1) for x in match[::2]]},
        'traj': {'t': [round(float(x), 2) for x in tr[::4]], 'x': [round(float(x), 3) for x in slam[0][::4]],
                 'y': [round(float(x), 3) for x in slam[1][::4]]}})
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False))
    print(f'全体(走行中) 一致率中央 {summary["match_med"]}% p10 {summary["match_p10"]}% 低一致 {summary["low_pct"]}% '
          f'ジャンプ {summary["jumps"]} 最大 {summary["max_jump_m"]}m/{summary["max_jump_deg"]}° 経路横ずれ {summary["lat_mean"]}m')
    print(f'\n{name}補正ジャンプ {len(jump_i)} 件(大きい順に上位15)')
    for i in sorted(jump_i, key=lambda i: -dmo[i])[:15]:
        print(f'  t={tr[i]:6.1f}s 周{lap[i] + 1} ({slam[0][i]:5.2f},{slam[1][i]:5.2f}) '
              f'補正 {dmo[i] * 100:4.1f}cm {dmy[i]:4.1f}°  v={v[i]:.2f} wz={wz[i]:4.0f}°/s')
    print(f'\n地図との一致率が低い区間(走行中、< {LOW * 100:.0f}%)')
    seg = np.split(np.where(low)[0], np.where(np.diff(np.where(low)[0]) > 3)[0] + 1) if low.any() else []
    for g in sorted(seg, key=lambda g: match[g].min())[:15]:
        i = g[np.argmin(match[g])]
        print(f'  t={st[g[0]] - t_bag:6.1f}-{st[g[-1]] - t_bag:6.1f}s 周{slap[i] + 1} ({sp[0][i]:5.2f},{sp[1][i]:5.2f}) '
              f'最低 {match[i] * 100:3.0f}%  v={sv[i]:.2f} wz={np.interp(st[i], t, wz):4.0f}°/s')
    print(f'\n出力: {out}/overlay_lap*.png, {out}/timeseries.png')


if __name__ == '__main__':
    main()
