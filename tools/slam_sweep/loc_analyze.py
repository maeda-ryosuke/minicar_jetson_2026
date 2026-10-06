#!/usr/bin/env python3
"""localization結果を地図との一致度で評価する(ホストで実行)。

    python3 tools/slam_sweep/loc_analyze.py --map MAP_G4

正解軌跡は無いので、各scanを推定姿勢で地図に重ね、最寄りの占有セルから5cm以内に
入った点の割合(一致率)で評価する。通路内の床などに当たった点が常に一定数あるため、
平均距離ではなく割合を使う。比較として、開始姿勢にEKF(odometry/filtered)を積んだだけの軌跡
(EKFのみ)も同じ方法で評価する。前方240deg・2m以内に揃えて全条件を同じ点で比べる。
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage
import yaml

ROOT = Path(__file__).resolve().parents[2]
SWEEP = ROOT / 'replay_params/sweep'
FEAT = {'slow': 'feat_20261006_localization_slow.npz', 'fast': 'feat_20261006_localization_fast.npz',
        'd1005': 'features.npz'}
LX, LYAW = 0.332, -0.0349066   # base_link -> laser_frame
CAP = 0.3
INLIER = 0.05                   # 地図の壁に一致したとみなす距離[m]
LOST = 0.25                     # 一致率がこれを3秒続けて下回ったら見失いとみなす


class MapDT:
    def __init__(self, mdir):
        info = yaml.safe_load((mdir / 'map.yaml').read_text())
        self.img = np.array(Image.open(mdir / 'map.pgm'))
        self.res, (self.ox, self.oy) = info['resolution'], info['origin'][:2]
        self.H, self.W = self.img.shape
        self.dt = ndimage.distance_transform_edt(self.img >= 50) * self.res

    def resid(self, P):
        j = ((P[:, 0] - self.ox) / self.res).astype(int)
        i = (self.H - 1 - (P[:, 1] - self.oy) / self.res).astype(int)
        ok = (i >= 0) & (i < self.H) & (j >= 0) & (j < self.W)
        d = np.full(len(P), CAP)
        d[ok] = np.minimum(self.dt[i[ok], j[ok]], CAP)
        return (d <= INLIER).mean()

    def px(self, x, y):
        return (x - self.ox) / self.res, self.H - (y - self.oy) / self.res


def compose(a, b):
    """a ∘ b (2D姿勢、配列可)"""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = np.cos(ath), np.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


class Bag:
    def __init__(self, name):
        f = np.load(SWEEP / FEAT[name])
        self.st, self.R, e = f['scan_t'], f['ranges'], f['ekf']
        self.e = e
        ang = f['angle_min'] + np.arange(self.R.shape[1]) * f['angle_inc']
        self.use = np.abs(ang) <= math.radians(120)
        self.ang = ang[self.use] + LYAW
        eyaw = np.unwrap(e[:, 3])
        self.ekf = (np.interp(self.st, e[:, 0], e[:, 1]), np.interp(self.st, e[:, 0], e[:, 2]),
                    np.interp(self.st, e[:, 0], eyaw))
        self.v = np.interp(self.st, e[:, 0], np.abs(e[:, 4]))
        self.wz = np.interp(self.st, e[:, 0], np.degrees(np.abs(e[:, 5])))
        self.lap = np.floor(np.abs(self.ekf[2] - self.ekf[2][0]) / (2 * math.pi)).astype(int)

    def points(self, k, pose):
        r = self.R[k][self.use]
        ok = np.isfinite(r) & (r >= 0.1) & (r <= 2.0)
        px, py = LX + r[ok] * np.cos(self.ang[ok]), r[ok] * np.sin(self.ang[ok])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return np.c_[pose[0] + c * px - s * py, pose[1] + s * px + c * py]

    def residuals(self, m, poses, step=2):
        idx = np.arange(0, len(self.st), step)
        return idx, np.array([m.resid(self.points(k, (poses[0][k], poses[1][k], poses[2][k]))) for k in idx])


def summarize(t, res, bag, idx):
    """res: scanごとの一致率(0-1、高いほど良い)"""
    moving = bag.v[idx] > 0.05
    r = res[moving]
    laps = {}
    for L in range(int(bag.lap.max()) + 1):
        mm = moving & (bag.lap[idx] == L)
        if mm.sum() > 20:
            laps[L + 1] = round(float(np.median(res[mm])) * 100, 1)
    lost = None
    bad = res < LOST
    run = 0
    for k in range(len(bad)):
        run = run + 1 if bad[k] else 0
        if run * (t[1] - t[0]) >= 3.0:
            lost = round(float(t[k] - 3.0), 1)
            break
    return {'inlier_med': round(float(np.median(r)) * 100, 1), 'inlier_p10': round(float(np.percentile(r, 10)) * 100, 1),
            'low_pct': round(float((r < LOST).mean()) * 100, 1), 'laps': laps, 'lost_t': lost}


def draw(m, out, paths, marks=()):
    img = Image.fromarray(m.img).convert('RGB')
    d = ImageDraw.Draw(img)
    for (xs, ys), col, w in paths:
        d.line([m.px(x, y) for x, y in zip(xs[::3], ys[::3])], fill=col, width=w)
    for x, y in marks:
        cx, cy = m.px(x, y)
        d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], outline=(220, 30, 30), width=2)
    img = img.resize((img.size[0] * 2, img.size[1] * 2), Image.NEAREST)
    img.quantize(64).save(out, optimize=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='MAP_G4')
    args = ap.parse_args()
    m = MapDT(SWEEP / args.map)
    base = SWEEP / 'loc' / args.map
    results = []
    for bag_dir in sorted(p for p in base.iterdir() if p.is_dir()):
        bag = Bag(bag_dir.name)
        start = yaml.safe_load((Path(__file__).parent / 'loc_conditions.yaml').read_text())['bags'][bag_dir.name]['start']
        t0 = bag.st[0]
        ekf_only = compose(start, bag.ekf)
        idx, r_ekf = bag.residuals(m, ekf_only)
        ekf_sum = summarize(bag.st[idx] - t0, r_ekf, bag, idx)
        k2 = np.arange(0, len(bag.st), 5)   # 2Hz
        traj = lambda P: [[round(float(P[0][k]), 3) for k in k2], [round(float(P[1][k]), 3) for k in k2]]
        results.append({'map': args.map, 'bag': bag_dir.name, 'name': 'EKF_only', **ekf_sum,
                        'traj_t': [round(float(bag.st[k] - t0), 1) for k in k2], 'traj': traj(ekf_only),
                        'series': {'t': [round(float(v), 1) for v in (bag.st[idx] - t0)[::5]],
                                   'res': [round(float(v) * 100, 1) for v in r_ekf[::5]]}})
        draw(m, bag_dir / 'ekf_only.png', [((ekf_only[0], ekf_only[1]), (235, 104, 52), 2)])
        print(f'{bag_dir.name} EKF_only {ekf_sum}')
        for run in sorted(p for p in bag_dir.iterdir() if (p / 'tf.csv').exists()):
            raw = np.genfromtxt(run / 'tf.csv', delimiter=',', names=True)
            if raw.size < 20:
                continue
            tt = raw['t']
            mo = (np.interp(bag.st, tt, raw['mo_x']), np.interp(bag.st, tt, raw['mo_y']),
                  np.interp(bag.st, tt, np.unwrap(raw['mo_yaw'])))
            loc = compose(mo, bag.ekf)
            idx, r_loc = bag.residuals(m, loc)
            s = summarize(bag.st[idx] - t0, r_loc, bag, idx)
            # 補正のジャンプ: 0.2秒ごとの map->odom の変化
            dmo = np.hypot(np.diff(raw['mo_x']), np.diff(raw['mo_y']))
            dyaw = np.degrees(np.abs(np.diff(np.unwrap(raw['mo_yaw']))))
            jmask = (dmo > 0.15) | (dyaw > 5)
            jumps = [{'t': round(float(tt[i + 1] - t0), 1), 'dpos': round(float(dmo[i]), 2), 'dyaw': round(float(dyaw[i]), 1)}
                     for i in np.where(jmask)[0]]
            # EKFのみとの差(補正量)
            corr = np.hypot(loc[0] - ekf_only[0], loc[1] - ekf_only[1])
            cond = yaml.safe_load((run / 'condition.yaml').read_text())
            # 地図との一致が悪い時刻の状況(残差上位5%)
            worst = idx[r_loc <= np.percentile(r_loc, 5)]
            ctx = {'v': round(float(bag.v[worst].mean()), 2), 'wz': round(float(bag.wz[worst].mean()), 1),
                   'v_all': round(float(bag.v[idx].mean()), 2), 'wz_all': round(float(bag.wz[idx].mean()), 1)}
            marks = [(float(np.interp(j['t'] + t0, bag.st, loc[0])), float(np.interp(j['t'] + t0, bag.st, loc[1]))) for j in jumps[:30]]
            draw(m, run / 'overlay.png', [((ekf_only[0], ekf_only[1]), (235, 104, 52), 2), ((loc[0], loc[1]), (42, 120, 214), 2)], marks)
            res = {'map': args.map, 'bag': bag_dir.name, 'name': run.name, 'cond': {k: cond[k] for k in ('fov', 'range', 'slam') if k in cond},
                   **s, 'jumps': len(jumps), 'jump_list': jumps[:20], 'max_jump_m': round(float(dmo.max()), 2),
                   'max_jump_deg': round(float(dyaw.max()), 1), 'corr_final_m': round(float(corr[-1]), 2),
                   'traj': traj(loc),
                   'corr_max_m': round(float(corr.max()), 2), 'worst_ctx': ctx,
                   'series': {'t': [round(float(v), 1) for v in (bag.st[idx] - t0)[::5]],
                              'res': [round(float(v) * 100, 1) for v in r_loc[::5]]}}
            results.append(res)
            print(f"{bag_dir.name} {run.name:16s} 一致率 {s['inlier_med']:5.1f}% p10 {s['inlier_p10']:5.1f}% 低一致 {s['low_pct']:5.1f}% "
                  f"laps {s['laps']} lost {s['lost_t']} jumps {len(jumps)} maxjump {res['max_jump_m']}m/{res['max_jump_deg']}deg corr_end {res['corr_final_m']}")
    info = yaml.safe_load((SWEEP / args.map / 'map.yaml').read_text())
    out = {'map': args.map, 'map_info': {'resolution': info['resolution'], 'origin': info['origin'][:2],
                                         'width': m.W, 'height': m.H}, 'runs': results}
    (base / 'results.json').write_text(json.dumps(out, ensure_ascii=False))


if __name__ == '__main__':
    main()
