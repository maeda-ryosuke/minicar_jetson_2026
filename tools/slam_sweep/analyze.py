#!/usr/bin/env python3
"""sweep結果を集計し、崩壊点・原因材料・地図指標を results.json と画像に出す(ホストで実行)。

    python3 tools/slam_sweep/analyze.py

EKF(odom->base_link)は1周の終点誤差が約0.3mと良好なので、SLAM姿勢(map->base_link)
との相対運動の食い違いで「SLAMがどこで外れたか」を判定する。
"""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
import yaml

ROOT = Path(__file__).resolve().parents[2]
SWEEP = ROOT / 'replay_params/sweep'
JUMP_YAW = math.radians(5.0)   # 1秒あたりのSLAMとEKFの相対回転差
JUMP_POS = 0.3                 # 1秒あたりの相対並進差[m]
DRIFT_YAW = math.radians(15.0)  # 開始からの累積yaw差


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rel_motion(x, y, yaw):
    """各1秒区間の車体座標系での相対運動(dx, dy, dyaw)。"""
    dx, dy = np.diff(x), np.diff(y)
    c, s = np.cos(yaw[:-1]), np.sin(yaw[:-1])
    return c * dx + s * dy, -s * dx + c * dy, wrap(np.diff(yaw))


def scan_context(feat, t, fov, rmax):
    """時刻tの前後1秒のscanについて、有効ビーム数と廊下度合いを返す。"""
    st = feat['scan_t']
    idx = np.where(np.abs(st - t) <= 1.0)[0]
    if len(idx) == 0:
        return {}
    ang = np.degrees(feat['angle_min'] + np.arange(feat['ranges'].shape[1]) * feat['angle_inc'])
    use = np.abs(ang) <= fov / 2
    r = feat['ranges'][idx][:, use]
    a = ang[use]
    ok = np.isfinite(r) & (r >= 0.1) & (r <= rmax)
    valid = ok.sum(axis=1)
    front = (ok & (np.abs(a) <= 45)).sum(axis=1)
    side = (ok & (np.abs(np.abs(a) - 90) <= 30)).sum(axis=1)
    return {'valid_beams': int(valid.mean()), 'front_beams': int(front.mean()),
            'side_ratio': round(float(side.sum() / max(valid.sum(), 1)), 2)}


def ekf_context(feat, t):
    e = feat['ekf']
    m = np.abs(e[:, 0] - t) <= 1.0
    gaps = np.diff(feat['odom_recv_t'])
    gt = feat['odom_recv_t'][1:]
    g = (np.abs(gt - t) <= 2.0) & (gaps > 0.1)
    return {'v': round(float(np.abs(e[m, 4]).mean()), 2),
            'wz_deg': round(float(np.degrees(np.abs(e[m, 5]).max())), 1),
            'odom_gaps_2s': int(g.sum()), 'odom_gap_max': round(float(gaps[g].max()) if g.any() else 0.0, 2)}


def map_metrics(mp, info):
    img = np.array(Image.open(mp))
    occ = np.argwhere(img < 50)  # 占有
    res = info['resolution']
    pts = occ[:, ::-1].astype(float) * res
    best = None
    for deg in range(0, 90):
        th = math.radians(deg)
        rx = pts[:, 0] * math.cos(th) + pts[:, 1] * math.sin(th)
        ry = -pts[:, 0] * math.sin(th) + pts[:, 1] * math.cos(th)
        # 外れ値に引っ張られないよう0.5%/99.5%点で外形を取る
        w = np.percentile(rx, 99.5) - np.percentile(rx, 0.5)
        h = np.percentile(ry, 99.5) - np.percentile(ry, 0.5)
        if best is None or w * h < best[0] * best[1]:
            best = (w, h)
    w, h = sorted(best, reverse=True)
    return {'occupied_cells': int(len(occ)), 'extent_m': [round(w, 2), round(h, 2)]}


def draw(run, info, tf, collapse_i, out):
    img = Image.open(run / 'map.pgm').convert('RGB')
    H = img.size[1]
    res, ox, oy = info['resolution'], info['origin'][0], info['origin'][1]
    px = lambda x, y: ((x - ox) / res, H - (y - oy) / res)
    d = ImageDraw.Draw(img)
    pts = [px(x, y) for x, y in zip(tf['mb_x'], tf['mb_y'])]
    d.line(pts, fill=(40, 110, 220), width=2)
    sx, sy = pts[0]
    d.ellipse([sx - 5, sy - 5, sx + 5, sy + 5], outline=(20, 150, 60), width=3)
    if collapse_i is not None:
        cx, cy = pts[collapse_i]
        d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], outline=(220, 30, 30), width=3)
    img.save(out)


def analyze(run, feat):
    cond = yaml.safe_load((run / 'condition.yaml').read_text())
    scan = yaml.safe_load((run / 'scan.yaml').read_text())['scan_filter_node']['ros__parameters']
    raw = np.genfromtxt(run / 'tf.csv', delimiter=',', names=True)
    tf = {k: raw[k] for k in raw.dtype.names}
    t0 = feat['ekf'][0, 0]
    t = tf['t'] - t0
    sl = rel_motion(tf['mb_x'], tf['mb_y'], tf['mb_yaw'])
    ek = rel_motion(tf['ob_x'], tf['ob_y'], tf['ob_yaw'])
    dpos = np.hypot(sl[0] - ek[0], sl[1] - ek[1])
    dyaw = np.abs(wrap(sl[2] - ek[2]))
    drift = wrap(tf['mo_yaw'] - tf['mo_yaw'][0])
    jump = np.where((dyaw > JUMP_YAW) | (dpos > JUMP_POS))[0]
    slow = np.where(np.abs(drift) > DRIFT_YAW)[0]
    events = []
    if len(jump):
        events.append(('jump', int(jump[0]) + 1))
    if len(slow):
        events.append(('drift', int(slow[0])))
    collapse = min(events, key=lambda e: e[1]) if events else None
    info = yaml.safe_load((run / 'map.yaml').read_text())
    res = {
        'name': run.name, 'cond': cond,
        'final_map_odom': [round(float(tf['mo_x'][-1]), 2), round(float(tf['mo_y'][-1]), 2),
                           round(math.degrees(float(drift[-1])), 1)],
        'max_abs_yaw_drift_deg': round(math.degrees(float(np.abs(drift).max())), 1),
        'jumps': [{'t': round(float(t[i + 1]), 0), 'dyaw_deg': round(math.degrees(float(dyaw[i])), 1),
                   'dpos': round(float(dpos[i]), 2)} for i in jump[:8]],
        'series': {'t': [round(float(v), 1) for v in t],
                   'yaw_drift_deg': [round(math.degrees(float(v)), 2) for v in drift]},
        **map_metrics(run / 'map.pgm', info),
    }
    if collapse:
        i = collapse[1]
        res['collapse'] = {'kind': collapse[0], 't': round(float(t[i]), 0),
                           'pos': [round(float(tf['mb_x'][i]), 2), round(float(tf['mb_y'][i]), 2)],
                           'ekf_pos': [round(float(tf['ob_x'][i]), 2), round(float(tf['ob_y'][i]), 2)],
                           **scan_context(feat, tf['t'][i], scan['fov_deg'], scan['range_max']),
                           **ekf_context(feat, tf['t'][i])}
    draw(run, info, tf, collapse[1] if collapse else None, run / 'map_overlay.png')
    return res


def main():
    feat = dict(np.load(SWEEP / 'features.npz'))
    results = []
    for run in sorted(p for p in SWEEP.iterdir() if (p / 'tf.csv').exists() and (p / 'map.pgm').exists()):
        r = analyze(run, feat)
        results.append(r)
        c = r.get('collapse', {})
        print(f"{r['name']:20s} final yaw {r['final_map_odom'][2]:+7.1f}deg max {r['max_abs_yaw_drift_deg']:6.1f} "
              f"occ {r['occupied_cells']:6d} extent {r['extent_m']}  collapse {c.get('kind','-')} t={c.get('t','-')} "
              f"valid={c.get('valid_beams','-')} front={c.get('front_beams','-')} side={c.get('side_ratio','-')} "
              f"wz={c.get('wz_deg','-')} gaps={c.get('odom_gaps_2s','-')}")
    (SWEEP / 'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
