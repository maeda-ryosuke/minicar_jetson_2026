"""実走の MPPI の指令と舵角を車両モデルに流して、throttle_map の候補を比べる。

    python -X utf8 -I replay_map.py <minicar_motor のパス> <name> <開始s>-<終了s> [...]

比べるマップは下の MAPS を編集する。車両モデルの表 (C_TAB / V_TAB / DRAG) は
count_speed.py の結果で更新する。今のマップで実機の数字 (誤差 RMS・振れ幅) に
近い値が出ることを先に確かめてから候補を比べること。

車両モデル (2026-10-19 のほぼ定常データ):
  直線寄りの定常速度: カウント -> m/s の表 (360〜364 は実測、それ以外は推定)
  舵角による低下: 舵いっぱい (17.2°) で 0.17 m/s (362 カウントで 0.66 -> 0.49)
  出力 -> 速度の遅れ 0.2s、1 次遅れ 0.6s、静止摩擦
"""
import sys
import numpy as np
sys.path.insert(0, sys.argv[1])
from minicar_motor.motor_driver_core import SpeedController

C_TAB = [345, 350, 356, 360, 361, 362, 363, 364, 366, 368, 372.6]
V_TAB = [1.65, 1.40, 1.10, 0.90, 0.69, 0.66, 0.61, 0.55, 0.42, 0.25, 0.0]
DRAG = 0.17 / 17.2          # [m/s / deg]
FFV = [0.0, 0.001, 0.20, 0.80, 1.25, 1.54]
MAPS = {
    '10/19 のマップ': [0.0, 0.2122, 0.2122, 0.4571, 0.6408, 0.7592],
    '10/20 のマップ': [0.0, 0.25, 0.30, 0.37, 0.47, 0.55],
}
DT = 0.02
NAME = sys.argv[2]
SEGS = [tuple(float(x) for x in a.split('-')) for a in sys.argv[3:]]
d = np.load(f'd_{NAME}.npz'); cr, od, st = d['cmd_raw'], d['odom'], d['steer']
t0 = max(cr[0, 0], od[0, 0])


def vss(count, deg):
    return max(float(np.interp(count, C_TAB, V_TAB)) - DRAG * abs(deg), 0.0)


def run(seg, ffu, kp=0.15, ki=0.4, delay=0.2, tau=0.6):
    T = np.arange(t0 + seg[0], t0 + seg[1], DT)
    VC = np.interp(T, cr[:, 0], cr[:, 1]); DE = np.degrees(np.interp(T, st[:, 0], st[:, 1]))
    s = SpeedController(kp, ki, -0.15, 0.3, 0.7, 0.25)
    v = float(np.interp(T[0], od[:, 0], od[:, 5])); vf = vm = v; to = 0.0
    buf = [379.0] * (int(round(delay / DT)) + 1); out = []
    for k in range(len(T)):
        if k * DT >= to:
            vm = round(v / 0.0262) * 0.0262; to += 0.027
        vf += (vm - vf) * DT / (0.05 + DT)
        u = s.update(VC[k], vf, float(np.interp(VC[k], FFV, ffu)), DT).u
        buf.append(round(379 - 49 * u)); c = buf.pop(0)
        target = vss(c, DE[k])
        if v < 0.01 and target < 0.05:
            v = 0.0
        else:
            v += (target - v) * DT / tau
        out.append((VC[k], v, c))
    a = np.array(out)
    vc, v, c = a.T
    k = int(2 / DT)
    swing = np.median([np.ptp(v[i:i + k]) for i in range(0, len(v) - k, 5)])
    cswing = np.median([np.ptp(c[i:i + k]) for i in range(0, len(c) - k, 5)])
    return np.sqrt(np.mean((v - vc) ** 2)), swing, (v < 0.35).mean() * 100, cswing


# まずモデルの妥当性: 実機と同じ「今のマップ」で、実機の数字に近いか
for seg in SEGS:
    print(f'--- 区間 {seg[0]}-{seg[1]}s')
    for name, ffu in MAPS.items():
        for kp, ki in ((0.15, 0.4), (0.15, 0.25)):
            e, sw, lo, cs = run(seg, ffu, kp, ki)
            print(f'  {name:8s} kp{kp} ki{ki}: 誤差 RMS {e:.3f}  2s 振れ幅 {sw:.2f}  出力振れ {cs:.0f} カウント  実速<0.35 {lo:.1f}%')
