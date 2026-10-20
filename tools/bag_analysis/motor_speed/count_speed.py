"""ほぼ定常な区間で、出力カウントごとの実速度の分布を見る (throttle_map を引き直す材料)。

    python -X utf8 -I count_speed.py <name>:<開始s>-<終了s>[,<開始s>-<終了s>...] [<name>:...]
    例: count_speed.py 202610191124_speedpi:6-100 202610191200_speedpi2:24-107,128-185

d_<name>.npz (decode.py の出力) をカレントディレクトリから読む。直前 1s の
出力カウントの振れが 2 以下で、実速の変化が 0.1 m/s 未満の点だけを使う。
"""
import sys

import numpy as np
SEG = {}
for arg in sys.argv[1:]:
    name, segs = arg.split(':')
    SEG[name] = [tuple(float(x) for x in s.split('-')) for s in segs.split(',')]
DT = 0.05
rows = []
for name, segs in SEG.items():
    d = np.load(f'd_{name}.npz'); cr, od, sc, st, pwm = d['cmd_raw'], d['odom'], d['sc'], d['steer'], d['pwm']
    t0 = max(cr[0, 0], od[0, 0])
    for a, b in segs:
        T = np.arange(t0 + a, t0 + b, DT)
        vo = np.interp(T, od[:, 0], od[:, 5]); cnt = np.interp(T, pwm[:, 0], pwm[:, 2])
        de = np.abs(np.degrees(np.interp(T, st[:, 0], st[:, 1])))
        k = int(1.0 / DT)   # 直前 1s のカウント平均 (速度の遅れを考慮) と、その間の速度変化
        for i in range(k, len(T)):
            c_avg = cnt[i - k:i + 1].mean(); dv = vo[i] - vo[i - k]
            if np.ptp(cnt[i - k:i + 1]) <= 2 and abs(dv) < 0.1:
                rows.append((name, c_avg, vo[i], de[i]))
c = np.array([r[1] for r in rows]); v = np.array([r[2] for r in rows]); de = np.array([r[3] for r in rows])
run = np.array([r[0] for r in rows])
print('1s 平均カウント   直線寄り(<8°) 実速 中央値 [p25-p75] n      舵大(>=12°)')
for lo in np.arange(357.5, 368.5, 1.0):
    m = (c >= lo) & (c < lo + 1)
    out = []
    for mm in (m & (de < 8), m & (de >= 12)):
        out.append('%.2f [%.2f-%.2f] %4d' % (np.median(v[mm]), *np.percentile(v[mm], [25, 75]), mm.sum()) if mm.sum() >= 15 else '        -         ')
    print(f'  {lo + 0.5:5.0f}            {out[0]}      {out[1]}')
for name in SEG:
    m = (run == name) & (de < 8)
    A = np.vstack([c[m], np.ones(m.sum())]).T
    sl, ic = np.linalg.lstsq(A, v[m], rcond=None)[0]
    print(f'{name}: 直線寄り 回帰 v = {sl:+.3f} * count + {ic:.1f}  -> 1 カウントあたり {-sl:.3f} m/s、v=0 のカウント {-ic / sl:.1f}')
