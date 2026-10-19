"""止まりかけ (実速 < 0.3 m/s、指令 >= 0.45) のイベントを、地図上の位置と直前の状態つきで並べる。

    python -X utf8 -I dips.py <name> [<t_end>]
"""
import pickle
import struct
import sys

import numpy as np

name = sys.argv[1]
t_end = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
d = np.load(f'd_{name}.npz')
cr, od, sc, st, pwm = d['cmd_raw'], d['odom'], d['sc'], d['steer'], d['pwm']
rec = pickle.load(open(f'r_{name}.pkl', 'rb'))
tid = [k for k, v in rec['topics'].items() if v[0] == '/amcl_pose'][0]
A = []
for t, ts, b in sorted(rec['msgs'].values(), key=lambda r: r[1]):
    if t != tid:
        continue
    i = 4 + 8
    n = struct.unpack_from('<I', b, i)[0]; i += 4 + n
    r = (i - 4) % 8; i += (8 - r) if r else 0
    x, y = struct.unpack_from('<2d', b, i)
    A.append((ts * 1e-9, x, y))
A = np.array(A)

DT = 0.05
t0 = max(cr[0, 0], od[0, 0])
T = np.arange(t0, min(cr[-1, 0], od[-1, 0], t0 + t_end), DT)
f = lambda a, c: np.interp(T, a[:, 0], a[:, c])
vc, vo, uo, ui, uff = f(cr, 1), f(od, 5), f(sc, 6), f(sc, 5), f(sc, 3)
de = np.degrees(f(st, 1)); cnt = f(pwm, 2)
X, Y = np.interp(T, A[:, 0], A[:, 1]), np.interp(T, A[:, 0], A[:, 2])

run = vc >= 0.45
low = run & (vo < 0.3)
e = np.diff(np.r_[0, low.astype(int), 0]); s = np.where(e == 1)[0]; g = np.where(e == -1)[0]
print(f'{name}: 走行 {run.sum() * DT:.0f}s  実速/指令 {vo[run].mean() / vc[run].mean():.2f}  '
      f'実速<0.3 {low.sum() * DT:.1f}s ({low.sum() / run.sum() * 100:.1f}%)')
print('   t    長さ  位置(x,y)      最低速  舵角   出力最小(直前1s)  カウント  指令  直前1sの指令変化  直前1sの最高速')
for a, b in zip(s, g):
    if (b - a) * DT < 0.15:
        continue
    k0 = max(0, a - int(1 / DT))
    print('%6.1f %4.1fs (%5.2f,%5.2f)  %5.2f %+6.1f  %.2f             %4.0f     %.2f  %+.2f            %.2f' % (
        T[a] - t0, (b - a) * DT, X[a], Y[a], vo[a:b].min(), de[a:b].mean(), uo[k0:a + 1].min(),
        cnt[a:b].mean(), vc[a:b].mean(), vc[a] - vc[k0], vo[k0:a + 1].max()))
