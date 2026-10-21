"""速度 PI の実走データから「その速度を保つのに実際に必要だった出力」を引く。

throttle_map (フィードフォワード) をコース路面に合わせて引き直すための材料。

    python -X utf8 -I map_fit.py <name> <開始s> <終了s> [<開始s> <終了s> ...]

<name> は decode.py の出力 d_<name>.npz (カレントディレクトリ)。区間は
壁への接触・手での復帰を除いた通常走行の区間を指定する (dips.py で探す)。
1 秒間の実速の変化が 0.12 m/s 未満の「ほぼ定常」な点だけを使う。
"""
import sys

import numpy as np

STOP, SPAN = 379, 49          # pwm_params.json の stop と stop - front
MAP_V = [0.2, 0.8, 1.25, 1.54]
MAP_U = [0.30, 0.37, 0.47, 0.55]           # motor_driver_params.yaml の前進側 (10/20)

name = sys.argv[1]
ranges = [(float(a), float(b)) for a, b in zip(sys.argv[2::2], sys.argv[3::2])]
d = np.load(f'd_{name}.npz')
cr, od, sc, st = d['cmd_raw'], d['odom'], d['sc'], d['steer']
t0 = max(cr[0, 0], od[0, 0])

rows = []
for a, b in ranges:
    T = np.arange(t0 + a, t0 + b, 0.05)
    f = lambda arr, c: np.interp(T, arr[:, 0], arr[:, c])
    vm, ui, uo = f(sc, 2), f(sc, 5), f(sc, 6)
    de = np.abs(np.degrees(f(st, 1)))
    k = 20
    ok = np.array([i >= k and np.ptp(vm[i - k:i + 1]) < 0.12 for i in range(len(T))])
    rows.append((vm[ok], uo[ok], de[ok], ui))
    print(f'{a:.0f}-{b:.0f}s: u_i 平均 {ui.mean():+.3f} (+ ならマップが弱い、- なら強い)')

vm = np.concatenate([r[0] for r in rows]); uo = np.concatenate([r[1] for r in rows])
de = np.concatenate([r[2] for r in rows])
print('実速帯           舵角     必要だった出力 (カウント)   マップ    点数')
for lo in np.arange(0.3, 1.3, 0.15):
    hi = lo + 0.15
    for dl, dh, lab in ((0, 8, '直線寄り'), (8, 12, '中'), (12, 18, '大')):
        m = (vm >= lo) & (vm < hi) & (de >= dl) & (de < dh)
        if m.sum() >= 10:
            u = uo[m].mean()
            print(f'{lo:.2f}-{hi:.2f} m/s  {lab:6s}  {u:.3f} ({STOP - SPAN * u:5.1f})          '
                  f'{np.interp((lo + hi) / 2, MAP_V, MAP_U):.3f}    {m.sum()}')
