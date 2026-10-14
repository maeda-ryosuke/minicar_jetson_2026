#!/usr/bin/env python3
"""再生した自己位置推定の各条件を pose_error.py の同じ指標で評価し、表にまとめる(ホストで実行)。

    python3 tools/bag_analysis/compare_runs.py bags/<bag> replay_params/sweep/amcl/<bag>

<sweep>/<cond>/tf.csv ごとに pose_error.py を流して <cond>/summary.json を作り、
走行時の自己位置(bag の tf)と並べて <sweep>/compare.json に出す。
oracle.py の結果(<bag>/analysis/oracle.json)があれば、各条件の自己位置と
「地図に最もよく合う姿勢」との差(中央値)も出す。
"""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

HERE = Path(__file__).resolve().parent


def oracle_gap(bag, summ):
    f = bag / 'analysis/oracle.json'
    if not f.exists():
        return None
    o = json.loads(f.read_text())
    run = json.loads((bag / 'analysis/summary.json').read_text())['traj']
    rt = np.array(run['t'])
    ox = np.interp(o['t'], rt, run['x']) + np.array(o['dx'])
    oy = np.interp(o['t'], rt, run['y']) + np.array(o['dy'])
    tr = summ['traj']
    ex, ey = np.interp(o['t'], tr['t'], tr['x']), np.interp(o['t'], tr['t'], tr['y'])
    return round(float(np.median(np.hypot(ex - ox, ey - oy))), 3)


def main():
    bag, sweep = Path(sys.argv[1]), Path(sys.argv[2])
    rows = []
    subprocess.run([sys.executable, HERE / 'pose_error.py', bag, '--label', 'slam_toolbox(走行時)'],
                   check=True, capture_output=True)
    rows.append(('RUN_slam_toolbox(走行時)', json.loads((bag / 'analysis/summary.json').read_text())))
    for d in sorted(p for p in sweep.iterdir() if (p / 'tf.csv').exists()):
        label = 'slam_toolbox' if d.name.startswith('S') else 'AMCL'
        subprocess.run([sys.executable, HERE / 'pose_error.py', bag, '--tf-csv', d / 'tf.csv', '--out', d,
                        '--label', f'{label} {d.name}'], check=True, capture_output=True)
        rows.append((d.name, json.loads((d / 'summary.json').read_text())))
    print(f'{"条件":26s} {"一致率中央":>6} {"p10":>5} {"低一致%":>6} {"跳び":>4} {">10cm":>5} {"最大跳び":>7} {"横ずれ":>6} {"上限姿勢との差":>8}')
    out = []
    for name, s in rows:
        gap = oracle_gap(bag, s)
        s['oracle_gap_m'] = gap
        out.append({'name': name, **{k: v for k, v in s.items() if k not in ('series', 'traj')}})
        print(f'{name:26s} {s["match_med"]:8.1f} {s["match_p10"]:5.1f} {s["low_pct"]:7.1f} {s["jumps"]:4d} '
              f'{s.get("jumps_10cm", 0):5d} {s["max_jump_m"]:6.2f}m {s["lat_mean"]:5.3f}m {gap if gap is not None else "-":>8}')
    (sweep / 'compare.json').write_text(json.dumps(out, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
