#!/usr/bin/env python3
"""実走bagを再生して AMCL (と比較用の slam_toolbox localization) を条件ごとに流す(ホストで実行)。

    python3 tools/slam_sweep/amcl_sweep.py --bag 202610101840
    python3 tools/slam_sweep/amcl_sweep.py --bag 202610101840 --only A00_base --record

条件は amcl_conditions.yaml。結果は replay_params/sweep/amcl/<bag>/<cond>/ (tf.csv, amcl_pose.csv, loc.log)。
--record を付けると、rviz2 で見るための結果bagを bags/<bag>_<cond>/ に記録する(1条件ずつ指定)。
初期姿勢は、bag開始直後の走行時の自己位置(bagの map->odom と odom->base_link の合成)。
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'src/minicar_bringup/config'
SWEEP = ROOT / 'replay_params/sweep/amcl'
sys.path.insert(0, str(ROOT / 'tools/bag_analysis'))
import pose_error as pe  # noqa: E402


def start_pose(bag):
    f = np.load(ROOT / 'bags' / bag / 'analysis/run.npz')
    mo, ob = pe.uniq(f['tf_map_odom']), pe.uniq(f['tf_odom_base'])
    t = np.array([ob[0, 0] + 0.5])
    return [round(float(a[0]), 4) for a in pe.compose(pe.interp_pose(mo, t), pe.interp_pose(ob, t))]


def run(i, cond, args, start):
    out = SWEEP / args.bag / cond['name']
    out.mkdir(parents=True, exist_ok=True)
    scan = yaml.safe_load((CONFIG / 'scan_filter_localization.yaml').read_text())
    scan['scan_filter_node']['ros__parameters'].update(cond.get('scan', {}))
    (out / 'scan.yaml').write_text(yaml.safe_dump(scan, sort_keys=False))
    engine = cond.get('engine', 'amcl')
    if engine == 'amcl':
        p = yaml.safe_load((CONFIG / 'amcl_localization.yaml').read_text())
        p['amcl']['ros__parameters'].update(cond.get('amcl', {}))
        (out / 'amcl.yaml').write_text(yaml.safe_dump(p, sort_keys=False))
        mp = '/maps/track_v1.yaml'
    else:
        p = yaml.safe_load((CONFIG / 'slam_toolbox_localization.yaml').read_text())
        sp = p['slam_toolbox']['ros__parameters']
        sp['use_sim_time'] = True
        sp['map_start_pose'] = start
        sp.update(cond.get('slam', {}))
        (out / 'slam.yaml').write_text(yaml.safe_dump(p, sort_keys=False))
        mp = '/maps/track_v1'
    (out / 'condition.yaml').write_text(yaml.safe_dump(dict(cond, bag=args.bag, start=start),
                                                       sort_keys=False, allow_unicode=True))
    rec = [f'/bags/{args.bag}_{cond["name"]}'] if args.record else []
    t0 = time.time()
    res = subprocess.run(
        ['docker', 'compose', 'exec', '-T', '-e', f'ROS_DOMAIN_ID={70 + i}', 'replay',
         '/tools/slam_sweep/run_amcl.sh', f'/replay_params/sweep/amcl/{args.bag}/{cond["name"]}',
         f'/bags/{args.bag}', engine, mp, *[str(v) for v in start], *rec],
        cwd=ROOT, capture_output=True, text=True)
    print(f'{cond["name"]:22s} {res.stdout.strip()[-80:] or res.stderr.strip()[-200:]} ({time.time() - t0:.0f}s)', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='202610101840')
    ap.add_argument('--only', nargs='*', default=[])
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--record', action='store_true')
    args = ap.parse_args()
    spec = yaml.safe_load((Path(__file__).parent / 'amcl_conditions.yaml').read_text())
    conds = [c for c in spec['conditions'] if not args.only or c['name'] in args.only]
    start = start_pose(args.bag)
    print(f'初期姿勢 {start}  {len(conds)} 条件')
    with ThreadPoolExecutor(args.jobs) as ex:
        for i, c in enumerate(conds):
            ex.submit(run, i % 25, c, args, start)


if __name__ == '__main__':
    main()
