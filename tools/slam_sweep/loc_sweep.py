#!/usr/bin/env python3
"""保存済み地図でslam_toolbox localizationを条件ごとに流す(ホストで実行)。

    python3 tools/slam_sweep/loc_sweep.py --map MAP_G4 --bags slow fast
    python3 tools/slam_sweep/loc_sweep.py --map MAP_H4 --bags slow --only L01_trust01

結果は replay_params/sweep/loc/<map>/<bag>/<cond>/ (tf.csv, slam.log)。
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE_SLAM = ROOT / 'src/minicar_bringup/config/slam_toolbox_localization.yaml'
BASE_SCAN = ROOT / 'src/minicar_scan/config/scan_filter_params.yaml'
SWEEP = ROOT / 'replay_params/sweep'


def run(i, cond, bag_name, bag, args):
    rel = f'loc/{args.map}/{bag_name}/{cond["name"]}'
    out = SWEEP / rel
    out.mkdir(parents=True, exist_ok=True)
    scan = yaml.safe_load(BASE_SCAN.read_text())
    sp = scan['scan_filter_node']['ros__parameters']
    sp['fov_deg'] = float(cond.get('fov', 240))
    sp['range_max'] = float(cond.get('range', 2.0))
    slam = yaml.safe_load(BASE_SLAM.read_text())
    p = slam['slam_toolbox']['ros__parameters']
    if cond.get('base_from_map'):
        # 地図作成時のパラメータ一式を引き継ぎ、localization固有の項目だけ差し替える
        src = cond.get('base_map', args.map)
        mp = yaml.safe_load((SWEEP / src / 'slam.yaml').read_text())['slam_toolbox']['ros__parameters']
        keep = {k: p[k] for k in ('mode', 'map_file_name', 'map_start_at_dock', 'enable_interactive_mode')}
        p.clear()
        p.update(mp)
        p.update(keep)
    p['use_sim_time'] = True
    p['max_laser_range'] = sp['range_max']
    p['map_start_pose'] = [float(v) for v in bag['start']]
    p.update(cond.get('slam', {}))
    (out / 'scan.yaml').write_text(yaml.safe_dump(scan, sort_keys=False))
    (out / 'slam.yaml').write_text(yaml.safe_dump(slam, sort_keys=False))
    (out / 'condition.yaml').write_text(yaml.safe_dump(
        dict(cond, map=args.map, bag=bag['path'], start=bag['start']), sort_keys=False, allow_unicode=True))
    t0 = time.time()
    res = subprocess.run(
        ['docker', 'compose', 'exec', '-T', '-e', f'ROS_DOMAIN_ID={60 + i}', 'replay',
         '/tools/slam_sweep/run_loc.sh', f'/replay_params/sweep/{rel}', bag['path'],
         f'/replay_params/sweep/{args.map}/map'],
        cwd=ROOT, capture_output=True, text=True)
    print(f'{bag_name:5s} {cond["name"]:18s} {res.stdout.strip() or res.stderr.strip()[-200:]} '
          f'({time.time() - t0:.0f}s)', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='MAP_G4')
    ap.add_argument('--bags', nargs='+', default=['slow', 'fast'])
    ap.add_argument('--only', nargs='*', default=[])
    ap.add_argument('--jobs', type=int, default=4)
    args = ap.parse_args()
    spec = yaml.safe_load((Path(__file__).parent / 'loc_conditions.yaml').read_text())
    conds = [c for c in spec['conditions'] if not args.only or c['name'] in args.only]
    jobs = [(c, b) for b in args.bags for c in conds]
    with ThreadPoolExecutor(args.jobs) as ex:
        for i, (c, b) in enumerate(jobs):
            ex.submit(run, i % 30, c, b, spec['bags'][b], args)


if __name__ == '__main__':
    main()
