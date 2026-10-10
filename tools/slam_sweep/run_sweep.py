#!/usr/bin/env python3
"""bag再生でSLAM条件を並列に流す(ホストで実行)。

    python3 tools/slam_sweep/run_sweep.py --bag /bags/<bag> --groups A B
    python3 tools/slam_sweep/run_sweep.py --bag /bags/<bag> --groups C D --best-range 3
    python3 tools/slam_sweep/run_sweep.py --bag /bags/<bag> --extra 'name=F_final,range=3,loop=true,slam.distance_variance_penalty=2.0'

事前に docker compose --profile replay up -d replay しておく。
結果は replay_params/sweep/<name>/ (map.pgm/yaml, tf.csv, slam.log)。
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from pathlib import Path
import subprocess
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
BASE_SLAM = ROOT / 'src/minicar_bringup/config/slam_toolbox_mapping.yaml'
BASE_SCAN = ROOT / 'src/minicar_scan/config/scan_filter_params.yaml'
SWEEP = ROOT / 'replay_params/sweep'


def _value(text):
    return yaml.safe_load(text)


def parse_extra(spec):
    cond = {'slam': {}}
    for item in spec.split(','):
        key, val = item.split('=', 1)
        if key.startswith('slam.'):
            cond['slam'][key[5:]] = _value(val)
        else:
            cond[key] = _value(val)
    return cond


def build(cond, best_range):
    rng = cond.get('range', 5)
    if rng == 'best':
        rng = best_range
    rmax = 30.0 if str(rng) == 'inf' else float(rng)
    scan = yaml.safe_load(BASE_SCAN.read_text())
    sp = scan['scan_filter_node']['ros__parameters']
    sp['fov_deg'] = float(cond.get('fov', 240))
    sp['range_max'] = rmax
    slam = yaml.safe_load(BASE_SLAM.read_text())
    p = slam['slam_toolbox']['ros__parameters']
    p['use_sim_time'] = True
    if cond.get('scale', True):
        # 既定yamlの「### 2m」系は2m想定。コースが約10mなのでそれ以上は頭打ちにする。
        reff = min(rmax, 10.0)
        p['max_laser_range'] = rmax
        p['scan_buffer_maximum_scan_distance'] = round(1.25 * reff, 3)
        p['link_scan_maximum_distance'] = round(0.3 * reff, 3)
        p['loop_search_maximum_distance'] = round(0.75 * reff, 3)
    p['do_loop_closing'] = bool(cond.get('loop', False))
    p.update(cond.get('slam', {}))
    return scan, slam, rmax


def run(i, cond, args):
    out = SWEEP / cond['name']
    out.mkdir(parents=True, exist_ok=True)
    scan, slam, rmax = build(cond, args.best_range)
    (out / 'scan.yaml').write_text(yaml.safe_dump(scan, sort_keys=False))
    (out / 'slam.yaml').write_text(yaml.safe_dump(slam, sort_keys=False))
    (out / 'condition.yaml').write_text(yaml.safe_dump(
        dict(cond, range_resolved=rmax, bag=args.bag), sort_keys=False, allow_unicode=True))
    t0 = time.time()
    res = subprocess.run(
        ['docker', 'compose', 'exec', '-T', '-e', f'ROS_DOMAIN_ID={60 + i}', 'replay',
         '/tools/slam_sweep/run_one.sh', f'/replay_params/sweep/{cond["name"]}', args.bag],
        cwd=ROOT, capture_output=True, text=True)
    print(f'{cond["name"]:22s} {res.stdout.strip() or res.stderr.strip()[-200:]} '
          f'({time.time() - t0:.0f}s)', flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True, help='コンテナ内のbagパス')
    ap.add_argument('--groups', nargs='*', default=[])
    ap.add_argument('--only', nargs='*', default=[], help='条件名で絞り込む')
    ap.add_argument('--extra', action='append', default=[], help='追加条件 key=val,...')
    ap.add_argument('--best-range', default=3)
    ap.add_argument('--jobs', type=int, default=3)
    args = ap.parse_args()
    spec = yaml.safe_load((Path(__file__).parent / 'conditions.yaml').read_text())
    conds = []
    for g in args.groups:
        for c in spec['groups'][g]:
            merged = copy.deepcopy(spec.get('group_defaults', {}).get(g, {}))
            slam = {**merged.get('slam', {}), **c.get('slam', {})}
            merged.update(c)
            merged['slam'] = slam
            conds.append(merged)
    conds += [parse_extra(e) for e in args.extra]
    if args.only:
        conds = [c for c in conds if c['name'] in args.only]
    with ThreadPoolExecutor(args.jobs) as ex:
        for i, c in enumerate(conds):
            ex.submit(run, i % 30, c, args)


if __name__ == '__main__':
    main()
