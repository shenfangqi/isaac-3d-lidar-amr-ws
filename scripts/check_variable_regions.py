#!/usr/bin/env python3
"""
Offline check of a variable-region file before it is adopted.  (read-only)

A variable region is ignored by the 3D scoring for every candidate, so a
badly drawn box can help a wrong pose as much as it helps the truth.  This
runs the existing offline evaluations with the region file and fails on any
wrong acceptance.

``quick`` (after each box edit, about 25 min for one box):
  - the labelled real samples with all regions: no negative may be
    accepted, no positive may accept a wrong leader;
  - for each checked region, synthetic views within --margin of the box
    with the box closed (a thin plane across the box, from the floor up),
    all regions: 0 wrong.
``full`` (once per set of edits; required before adoption): quick for every
  region, the same local views with the box open (scene as mapped), window
  repeats and the degraded map-wide analysis.

Both modes sample 0.2 m x 6 headings: a wrong acceptance seen so far
occupied one position and two of six headings, so a sparser grid would
miss it.  A local result is reused while its inputs are unchanged: the
box, every other box overlapping the analysed area, the mesh, the
analysis settings and the localization code.  Writes every evaluation and
``summary.json`` under --out; exit code 1 on a failure.  Nothing is
published and no ROS node is started.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
DIAGNOSIS = ROOT / 'calibration_data/2026-10-09_localization_diagnosis'
CHECKS = ROOT / 'calibration_data/variable_region_checks'
STEP, HEADINGS = '0.2', '6'
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.localization_surface_model import file_sha256  # noqa: E402
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions)


def closed_surface(box, thickness=.02):
    """A thin plane across the box, normal to its thinnest horizontal side."""
    low, high = list(box[:3]), list(box[3:])
    axis = 0 if high[0] - low[0] < high[1] - low[1] else 1
    middle = (low[axis] + high[axis]) / 2
    low[axis], high[axis] = middle - thickness / 2, middle + thickness / 2
    low[2] = max(low[2], 0.)
    return ','.join(f'{v:.3f}' for v in low + high)


def code_hash():
    """The localization code and analysis script the local results depend on."""
    digest = hashlib.sha256()
    files = sorted((ROOT / 'src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup')
                   .glob('localization_*.py'))
    for path in files + [ROOT / 'scripts/analyze_localizability_3d.py']:
        digest.update(path.name.encode() + path.read_bytes())
    return digest.hexdigest()


def run(name, command, out):
    """Run one evaluation script, keep its log, return its output JSON."""
    started = time.monotonic()
    print(f'== {name}', flush=True)
    with open(out / f'{name}.log', 'w', encoding='utf-8') as log:
        code = subprocess.call([sys.executable, *command], stdout=log,
                               stderr=subprocess.STDOUT, cwd=ROOT)
    if code:
        raise SystemExit(f'{name} failed (exit {code}), see {out / (name + ".log")}')
    print(f'   {time.monotonic() - started:.0f} s', flush=True)
    return json.loads((out / f'{name}.json').read_text())


def check(results, name, ok, detail):
    results.append(dict(check=name, passed=bool(ok), detail=detail))
    print(f'   {"PASS" if ok else "FAIL"}  {name}: {detail}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('quick', 'full'))
    parser.add_argument('regions', type=Path, help='variable-region YAML to check')
    parser.add_argument('--only', action='append', default=[],
                        help='quick: region name to check (repeatable; default: every region)')
    parser.add_argument('--map', type=Path,
                        default=DIAGNOSIS / 'real_map/carbot_map_20260928_215841.yaml')
    parser.add_argument('--mesh', type=Path,
                        default=DIAGNOSIS / 'real_map/carbot_map_20260928_215841.ply')
    parser.add_argument('--samples', type=Path,
                        default=DIAGNOSIS / 'surface_samples_2026-10-10.json')
    parser.add_argument('--margin', type=float, default=1.5,
                        help='synthetic views within this distance of the box, m')
    parser.add_argument('--max-pairs', type=int, default=20)
    parser.add_argument('--workers', type=int, default=16)
    parser.add_argument('--surface-additions', type=Path,
                        help='surface-additions NPZ (default: the per-map file in '
                             'config/localization if present)')
    parser.add_argument('--no-reuse', action='store_true',
                        help='rerun local checks even when their inputs are unchanged')
    parser.add_argument('--out', type=Path,
                        help=f'output directory (default: {CHECKS.relative_to(ROOT)}/<time>)')
    args = parser.parse_args()
    regions_path = args.regions.resolve()
    mesh_sha = file_sha256(args.mesh)
    regions = load_variable_regions(regions_path, mesh_sha)
    unknown = set(args.only) - set(regions.names)
    if unknown:
        raise SystemExit(f'no region named {sorted(unknown)} in {regions_path}')
    chosen = [k for k, n in enumerate(regions.names)
              if args.mode == 'full' or not args.only or n in args.only]
    out = args.out or CHECKS / time.strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=True)
    cache = CHECKS / 'cache'
    cache.mkdir(parents=True, exist_ok=True)
    shutil.copy(regions_path, out / 'regions.yaml')
    common = ['--map', str(args.map), '--mesh', str(args.mesh)]
    additions = args.surface_additions or (
        ROOT / 'src/isaac_3d_lidar_bringup/config/localization'
        / f'{args.map.stem}_surface_additions.npz')
    if additions.exists():
        common += ['--surface-additions', str(additions.resolve())]
        print(f'surface additions: {additions}', flush=True)
    analyze = ['scripts/analyze_localizability_3d.py', *common, '--workers', str(args.workers)]
    code = code_hash()
    results = []

    report = run('samples', ['scripts/evaluate_surface_decisions.py', '--samples',
                             str(args.samples), *common, '--spacings', '0.02',
                             '--search3d', 'union', '--variable-regions', str(regions_path),
                             '--output', str(out / 'samples.json')], out)
    models = [s['models']['0.02'] for s in report['samples']]
    wrong = sum(m['positive']['accepted'] and not m['positive']['leader_is_truth']
                for m in models)
    negative = sum(bool(m['negative']['accepted']) for m in models)
    accepted = sum(bool(m['positive']['accepted']) for m in models)
    check(results, 'real samples', wrong == 0 and negative == 0,
          f'{accepted}/{len(models)} accepted, {wrong} wrong leader, '
          f'{negative} negative accepted')

    states = ('closed', 'open') if args.mode == 'full' else ('closed',)
    for k in chosen:
        name, box = regions.names[k], regions.boxes[k]
        bounds = (box[0] - args.margin, box[1] - args.margin,
                  box[3] + args.margin, box[4] + args.margin)
        nearby = sorted(b.round(3).tolist() for j, b in enumerate(regions.boxes)
                        if j != k and b[0] <= bounds[2] and b[3] >= bounds[0]
                        and b[1] <= bounds[3] and b[4] >= bounds[1])
        print(f'-- {name}: {box.round(2).tolist()}', flush=True)
        for state in states:
            key = hashlib.sha256(json.dumps(dict(
                state=state, box=box.round(3).tolist(), nearby=nearby, mesh=mesh_sha,
                map=file_sha256(args.map), margin=args.margin, step=STEP,
                headings=HEADINGS, code=code,
                additions=file_sha256(additions) if additions.exists() else '')).encode()
            ).hexdigest()[:20]
            tag = f'region{k + 1}_{state}'
            cached = cache / f'{key}.json'
            if cached.exists() and not args.no_reuse:
                shutil.copy(cached, out / f'{tag}.json')
                summary = json.loads(cached.read_text())['summary']
                print(f'== {tag}: unchanged, reused {cached.name}', flush=True)
            else:
                extra = [f'--add-surface={closed_surface(box)}'] if state == 'closed' else []
                summary = run(tag, [
                    *analyze, '--step', STEP, '--headings', HEADINGS, *extra,
                    '--region-bounds', *[str(round(v, 2)) for v in bounds],
                    '--variable-regions', str(regions_path),
                    '--output', str(out / f'{tag}.json')], out)['summary']
                shutil.copy(out / f'{tag}.json', cached)
            check(results, f'{name}, {state}', summary['wrong_leaders'] == 0,
                  f'{summary["verdicts"]}')

    if args.mode == 'full':
        repeats = run('repeats', ['scripts/evaluate_window_repeats.py', '--samples',
                                  str(args.samples), *common, '--workers', '8',
                                  '--max-pairs', str(args.max_pairs), '--variable-regions',
                                  str(regions_path), '--output', str(out / 'repeats.json')],
                      out)
        rows = [r['summary'] for r in repeats['results'].values()]
        pairs = sum(r['pairs'] for r in rows)
        accepted = sum(r['accepted'] for r in rows)
        wrong = sum(r['wrong_leader_accepted'] for r in rows)
        negative = sum(r['negative_wrong_accepts'] for r in rows)
        check(results, 'window repeats', wrong == 0 and negative == 0,
              f'{accepted}/{pairs} accepted, {wrong} wrong leader, '
              f'{negative} negative accepted')
        summary = run('site_degraded', [
            *analyze, '--step', '0.3', '--headings', HEADINGS,
            '--variable-regions', str(regions_path), '--no-ceiling', '--clutter', '0.2',
            '--output', str(out / 'site_degraded.json')], out)['summary']
        check(results, 'map-wide degraded', summary['wrong_leaders'] == 0,
              f'{summary["verdicts"]}, {summary["places_all_accepted"]}/'
              f'{summary["places"]} places all headings')

    passed = all(r['passed'] for r in results)
    (out / 'summary.json').write_text(json.dumps(dict(
        schema=1, mode=f'variable_region_check_{args.mode}', navigation_accepted=False,
        regions=str(regions_path), names=list(regions.names), passed=passed,
        checks=results), indent=1) + '\n')
    print(f'{"PASSED" if passed else "FAILED"}: {out / "summary.json"}', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
