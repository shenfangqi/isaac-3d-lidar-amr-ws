#!/usr/bin/env python3
"""
Repeat the 3D decision over many independent windows of one stationary bag.

Acceptance needs every run to succeed, so one window pair per place says
little.  The robot does not move in these captures: every disjoint pair of
2.5 s cloud windows (TRAIN then HOLDOUT, as in the manager) is an
independent repetition of the 3D decision at the same pose, with the same
candidate set (from the diagnosis) and the same 2D HOLDOUT frames.

For each pair: the positive decision (all candidates) and the negative one
(no starting candidate within --exclude-m of the truth).  Reports per bag
the share accepted, wrong acceptances and the minimum gap and fit.  Ground
truth is read only for scoring.  No ROS node is started.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from diagnose_localization_bag import interpolate_pose, load_map, read_bag  # noqa: E402
from isaac_3d_lidar_bringup import localization_surface_check as surface  # noqa: E402
from isaac_3d_lidar_bringup.localization_contracts import SE2  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from recheck_candidates_3d import chain_to_base, read_clouds  # noqa: E402
from replay_localization_surface_decision import (  # noqa: E402
    reconstruct_frames, same_pose)
import evaluate_surface_decisions as single  # noqa: E402

WINDOW_NS = 2_500_000_000


def clouds_in_reference(bag, train, max_per_cloud=6000):
    """Every stationary cloud of the bag in the reference base frame."""
    static, odom, clouds = read_clouds(bag, 0, 2 ** 62)
    poses = [(t, SE2(x, y, yaw)) for t, x, y, yaw in odom]
    stamps = [t for t, _ in poses]
    reference = train[0].T_odom_base.inverse()
    out = []
    for stamp, frame, points in clouds:
        try:
            pose = interpolate_pose(poses, stamps, stamp)
            before = interpolate_pose(poses, stamps, stamp - 250_000_000)
            after = interpolate_pose(poses, stamps, stamp + 250_000_000)
        except ValueError:
            continue
        delta = before.inverse().compose(after)
        if math.hypot(delta.x, delta.y) / .5 > .02 or abs(delta.yaw) / .5 > .03:
            continue
        relative = reference.compose(pose)
        matrix = (surface.planar(relative.x, relative.y, relative.yaw)
                  @ chain_to_base(frame, static))
        points = points[np.isfinite(points).all(1)]
        if len(points) > max_per_cloud:
            points = points[np.linspace(0, len(points) - 1, max_per_cloud).astype(int)]
        out.append((stamp, ((matrix @ np.c_[points, np.ones(len(points))].T).T[:, :3])
                    .astype(np.float32)))
    origin = chain_to_base(clouds[-1][1], static)[:3, 3]
    return out, origin


def evaluate_bag(job):
    entry, mesh, map_path, spacing, exclude, max_pairs = job
    report = json.loads(Path(ROOT / entry['diagnosis']).read_text())
    view = report['views'][-1]
    bag = Path(report['bag'])
    bag = bag if bag.is_absolute() else ROOT / bag
    scans, odom, _st, static, _c, _t, _session = read_bag(bag, '/scan_localization')
    report['views'] = [v for v in report['views'] if v['view_id'] <= view['view_id']]
    train, holdout = reconstruct_frames(report, scans, odom, static, view)
    clouds, origin = clouds_in_reference(bag, train)
    grid = load_map(map_path)
    surface_points = load_surface_model(mesh, spacing).points
    poses = [tuple(c['pose_at_reference'][k] for k in ('x', 'y', 'yaw'))
             for c in view['candidates']]
    seeds = [tuple(s) for s in view.get('unrefined', [])]
    stamps = np.array([s for s, _ in clouds])
    starts = np.arange(stamps[0], stamps[-1] - 2 * WINDOW_NS, 2 * WINDOW_NS)
    if max_pairs and len(starts) > max_pairs:
        starts = starts[np.linspace(0, len(starts) - 1, max_pairs).astype(int)]
    sample = dict(report=report, view=view, train=train, holdout=holdout, origin=origin)
    truth_doc = json.loads(Path(ROOT / entry['truth']).read_text())['true_candidate']
    truth = (truth_doc['x'], truth_doc['y'], math.radians(truth_doc['yaw_deg']))
    far = [p for p in poses if math.hypot(p[0] - truth[0], p[1] - truth[1]) > exclude]
    far_seeds = [p for p in seeds if math.hypot(p[0] - truth[0], p[1] - truth[1]) > exclude]
    rows = []
    for start in starts:
        windows = []
        for lo in (start, start + WINDOW_NS):
            chosen = [p for s, p in clouds if lo <= s < lo + WINDOW_NS]
            windows.append(np.vstack(chosen) if chosen else np.zeros((0, 3)))
        # Gaps in the recording (recorder losses) leave short windows; the
        # manager would refuse them too, so they are not repetitions.
        if min(len(w) for w in windows) < 20000:
            continue
        sample['first'], sample['second'] = windows
        positive = single.decide(surface_points, sample, grid, poses, seeds, .35, True)
        negative = single.decide(surface_points, sample, grid, far, far_seeds, .35, True)
        rows.append(dict(
            start_s=round((start - stamps[0]) / 1e9, 1),
            accepted=positive['accepted'], reason=positive['reason'],
            leader_is_truth=(positive['leader'] is not None
                             and same_pose(positive['leader'], truth)),
            gaps=positive['gaps'], fit=positive['metrics_2d'].get('fit_3d'),
            negative_accepted=negative['accepted'], negative_reason=negative['reason'],
            negative_gaps=negative.get('gaps')))
    return entry['name'], rows


def summary(name, rows):
    accepted = [r for r in rows if r['accepted']]
    gaps = [min(g for g in r['gaps'] if g is not None) for r in rows
            if r['gaps'] and None not in r['gaps']]
    fits = [r['fit'] for r in accepted if r['fit'] is not None]
    reasons = {}
    for r in rows:
        if not r['accepted']:
            reasons[r['reason']] = reasons.get(r['reason'], 0) + 1
    return dict(
        name=name, pairs=len(rows), accepted=len(accepted),
        wrong_leader_accepted=sum(1 for r in accepted if not r['leader_is_truth']),
        negative_wrong_accepts=sum(1 for r in rows if r['negative_accepted']),
        min_gap=round(min(gaps), 4) if gaps else None,
        min_fit=round(min(fits), 4) if fits else None, refusals=reasons)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=.02)
    parser.add_argument('--exclude-m', type=float, default=1.2)
    parser.add_argument('--max-pairs', type=int, default=0)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    load_surface_model(args.mesh, args.spacing)          # build the cache once
    jobs = [(entry, args.mesh, args.map, args.spacing, args.exclude_m, args.max_pairs)
            for entry in json.loads(args.samples.read_text())]
    results = {}
    with ProcessPoolExecutor(args.workers) as pool:
        for name, rows in pool.map(evaluate_bag, jobs):
            results[name] = dict(summary=summary(name, rows), pairs=rows)
            print(json.dumps(results[name]['summary']), flush=True)
    args.output.write_text(json.dumps(dict(
        schema=1, mode='offline_window_repeats', navigation_accepted=False,
        spacing=args.spacing, exclude_m=args.exclude_m, results=results), indent=1) + '\n')


if __name__ == '__main__':
    np.seterr(all='ignore')
    main()
