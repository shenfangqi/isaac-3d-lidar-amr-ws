#!/usr/bin/env python3
"""
Read-only evaluation of the 3D localization decision on labelled bags.

For every sample (a diagnosis JSON from diagnose_localization_bag.py plus an
operator ground-truth JSON) and every surface representation, runs the
production ``decide_surface`` on two independent 2.5 s cloud windows and
reports the verdict, the gaps, the fit and whether the leader is the truth.

Negative check: the same decision without any starting candidate within
``--exclude-m`` of the truth (beyond the 1 m refinement reach), so it can
only refuse or accept a wrong pose.  Ground truth is read only after the
decisions.  No ROS node is started; nothing is published.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from diagnose_localization_bag import load_map, read_bag  # noqa: E402
from isaac_3d_lidar_bringup import localization_hypotheses as lh  # noqa: E402
from isaac_3d_lidar_bringup import localization_surface_check as surface  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_search import (  # noqa: E402
    build_surface_field, free_positions, search_surface, SurfaceSearchConfig)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    decide_surface)
from recheck_candidates_3d import chain_to_base, read_clouds  # noqa: E402
from replay_localization_surface_decision import (  # noqa: E402
    reconstruct_frames, reconstruct_points, same_pose)


def load_sample(diagnosis):
    report = json.loads(Path(diagnosis).read_text())
    view = report['views'][-1]
    bag = Path(report['bag'])
    if not bag.is_absolute():
        bag = ROOT / bag
    scans, odom, _statuses, static, _counts, _types, session = read_bag(
        bag, '/scan_localization')
    if session != report['session']:
        raise ValueError(f'{diagnosis}: diagnosis/bag session mismatch')
    report['views'] = [v for v in report['views'] if v['view_id'] <= view['view_id']]
    train, holdout = reconstruct_frames(report, scans, odom, static, view)
    end_ns = max(f.stamp_ns for f in holdout)
    first, _, _ = reconstruct_points(bag, train, end_ns - 2_500_000_001, window_s=2.5)
    second, _, _ = reconstruct_points(bag, train, end_ns, window_s=2.5)
    static, _odom, clouds = read_clouds(bag, end_ns - 1_000_000_000, end_ns)
    # Sensor position in the base frame (stationary capture).
    origin = chain_to_base(clouds[-1][1], static)[:3, 3]
    poses = [tuple(c['pose_at_reference'][k] for k in ('x', 'y', 'yaw'))
             for c in view['candidates']]
    seeds = [tuple(s) for s in view.get('unrefined', [])]
    return dict(report=report, view=view, train=train, holdout=holdout,
                first=first, second=second, poses=poses, seeds=seeds, origin=origin)


def decide(surface_points, sample, grid, poses, seeds, max_conflict, see_through):
    report = sample['report']
    config = lh.SearchConfig(**report['search_config'])
    thresholds = lh.ValidationThresholds(**report['thresholds'])
    started = time.monotonic()
    decision = decide_surface(
        surface_points, sample['first'], sample['second'], poses, grid,
        sample['holdout'], sample['train'][0].T_odom_base, config, thresholds,
        max_conflict, deadline=time.monotonic() + 600, seed_poses=seeds,
        sensor_origin=sample['origin'] if see_through else None)
    validation = decision.validation
    leader = (decision.pose or (validation.poses[validation.leader]
                                if validation and 0 <= validation.leader < len(validation.poses)
                                else None))
    parts = (validation.train, validation.holdout) if validation else (None, None)
    return dict(
        accepted=decision.accepted, reason=decision.reason,
        leader=None if leader is None else [round(v, 4) for v in leader],
        gaps=[None if p is None else p.composite_gap for p in parts],
        leader_composite=[None if p is None or p.leader < 0 else p.composite[p.leader]
                          for p in parts],
        metrics_2d=decision.metrics_2d,
        support_extents=list(validation.support_extents) if validation else [],
        see_through=list(validation.see_through) if validation else [],
        candidates=len(validation.poses) if validation and validation.poses else None,
        seconds=round(time.monotonic() - started, 2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=Path, required=True,
                        help='JSON list of {name, diagnosis, truth}')
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--spacings', default='0,0.02',
                        help='comma list; 0 = mesh vertices only')
    parser.add_argument('--exclude-m', type=float, default=1.2)
    parser.add_argument('--max-conflict', type=float, default=.35)
    parser.add_argument('--see-through', choices=('on', 'off'), default='on')
    parser.add_argument('--search3d', choices=('off', 'union'), default='off',
                        help='union: add the map-wide 3D search candidates (stage D)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    grid = load_map(args.map)
    models = {}
    for text in args.spacings.split(','):
        spacing = float(text)
        models[text] = (surface.load_ply_vertices(args.mesh) if spacing == 0
                        else load_surface_model(args.mesh, spacing).points)
    rows = []
    for entry in json.loads(args.samples.read_text()):
        sample = load_sample(ROOT / entry['diagnosis'])
        if lh.map_hash(grid) != sample['report']['map_hash']:
            raise ValueError(f'{entry["name"]}: map hash mismatch')
        row = dict(name=entry['name'], candidates=len(sample['poses']),
                   seeds=len(sample['seeds']), search_complete=sample['view']['search_complete'],
                   points=[len(sample['first']), len(sample['second'])], models={})
        # Truth is read only after the decisions below are fixed in code.
        truth_doc = json.loads((ROOT / entry['truth']).read_text())['true_candidate']
        truth = (truth_doc['x'], truth_doc['y'], math.radians(truth_doc['yaw_deg']))
        far = [p for p in sample['poses']
               if math.hypot(p[0] - truth[0], p[1] - truth[1]) > args.exclude_m]
        far_seeds = [p for p in sample['seeds']
                     if math.hypot(p[0] - truth[0], p[1] - truth[1]) > args.exclude_m]
        if args.search3d == 'union':
            config = SurfaceSearchConfig()
            field = build_surface_field(next(iter(models.values())), config)
            found = search_surface(field, sample['first'], free_positions(grid, config), config)
            row['search3d'] = dict(complete=found.complete, reason=found.reason,
                                   candidates=len(found.candidates))
            sample['seeds'] = sample['seeds'] + [c[:3] for c in found.candidates]
            far_seeds += [c[:3] for c in found.candidates
                          if math.hypot(c[0] - truth[0], c[1] - truth[1]) > args.exclude_m]
        for name, points in models.items():
            on = args.see_through == 'on'
            positive = decide(points, sample, grid, sample['poses'], sample['seeds'],
                              args.max_conflict, on)
            positive['leader_is_truth'] = (positive['leader'] is not None
                                           and same_pose(positive['leader'], truth))
            negative = decide(points, sample, grid, far, far_seeds, args.max_conflict, on) \
                if len(far) + len(far_seeds) >= 2 else dict(accepted=False, reason='TOO_FEW')
            negative['wrong_accept'] = bool(negative['accepted'])
            row['models'][name] = dict(positive=positive, negative=negative)
            verdict = 'ACCEPT' if positive['accepted'] else positive['reason']
            against = 'WRONG_ACCEPT' if negative['accepted'] else negative['reason']
            print(f"{entry['name']:12s} {name:5s} +{verdict:30s}"
                  f" truth={positive['leader_is_truth']!s:5s} gaps={positive['gaps']}"
                  f" lead={positive['leader_composite']}"
                  f" fit={positive['metrics_2d'].get('fit_3d')} t={positive['seconds']}s"
                  f" | -{against} gaps={negative.get('gaps')}", flush=True)
        rows.append(row)
    args.output.write_text(json.dumps(dict(
        schema=1, mode='offline_surface_decision_evaluation', navigation_accepted=False,
        mesh=str(args.mesh), spacings=list(models), exclude_m=args.exclude_m,
        see_through=args.see_through, search3d=args.search3d,
        samples=rows), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist')
        else asdict(o)) + '\n')


if __name__ == '__main__':
    np.seterr(all='ignore')
    main()
