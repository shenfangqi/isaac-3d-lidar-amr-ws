#!/usr/bin/env python3
"""Read-only replay of the 3D localization decision on saved candidates.

For each view: the production 2D validation, the former 3D-prior path
(3D nomination re-validated through every 2D gate, now superseded) for
comparison, and the
production 3D decision (independent TRAIN/HOLDOUT cloud windows, refined
candidates, bounded support, 2D sanity).  Uses every candidate from a saved
diagnosis.  Reuses recorded scan stamps; does not re-run global search or
reconstruct executor scheduling.  Ground truth is read only after decisions.
No ROS node is started.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np

from diagnose_localization_bag import (
    read_bag, load_map, interpolate_pose, base_to_scan, sha256)
from recheck_candidates_3d import read_clouds, chain_to_base, same_pose
from isaac_3d_lidar_bringup.localization_contracts import (
    Keyframe, Hypothesis, SearchResult, SE2)
from isaac_3d_lidar_bringup import localization_hypotheses as lh
from isaac_3d_lidar_bringup import localization_surface_check as surface
from isaac_3d_lidar_bringup.localization_surface_validation import decide_surface


def reconstruct_frames(report, scans, odom, static, view):
    by_stamp = {s.stamp_ns: (received, s) for received, s in scans}
    stamps = [t for t, _ in odom]
    train, holdout = [], []
    for row in report['views']:
        if row['view_id'] > view['view_id']:
            break
        roles = [('TRAIN', row['train_stamps'], train)]
        if row['view_id'] == view['view_id']:
            roles.append(('HOLDOUT', row['holdout_stamps'], holdout))
        for role, selected, target in roles:
            for stamp in selected:
                received, scan = by_stamp[stamp]
                target.append(Keyframe(
                    len(train) + len(holdout), report['session'], stamp, row['view_id'],
                    scan, interpolate_pose(odom, stamps, stamp),
                    base_to_scan(scan.frame_id, static), received / 1e9, role))
    return tuple(train), tuple(holdout)


def reconstruct_points(bag, train, end_ns, window_s=5., max_per_cloud=6000):
    static, odom, clouds = read_clouds(bag, end_ns-int(window_s*1e9), end_ns)
    poses = [(t, SE2(x, y, yaw)) for t, x, y, yaw in odom]
    stamps = [t for t, _ in poses]
    reference = train[0].T_odom_base.inverse()
    parts, refused = [], 0
    for stamp, frame, points in clouds:
        try:
            pose = interpolate_pose(poses, stamps, stamp)
            before = interpolate_pose(poses, stamps, stamp-250_000_000)
            after = interpolate_pose(poses, stamps, stamp+250_000_000)
            delta = before.inverse().compose(after)
            if math.hypot(delta.x, delta.y)/.5 > .02 or abs(delta.yaw)/.5 > .03:
                raise ValueError('cloud is not stationary')
            relative = reference.compose(pose)
            matrix = (surface.planar(relative.x, relative.y, relative.yaw)
                      @ chain_to_base(frame, static))
        except ValueError:
            refused += 1
            continue
        points = points[np.isfinite(points).all(1)]
        if len(points) > max_per_cloud:
            points = points[np.linspace(0, len(points)-1, max_per_cloud).astype(int)]
        parts.append((matrix @ np.c_[points, np.ones(len(points))].T).T[:, :3])
    return (np.vstack(parts) if parts else np.zeros((0, 3))), len(parts), refused


def replay(diagnosis, map_path, mesh, truth_path=None, max_conflict=.35):
    report = json.loads(diagnosis.read_text())
    grid = load_map(map_path)
    if lh.map_hash(grid) != report['map_hash']:
        raise ValueError('diagnosis/map hash mismatch')
    bag = Path(report['bag'])
    scans, odom, statuses, static, counts, types, session = read_bag(bag, '/scan_localization')
    if session != report['session']:
        raise ValueError('diagnosis/bag session mismatch')
    config = lh.SearchConfig(**report['search_config'])
    thresholds = lh.ValidationThresholds(**report['thresholds'])
    vertices = surface.load_ply_vertices(mesh)
    rows = []
    for view in report['views']:
        train, holdout = reconstruct_frames(report, scans, odom, static, view)
        candidates = view['candidates']
        hypotheses = tuple(Hypothesis(
            **c['pose_at_reference'], score=c['train_score'], coverage=c['holdout']['coverage'],
            conflict=c['holdout']['conflict'], cluster_id=c['cluster_id'],
            per_view=tuple(c['train_per_view']), support_bounds=()) for c in candidates)
        result = SearchResult(session, report['map_hash'], view['search_complete'], hypotheses,
                              0, 0., view['search_reason'])
        initial = lh.validate_hypotheses(grid, result, train, holdout, config, thresholds,
                                         deadline=time.monotonic()+30)
        points, used, refused = reconstruct_points(bag, train, max(f.stamp_ns for f in holdout))
        poses = [(h.x, h.y, h.yaw) for h in hypotheses]
        started = time.monotonic()
        check = surface.check_surfaces(vertices, points, poses)
        duration = time.monotonic()-started
        final = initial
        if (result.complete and check.resolved and not initial.prior_conflict
                and initial.reason in ('AMBIGUOUS_LOCATION', 'NO_VALID_CANDIDATE')):
            prior = lh.PosePrior(SE2(*poses[check.leader]), .05, math.radians(2))
            final = lh.validate_hypotheses(grid, result, train, holdout, config, thresholds,
                                           deadline=time.monotonic()+30, prior=prior)
        end_ns = max(f.stamp_ns for f in holdout)
        independent_train, _, _ = reconstruct_points(
            bag, train, end_ns-2_500_000_001, window_s=2.5)
        independent_holdout, _, _ = reconstruct_points(bag, train, end_ns, window_s=2.5)
        started = time.monotonic()
        decision = decide_surface(
            vertices, independent_train, independent_holdout, poses, grid, holdout,
            train[0].T_odom_base, config, thresholds, max_conflict,
            deadline=time.monotonic()+60) if result.complete else None
        decision_duration = time.monotonic()-started
        rows.append(dict(view_id=view['view_id'], candidate_count=len(poses),
                         search_complete=result.complete, initial=asdict(initial),
                         surface=asdict(check), surface_duration_s=duration,
                         surface_pose=None if check.leader < 0 else poses[check.leader],
                         final=asdict(final), clouds_used=used, clouds_refused=refused,
                         surface_decision=None if decision is None else asdict(decision),
                         surface_decision_duration_s=decision_duration,
                         leader_2d_metrics=None if check.leader < 0 else lh.score_pose(
                             grid, SE2(*poses[check.leader]), holdout, train[0].T_odom_base,
                             config, config.refine_beams)))
    # Evaluation labels must never feed nomination or acceptance.
    if truth_path:
        truth = json.loads(truth_path.read_text())['true_candidate']
        expected = (truth['x'], truth['y'], math.radians(truth['yaw_deg']))
        for row in rows:
            winner = row['final']['winner']
            chosen = row['surface_decision']
            row['evaluation'] = dict(
                decision_matches_truth=None if not chosen or not chosen['accepted']
                else same_pose(chosen['pose'], expected),
                surface_matches_truth=(None if row['surface_pose'] is None
                                       else same_pose(row['surface_pose'], expected)),
                prior_path_matches_truth=None if winner is None else same_pose(
                    (winner['x'], winner['y'], winner['yaw']), expected))
    return dict(schema=1, navigation_accepted=False, mode='offline_decision_replay',
                diagnosis=str(diagnosis), diagnosis_sha256=sha256(diagnosis),
                map_hash=report['map_hash'], mesh_sha256=sha256(mesh),
                code_sha256={Path(m.__file__).name: sha256(Path(m.__file__))
                             for m in (lh, surface)},
                surface_config=asdict(surface.SurfaceCheckConfig()), views=rows,
                limitations=['Saved search candidates, no new global search.',
                             'Reconstructed source-time cloud windows ending at the last '
                             'holdout scan; not executor parity.',
                             'Production scoring and validation only; no AMCL, READY or '
                             'hardware validation.',
                             'All candidates are evaluated, within the current default '
                             'budget of 24.'],
                max_2d_conflict=max_conflict)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnosis', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--truth', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('refusing to overwrite existing evidence')
    result = replay(args.diagnosis, args.map, args.mesh, args.truth)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for row in result['views']:
        chosen = row['surface_decision'] or {}
        print(row['view_id'], '2D:', row['initial']['reason'] or 'ACCEPT',
              '3D-prior path:', row['final']['reason'] or 'ACCEPT',
              '3D decision:', chosen.get('reason', 'n/a') or 'ACCEPT', chosen.get('pose'),
              row.get('evaluation'), flush=True)


if __name__ == '__main__':
    main()
