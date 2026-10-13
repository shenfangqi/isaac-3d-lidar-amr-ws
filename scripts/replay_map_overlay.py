#!/usr/bin/env python3
"""
Replay the automatic map overlay (#54) over labelled sessions in time order.  (offline)

Every session is first localized by the production 3D decision
(``decide_surface`` with the map-wide 3D search, as the manager runs it)
against the mesh, the variable regions, the base additions and the overlay
learned from the EARLIER sessions only.  The same decision without any
candidate near the truth must refuse (negative check).  Then the session
updates the overlay only if it passes ``session_gate`` at the decided pose
(never the truth), whole-bag submap evidence is recorded, and a commit
passes ``self_check`` on the session's own cloud or is undone.  Every
version is kept in --store (``OverlayStore``).  Truth is used only to label
the outcome.

AMCL is not replayed: the gate gets ``--amcl-std`` (the 2026-10-09..12
sessions verified at about 2 cm / 1 deg).  No ROS node is started.
"""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from check_submap_changes import read_frames  # noqa: E402
from diagnose_localization_bag import load_map  # noqa: E402
from evaluate_surface_decisions import load_sample, SCAN_BAND  # noqa: E402
from isaac_3d_lidar_bringup import localization_hypotheses as lh  # noqa: E402
from isaac_3d_lidar_bringup.localization_map_overlay import (  # noqa: E402
    observe, OutcomeTracker, OverlayConfig, OverlayState, OverlayStore, self_check,
    SessionQuality, undo_commits, update_overlay)
from isaac_3d_lidar_bringup.localization_submap import build_submap  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_additions import (  # noqa: E402
    load_surface_additions)
from isaac_3d_lidar_bringup.localization_surface_check import (  # noqa: E402
    planar, surface_tree)
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_search import (  # noqa: E402
    build_surface_field, free_positions, prune_seeds, search_surface, SurfaceSearchConfig)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    _smooth_fit, decide_surface, refine_surface_pose, SurfaceRefineConfig)
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions, mask_grid)
from recheck_candidates_3d import same_pose  # noqa: E402


def decide(points, regions, additions, sample, grid, scoring, truth=None, exclude_m=1.2):
    """
    The production 3D decision with the map-wide 3D search (stage D).

    With ``truth``, the negative check: every candidate within ``exclude_m``
    of it, from 2D or from the 3D search, is removed first (as
    evaluate_surface_decisions.py --search3d union does), so the decision
    can only refuse or accept a wrong pose.
    """
    report = sample['report']
    config = lh.SearchConfig(**report['search_config'])
    thresholds = lh.ValidationThresholds(**report['thresholds'])
    extra = None if additions is None or not len(additions) else additions
    fitted = points if extra is None else np.vstack([points, extra])
    search_config = SurfaceSearchConfig()
    field = build_surface_field(fitted, search_config, regions)
    positions = free_positions(grid, search_config)
    poses, seeds, search = sample['poses'], sample['seeds'], (field, positions, search_config)
    if truth is not None:
        found = search_surface(field, sample['first'], positions, search_config)
        if not found.complete:
            return None
        seeds = (list(prune_seeds(field, sample['first'], seeds, found, search_config))
                 + [c[:3] for c in found.candidates])

        def far(pose):
            return math.hypot(pose[0] - truth[0], pose[1] - truth[1]) > exclude_m
        poses, seeds, search = [p for p in poses if far(p)], [p for p in seeds if far(p)], None
        if len(poses) + len(seeds) < 2:
            return None
    return decide_surface(
        points, sample['first'], sample['second'], poses, scoring, sample['holdout'],
        sample['train'][0].T_odom_base, config, thresholds, .35,
        deadline=time.monotonic() + 900, seed_poses=seeds, sensor_origin=sample['origin'],
        search=search, regions=regions, additions=extra)


def gaps(decision):
    validation = decision.validation
    if validation is None:
        return []
    return [p.composite_gap for p in (validation.train, validation.holdout) if p is not None]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--samples', type=Path, required=True,
                        help='JSON list of {name, diagnosis, truth}, in time order')
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--variable-regions', type=Path, required=True)
    parser.add_argument('--base-additions', type=Path,
                        help='operator surface additions kept under the overlay')
    parser.add_argument('--store', type=Path, required=True, help='new overlay directory')
    parser.add_argument('--amcl-std', type=float, nargs=2, default=(.02, 1.),
                        metavar=('XY_M', 'YAW_DEG'))
    parser.add_argument('--exclude-m', type=float, default=1.2)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.store.exists():
        parser.error('output or store exists; remove it first')

    model = load_surface_model(args.mesh, .02)
    points = model.points
    regions = load_variable_regions(args.variable_regions, model.mesh_sha256)
    base = (load_surface_additions(args.base_additions, model.mesh_sha256)
            if args.base_additions else np.zeros((0, 3)))
    grid = load_map(args.map)
    scoring = mask_grid(grid, regions, *SCAN_BAND)
    store = OverlayStore(args.store, model.mesh_sha256, base)
    config = OverlayConfig()
    state = OverlayState(model.mesh_sha256)
    store.save(state, 'empty')
    tracker = OutcomeTracker()
    rows = []
    for entry in json.loads(args.samples.read_text()):
        started = time.monotonic()
        sample = load_sample(ROOT / entry['diagnosis'])
        additions = np.vstack([base, state.surface()])
        decision = decide(points, regions, additions, sample, grid, scoring)
        truth_doc = json.loads((ROOT / entry['truth']).read_text())['true_candidate']
        truth = (truth_doc['x'], truth_doc['y'], math.radians(truth_doc['yaw_deg']))
        negative = decide(points, regions, additions, sample, grid, scoring, truth,
                          args.exclude_m)
        wrong = bool(negative is not None and negative.accepted)
        gap = min(gaps(decision)) if gaps(decision) else float('nan')
        quality = SessionQuality(decision.accepted, gap, args.amcl_std[0],
                                 math.radians(args.amcl_std[1]))
        row = dict(name=entry['name'], version_used=state.version,
                   overlay_points=int(len(state.surface())), accepted=decision.accepted,
                   reason=decision.reason, gaps=gaps(decision),
                   leader_is_truth=bool(decision.pose and same_pose(decision.pose, truth)),
                   negative=('TOO_FEW' if negative is None else 'WRONG_ACCEPT' if wrong
                             else negative.reason),
                   wrong_accept=wrong)
        if tracker.record(decision.accepted):
            parent = tracker.parent
            store.rollback(parent)
            state = store.load(parent)
            row['rolled_back_to'] = parent
            tracker.start(parent, -1)
        if decision.accepted:
            pose = decision.pose
            frames, _ = read_frames(ROOT / sample['report']['bag'])
            reference = sample['train'][0].T_odom_base
            submap = build_submap(frames, planar(reference.x, reference.y, reference.yaw))
            tree = surface_tree(points, regions, additions)
            evidence = observe(state, submap, pose, tree, regions, config)
            new, report = update_overlay(state, quality, evidence, config)
            row['update'] = dict(usable=report.usable, reason=report.reason,
                                 added_points=int(len(evidence.added)),
                                 added_share=round(evidence.added_share, 4),
                                 occupied=report.observed_cells, free=report.free_cells,
                                 promoted=report.promoted, retracted=report.retracted,
                                 deferred=report.deferred,
                                 refused=list(report.refused_components))
            if report.usable and (report.promoted or report.retracted):
                cloud = np.vstack([sample['first'], sample['second']])
                old_tree = tree
                new_tree = surface_tree(points, regions, np.vstack([base, new.surface()]))
                local = SurfaceRefineConfig(max_shift_m=.15, max_turn_rad=math.radians(5))

                def fit(candidate, use_new):
                    chosen = new_tree if use_new else old_tree
                    refined, _ = refine_surface_pose(chosen, cloud, candidate, local,
                                                     lambda: False)
                    sampled = cloud[np.linspace(0, len(cloud) - 1,
                                                min(len(cloud), 3000)).astype(int)]
                    return _smooth_fit(chosen, np.c_[sampled, np.ones(len(sampled))],
                                       .05)(np.asarray(refined))
                rivals = [p for i, p in enumerate(decision.validation.poses)
                          if i != decision.validation.leader
                          and not same_pose(p, pose, xy=.5, yaw=math.radians(20))]
                check = self_check(fit, pose, rivals)
                row['self_check'] = dict(keep=check.keep, reason=check.reason,
                                         leader_fit=check.leader_fit, lead=check.lead,
                                         rivals=len(rivals))
                if not check.keep:
                    new = undo_commits(state, new)
                elif report.promoted:
                    tracker.start(new.version, state.version)
            if report.usable:
                state = replace(new, version=store.next_version())
                store.save(state, entry['name'])
        row['seconds'] = round(time.monotonic() - started, 1)
        rows.append(row)
        update = row.get('update', {})
        print(f"{entry['name']:11s} v{row['version_used']:<2d} overlay {row['overlay_points']:5d} "
              f"{'ACCEPT' if decision.accepted else decision.reason:28s} "
              f"truth={row['leader_is_truth']!s:5s} gaps={[round(g, 3) for g in row['gaps']]} "
              f"neg={row['negative']} | {update.get('reason', '-')} "
              f"share {update.get('added_share', '-')} "
              f"+{update.get('promoted', 0)} -{update.get('retracted', 0)} "
              f"defer {update.get('deferred', 0)} refused {update.get('refused', [])} "
              f"check {row.get('self_check', {}).get('reason', '-')} {row['seconds']} s",
              flush=True)
    args.output.write_text(json.dumps(dict(
        schema=1, mode='offline_map_overlay_replay', navigation_accepted=False,
        mesh_sha256=model.mesh_sha256, base_additions=str(args.base_additions or ''),
        config=config.__dict__, amcl_std_assumed=list(args.amcl_std),
        final_version=state.version, final_overlay_points=int(len(state.surface())),
        sessions=rows), indent=1, default=float) + '\n')


if __name__ == '__main__':
    main()
