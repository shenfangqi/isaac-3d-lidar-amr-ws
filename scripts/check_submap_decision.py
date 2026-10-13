#!/usr/bin/env python3
"""
3D evidence of a multi-stop submap (issue #53).  (offline, read-only)

Splits a bag into stops by odometry (still for at least --min-stop-s:
speed under 0.02 m/s and turn rate under 0.03 rad/s over +-0.25 s, as the
stationary capture requires), takes two independent --window-s windows at
the end of each stop, builds the submap of the last --stops stops
(localization_submap_decision.submap_windows) and runs the map-wide 3D
search and validate_surface_evidence on it, with every point traced from
its own stop for the see-through check.

This is the 3D evidence only: the production decision also checks the
leader on the 2D map, which needs the manager's 2D keyframes.  A stationary
bag gives one stop, the stationary 3D evidence over longer windows.  The
truth (optional) labels the result only.  No ROS node is started.
"""

import argparse
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
from isaac_3d_lidar_bringup.localization_submap_decision import (  # noqa: E402
    Stop, SubmapDecisionConfig, submap_windows)
from isaac_3d_lidar_bringup.localization_surface_additions import (  # noqa: E402
    load_surface_additions)
from isaac_3d_lidar_bringup.localization_surface_check import surface_tree  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_search import (  # noqa: E402
    build_surface_field, free_positions, search_surface, SurfaceSearchConfig)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    SurfaceRefineConfig, validate_surface_evidence)
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions)
from recheck_candidates_3d import same_pose  # noqa: E402


def still(odom, stamp, max_speed=.02, max_turn=.03):
    """Whether odometry shows no motion over +-0.25 s around ``stamp``."""
    table = np.asarray(odom, float)
    if not table[0, 0] <= stamp - 250_000_000 or not stamp + 250_000_000 <= table[-1, 0]:
        return False
    yaw = np.unwrap(table[:, 3])
    before, after = ([np.interp(t, table[:, 0], column)
                      for column in (table[:, 1], table[:, 2], yaw)]
                     for t in (stamp - 250_000_000, stamp + 250_000_000))
    return (math.hypot(after[0] - before[0], after[1] - before[1]) / .5 <= max_speed
            and abs(after[2] - before[2]) / .5 <= max_turn)


def find_stops(frames, odom, min_stop_s, window_s):
    """Runs of still frames lasting ``min_stop_s``; two end windows of each."""
    runs, current = [], []
    for frame in frames:
        if still(odom, frame.stamp_ns):
            current.append(frame)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    stops = []
    for run in runs:
        end = run[-1].stamp_ns
        if (end - run[0].stamp_ns) / 1e9 < min_stop_s:
            continue
        window = int(window_s * 1e9)
        first = tuple(f for f in run if end - 2 * window < f.stamp_ns <= end - window)
        second = tuple(f for f in run if end - window < f.stamp_ns <= end)
        if first and second:
            stops.append(Stop(first, second))
    return stops


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--variable-regions', type=Path)
    parser.add_argument('--surface-additions', type=Path)
    parser.add_argument('--stops', type=int, default=4, help='use the last N stops')
    parser.add_argument('--min-stop-s', type=float, default=6.)
    parser.add_argument('--window-s', type=float, default=2.5)
    parser.add_argument('--max-path-m', type=float, default=SubmapDecisionConfig.max_path_m)
    parser.add_argument('--truth', type=Path, help='robot pose at the LAST stop')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'{args.output} exists; remove it first')
    started = time.monotonic()
    model = load_surface_model(args.mesh, .02)
    regions = (load_variable_regions(args.variable_regions, model.mesh_sha256)
               if args.variable_regions else None)
    additions = (load_surface_additions(args.surface_additions, model.mesh_sha256)
                 if args.surface_additions else None)
    frames, odom = read_frames(args.bag)
    stops = find_stops(frames, odom, args.min_stop_s, args.window_s)[-args.stops:]
    if not stops:
        parser.error('no stop of --min-stop-s found')
    config = SubmapDecisionConfig(max_path_m=args.max_path_m, max_stops=max(len(stops), 1))
    train, holdout, origins, path = submap_windows(stops, config)
    fitted = model.points if additions is None else np.vstack([model.points, additions])
    search_config = SurfaceSearchConfig()
    field = build_surface_field(fitted, search_config, regions)
    found = search_surface(field, train, free_positions(load_map(args.map), search_config),
                           search_config)
    report = dict(schema=1, mode='offline_submap_3d_evidence', bag=str(args.bag),
                  mesh_sha256=model.mesh_sha256, stops=len(stops), path_m=round(path, 3),
                  points=[len(train), len(holdout)], search_complete=found.complete,
                  search_reason=found.reason, candidates=len(found.candidates))
    if found.complete and found.candidates:
        tree = surface_tree(model.points, regions, additions)
        result = validate_surface_evidence(
            tree, train, holdout, [c[:3] for c in found.candidates],
            deadline=time.monotonic() + 900, refine_config=SurfaceRefineConfig(),
            sensor_origin=origins)
        leader = (result.poses[result.leader]
                  if result.poses and 0 <= result.leader < len(result.poses) else None)
        report.update(supported=result.supported, reason=result.reason,
                      leader=None if leader is None else [round(v, 4) for v in leader],
                      gaps=[None if p is None else p.composite_gap
                            for p in (result.train, result.holdout)])
        if args.truth and leader is not None:
            truth = json.loads(args.truth.read_text())['true_candidate']
            report['leader_is_truth'] = same_pose(
                leader, (truth['x'], truth['y'], math.radians(truth['yaw_deg'])))
    report['seconds'] = round(time.monotonic() - started, 1)
    args.output.write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps({k: report.get(k) for k in (
        'stops', 'path_m', 'points', 'candidates', 'supported', 'reason', 'leader', 'gaps',
        'leader_is_truth', 'seconds')}))


if __name__ == '__main__':
    main()
