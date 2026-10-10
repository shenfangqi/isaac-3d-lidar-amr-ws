#!/usr/bin/env python3
"""
Where does the scene differ from the map?  (Issue #13 stage A, read-only)

Given a stationary capture (a diagnosis JSON from
diagnose_localization_bag.py) and a confirmed pose, compares the 3D cloud
with the map surface in both directions:

- added:   observed points with no mapped surface nearby (a new object, a
           closed curtain in front of an open window);
- removed: rays that pass through mapped structure before their end (an
           object taken away, a door or curtain opened since mapping).

Both are summarised on a 0.5 m map grid with height ranges, so a map update
or a variable-region decision can be made per region.  The pose must be
confirmed independently (operator or an accepted localization); a wrong
pose shows up as disagreement everywhere.  No ROS node is started.
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from evaluate_surface_decisions import load_sample  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_check import (  # noqa: E402
    planar, surface_tree)
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    SurfaceConflictConfig, surface_occupancy, _voxel_keys)


def removed_crossings(occupancy, points, origin, pose, config):
    """Map positions where rays first cross mapped structure."""
    matrix = planar(*pose)
    world = (matrix @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    start = (matrix @ np.r_[np.asarray(origin, float), 1.])[:3]
    delta = world - start
    length = np.linalg.norm(delta, axis=1)
    direction = delta / np.maximum(length, 1e-9)[:, None]
    end = np.minimum(length - config.end_margin_m, config.max_range_m)
    steps = np.arange(config.start_m, max(float(end.max()), config.start_m), config.voxel_m / 2)
    samples = start + direction[:, None, :] * steps[None, :, None]
    keys = _voxel_keys(samples, config.voxel_m)
    slot = np.searchsorted(occupancy, keys)
    hit = ((occupancy[np.minimum(slot, len(occupancy) - 1)] == keys)
           & (steps[None, :] < end[:, None]))
    crossed = hit.any(1)
    first = hit.argmax(1)
    return samples[crossed, first[crossed]], int((end > config.start_m).sum())


def regions(points, total, cell=.5, top=12):
    if not len(points):
        return []
    keys = np.floor(points[:, :2] / cell).astype(int)
    unique, counts = np.unique(keys, axis=0, return_counts=True)
    out = []
    for index in np.argsort(-counts)[:top]:
        mask = (keys == unique[index]).all(1)
        z = points[mask, 2]
        out.append(dict(cell_center=[round(float(v), 2) for v in (unique[index] + .5) * cell],
                        points=int(counts[index]), share=round(float(counts[index]) / total, 4),
                        z_p10_p90=[round(float(v), 2) for v in np.percentile(z, [10, 90])]))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('diagnosis', type=Path)
    parser.add_argument('--pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--truth', type=Path, help='operator ground-truth JSON')
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=.02)
    parser.add_argument('--added-m', type=float, default=.15,
                        help='an observed point farther than this from the map is "added"')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (args.pose is None) == (args.truth is None):
        parser.error('give exactly one of --pose or --truth')
    if args.truth:
        truth = json.loads(args.truth.read_text())['true_candidate']
        pose = (truth['x'], truth['y'], math.radians(truth['yaw_deg']))
    else:
        pose = (args.pose[0], args.pose[1], math.radians(args.pose[2]))
    sample = load_sample(args.diagnosis)
    points = np.r_[sample['first'], sample['second']]
    points = points[np.hypot(points[:, 0], points[:, 1]) >= .5]
    model = load_surface_model(args.mesh, args.spacing)
    tree = surface_tree(model.points)
    world = (planar(*pose) @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    distance = tree.query(world, distance_upper_bound=1.0)[0]
    added = world[distance > args.added_m]
    config = SurfaceConflictConfig()
    rays = points[points[:, 2] > config.min_point_z_m]
    rays = rays[np.linspace(0, len(rays) - 1, min(len(rays), 20000)).astype(int)]
    crossings, tested = removed_crossings(
        surface_occupancy(model.points, config), rays, sample['origin'], pose, config)
    report = dict(
        schema=1, mode='offline_scene_consistency', diagnosis=str(args.diagnosis),
        pose=[pose[0], pose[1], math.degrees(pose[2])], mesh_sha256=model.mesh_sha256,
        points=int(len(points)),
        added=dict(threshold_m=args.added_m, share=round(len(added) / len(points), 4),
                   regions=regions(added, len(points))),
        removed=dict(rays=tested, share=round(len(crossings) / max(tested, 1), 4),
                     regions=regions(crossings, max(tested, 1))),
        limitations=['Pose given, not estimated: a wrong pose shows as disagreement '
                     'everywhere.', 'Added points include people and anything else in '
                     'view during the capture.', 'Removed uses the see-through rules '
                     'of SurfaceConflictConfig (height band, end margin).'])
    args.output.write_text(json.dumps(report, indent=1) + '\n')
    print(f"added {report['added']['share']:.1%} of points, removed "
          f"{report['removed']['share']:.1%} of rays")
    for kind in ('added', 'removed'):
        for region in report[kind]['regions'][:6]:
            print(f"  {kind:7s} cell {region['cell_center']} {region['share']:.1%} "
                  f"z {region['z_p10_p90']}")


if __name__ == '__main__':
    main()
