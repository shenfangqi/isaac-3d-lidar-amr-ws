#!/usr/bin/env python3
"""
Measure static structure the mesh misses, for the 3D fit.  (read-only input)

Inputs are stationary captures (diagnosis JSONs from
diagnose_localization_bag.py --stationary-capture) with operator-confirmed
poses.  Observed points inside --box that lie farther than --min-distance
from the mapped surface (and outside the variable regions) are collected,
voxelized at --voxel and kept where at least --min-count points agree.  The
result is a surface-additions NPZ bound to the mesh SHA-256
(isaac_3d_lidar_bringup/localization_surface_additions.py): fitted by the
3D scoring, never solid for see-through.

Measure from captures other than the ones used to test the result, and only
for structure the operator confirms is permanent (2026-10-12: a bed the
low MID-360 did not see from a distance while mapping).
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from evaluate_surface_decisions import load_sample  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_additions import (  # noqa: E402
    save_surface_additions)
from isaac_3d_lidar_bringup.localization_surface_check import planar  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    file_sha256, load_surface_model)
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions)


def missing_points(world, box, tree, min_distance, regions):
    """Observed points in ``box`` with no mapped surface within ``min_distance``."""
    inside = ((world >= box[:3]) & (world <= box[3:])).all(1)
    if regions is not None:
        inside &= ~regions.contains(world)
    candidates = world[inside]
    distance, _ = tree.query(candidates, distance_upper_bound=min_distance + 1.)
    return candidates[distance > min_distance]


def voxel_points(points, voxel, min_count):
    """Voxel centres holding at least ``min_count`` points."""
    keys, counts = np.unique(np.round(points / voxel).astype(int), axis=0,
                             return_counts=True)
    return keys[counts >= min_count] * voxel


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--capture', nargs=2, action='append', required=True,
                        metavar=('DIAGNOSIS', 'TRUTH'),
                        help='stationary-capture diagnosis JSON and its ground-truth JSON')
    parser.add_argument('--box', type=float, nargs=6, required=True,
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'),
                        help='map-frame box of the missing structure')
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=.02)
    parser.add_argument('--variable-regions', type=Path)
    parser.add_argument('--min-distance', type=float, default=.10)
    parser.add_argument('--voxel', type=float, default=.02)
    parser.add_argument('--min-count', type=int, default=2)
    parser.add_argument('--source', default='', help='what the additions are (stored)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sha = file_sha256(args.mesh)
    tree = cKDTree(load_surface_model(args.mesh, args.spacing).points)
    regions = (load_variable_regions(args.variable_regions, sha)
               if args.variable_regions else None)
    box = np.asarray(args.box, float)
    found = []
    for diagnosis, truth_path in args.capture:
        truth = json.loads(Path(truth_path).read_text())['true_candidate']
        pose = (truth['x'], truth['y'], math.radians(truth['yaw_deg']))
        sample = load_sample(diagnosis)
        points = np.vstack([sample['first'], sample['second']])
        world = (planar(*pose) @ np.c_[points, np.ones(len(points))].T).T[:, :3]
        missing = missing_points(world, box, tree, args.min_distance, regions)
        print(f'{Path(diagnosis).name}: {len(missing)} of {len(world)} points missing '
              f'from the mesh in the box', flush=True)
        found.append(missing)
    additions = voxel_points(np.vstack(found), args.voxel, args.min_count)
    if not len(additions):
        raise SystemExit('no missing structure found in the box')
    source = (f'{args.source}; captures: '
              + ', '.join(Path(d).name for d, _ in args.capture)
              + f'; box {args.box}; min distance {args.min_distance} m')
    save_surface_additions(args.output, additions, sha, source)
    heights = np.percentile(additions[:, 2], [5, 50, 95]).round(2).tolist()
    print(f'{len(additions)} points, bbox {additions.min(0).round(2).tolist()} - '
          f'{additions.max(0).round(2).tolist()}, height p5/50/95 {heights}')
    print(f'written {args.output} (mesh {sha[:12]})')


if __name__ == '__main__':
    np.seterr(all='ignore')
    main()
