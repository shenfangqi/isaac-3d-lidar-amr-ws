#!/usr/bin/env python3
"""Refine a planar pose against a saved map using continuous endpoint distance.

This is an offline diagnostic.  It never publishes a pose or authorizes
navigation.  A distance field breaks the broad plateaus created by the live
binary endpoint tolerance; the production free-ray metric is then evaluated on
the best distance candidates so wall-crossing solutions remain visible.
"""

import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.automatic_localization_quality import (  # noqa: E402
    scan_map_metrics,
)


def namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{
            key: namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [namespace(item) for item in value]
    return value


def selected_frames(scans, count):
    if count >= len(scans):
        return scans
    if count == 1:
        return [scans[len(scans) // 2]]
    return [
        scans[round(index * (len(scans) - 1) / (count - 1))]
        for index in range(count)
    ]


def transform(x, y, yaw):
    return namespace({
        'translation': {'x': x, 'y': y},
        'rotation': {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0),
        },
    })


def scan_points(scan):
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    valid = (np.isfinite(ranges)
             & (ranges >= scan.range_min)
             & (ranges <= scan.range_max))
    return np.column_stack((ranges[valid] * np.cos(angles[valid]),
                            ranges[valid] * np.sin(angles[valid])))


def distance_candidates(grid, scans, center, position_radius, position_step,
                        yaw_radius, yaw_step, occupied_threshold,
                        distance_cap, retain):
    """Rank local poses by robust continuous endpoint-to-wall distance."""
    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    cells = np.asarray(grid.data, dtype=np.int16).reshape(height, width)
    occupied = cells >= occupied_threshold
    distances = distance_transform_edt(~occupied) * resolution
    origin = grid.info.origin
    origin_yaw = 2.0 * math.atan2(
        origin.orientation.z, origin.orientation.w)
    cos_origin = math.cos(-origin_yaw)
    sin_origin = math.sin(-origin_yaw)
    points = [scan_points(scan) for scan in scans]
    points = [value for value in points if value.size]
    if not points:
        return [], []
    local = np.concatenate(points, axis=0)

    offsets = np.arange(-position_radius,
                        position_radius + position_step * 0.5,
                        position_step)
    x_values = center[0] + offsets
    y_values = center[1] + offsets
    mesh_x, mesh_y = np.meshgrid(x_values, y_values, indexing='xy')
    translations = np.column_stack((mesh_x.ravel(), mesh_y.ravel()))
    yaw_values = np.arange(center[2] - yaw_radius,
                           center[2] + yaw_radius + yaw_step * 0.5,
                           yaw_step)
    ranked = []
    yaw_profile = []
    for yaw in yaw_values:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rotated_x = cos_yaw * local[:, 0] - sin_yaw * local[:, 1]
        rotated_y = sin_yaw * local[:, 0] + cos_yaw * local[:, 1]
        best_for_yaw = None
        chunk_size = 128
        for start in range(0, len(translations), chunk_size):
            batch = translations[start:start + chunk_size]
            map_x = batch[:, 0, None] + rotated_x[None, :]
            map_y = batch[:, 1, None] + rotated_y[None, :]
            shifted_x = map_x - origin.position.x
            shifted_y = map_y - origin.position.y
            grid_x = np.floor(
                (cos_origin * shifted_x - sin_origin * shifted_y)
                / resolution).astype(np.int32)
            grid_y = np.floor(
                (sin_origin * shifted_x + cos_origin * shifted_y)
                / resolution).astype(np.int32)
            inside = ((grid_x >= 0) & (grid_x < width)
                      & (grid_y >= 0) & (grid_y < height))
            clipped_x = np.clip(grid_x, 0, width - 1)
            clipped_y = np.clip(grid_y, 0, height - 1)
            endpoint_distance = distances[clipped_y, clipped_x]
            known = cells[clipped_y, clipped_x] >= 0
            usable = inside & known
            robust_distance = np.where(
                usable, np.minimum(endpoint_distance, distance_cap),
                distance_cap)
            mean_distance = robust_distance.mean(axis=1)
            coverage = usable.mean(axis=1)
            for index, candidate in enumerate(batch):
                row = {
                    'x': float(candidate[0]),
                    'y': float(candidate[1]),
                    'yaw': float(yaw),
                    'mean_capped_endpoint_distance_m': float(
                        mean_distance[index]),
                    'endpoint_coverage': float(coverage[index]),
                }
                ranked.append(row)
                if (best_for_yaw is None
                        or row['mean_capped_endpoint_distance_m']
                        < best_for_yaw['mean_capped_endpoint_distance_m']):
                    best_for_yaw = row
        yaw_profile.append(best_for_yaw)
    ranked.sort(key=lambda row: (
        row['mean_capped_endpoint_distance_m'], -row['endpoint_coverage']))
    return ranked[:retain], yaw_profile


def add_free_ray_metrics(candidates, grid, scans, params):
    tolerance = math.ceil(
        params['scan_match_tolerance_m'] / grid.info.resolution)
    for candidate in candidates:
        metrics = [
            scan_map_metrics(
                grid, scan,
                transform(candidate['x'], candidate['y'], candidate['yaw']),
                params['occupied_threshold'], tolerance,
                max(len(scan.ranges), params['scan_score_max_beams']),
            )
            for scan in scans
        ]
        for name in ('score', 'coverage', 'wall_conflict_ratio',
                     'legacy_score'):
            candidate['mean_' + name] = sum(
                metric[name] for metric in metrics) / len(metrics)
        candidate['minimum_known_beams'] = min(
            metric['known'] for metric in metrics)
        # Continuous endpoint distance resolves sub-tolerance geometry; free
        # rays break ties in favour of physically possible poses.
        candidate['combined_cost'] = (
            candidate['mean_capped_endpoint_distance_m']
            + 0.25 * candidate['mean_wall_conflict_ratio']
            + 0.10 * (1.0 - candidate['mean_coverage'])
        )
    candidates.sort(key=lambda row: row['combined_cost'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--center', nargs=3, type=float, required=True,
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--position-radius', type=float, default=0.20)
    parser.add_argument('--position-step', type=float, default=0.01)
    parser.add_argument('--yaw-radius-deg', type=float, default=8.0)
    parser.add_argument('--yaw-step-deg', type=float, default=0.5)
    parser.add_argument('--frames', type=int, default=9)
    parser.add_argument('--retain', type=int, default=60)
    parser.add_argument('--distance-cap', type=float, default=0.30)
    parser.add_argument('--config', type=Path, default=(
        Path(__file__).resolve().parents[1] /
        'src/isaac_3d_lidar_bringup/config/nav2/'
        'carbot_auto_localization_real.yaml'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if min(args.position_radius, args.position_step, args.yaw_radius_deg,
           args.yaw_step_deg, args.frames, args.retain,
           args.distance_cap) <= 0:
        parser.error('search dimensions, counts, and cap must be positive')

    dataset = json.loads(args.dataset.read_text())
    grid = namespace(dataset['map'])
    scans = [namespace(entry['scan']) for entry in dataset['scans']]
    for scan in scans:
        if scan.header.frame_id != 'base_footprint':
            raise SystemExit('base_footprint scan required')
        scan.ranges = [math.nan if value is None else value
                       for value in scan.ranges]
    scans = selected_frames(scans, args.frames)
    params = yaml.safe_load(args.config.read_text())[
        'automatic_localization_manager']['ros__parameters']
    center = tuple(args.center)
    candidates, yaw_profile = distance_candidates(
        grid, scans, center, args.position_radius, args.position_step,
        math.radians(args.yaw_radius_deg), math.radians(args.yaw_step_deg),
        params['occupied_threshold'], args.distance_cap, args.retain)
    add_free_ray_metrics(candidates, grid, scans, params)
    report = {
        'navigation_accepted': False,
        'dataset': str(args.dataset.resolve()),
        'center': {'x': center[0], 'y': center[1], 'yaw': center[2]},
        'search': {
            'position_radius_m': args.position_radius,
            'position_step_m': args.position_step,
            'yaw_radius_deg': args.yaw_radius_deg,
            'yaw_step_deg': args.yaw_step_deg,
            'frames': len(scans),
            'finite_beams_per_frame': [len(scan_points(scan)) for scan in scans],
            'distance_cap_m': args.distance_cap,
            'retained_for_free_ray_evaluation': args.retain,
        },
        'best': candidates[0] if candidates else None,
        'candidates': candidates,
        'yaw_profile': yaw_profile,
        'warning': (
            'Offline geometric diagnostic only; physical reference, temporal '
            'stability, ambiguity, and independent validation are still required.'
        ),
    }
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(args.output)


if __name__ == '__main__':
    main()
