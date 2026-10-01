#!/usr/bin/env python3
"""Search a saved map for spatially distinct scan-matching hypotheses."""

import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.automatic_localization_quality import (  # noqa: E402
    angular_difference,
    scan_map_metrics,
)


def namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{
            key: namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [namespace(item) for item in value]
    return value


def transform(x, y, yaw):
    return namespace({
        'translation': {'x': x, 'y': y},
        'rotation': {
            'x': 0.0, 'y': 0.0,
            'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0),
        },
    })


def selected_frames(scans, count):
    if count >= len(scans):
        return scans
    if count == 1:
        return [scans[len(scans) // 2]]
    return [
        scans[round(index * (len(scans) - 1) / (count - 1))]
        for index in range(count)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--reference', nargs=3, type=float, required=True,
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--config', type=Path, default=(
        Path(__file__).resolve().parents[1] /
        'src/isaac_3d_lidar_bringup/config/nav2/'
        'carbot_auto_localization_real.yaml'))
    parser.add_argument('--position-step', type=float, default=0.40)
    parser.add_argument('--yaw-step-deg', type=float, default=30.0)
    parser.add_argument('--coarse-beams', type=int, default=30)
    parser.add_argument('--coarse-frames', type=int, default=1)
    parser.add_argument('--refine-count', type=int, default=40)
    parser.add_argument('--refine-frames', type=int, default=12)
    parser.add_argument('--exclude-radius', type=float, default=0.50)
    parser.add_argument('--search-radius', type=float,
                        help='optionally limit positions to this reference radius')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.position_step <= 0 or args.yaw_step_deg <= 0:
        parser.error('search steps must be positive')

    dataset = json.loads(args.dataset.read_text())
    params = yaml.safe_load(args.config.read_text())[
        'automatic_localization_manager']['ros__parameters']
    grid = namespace(dataset['map'])
    scans = [namespace(entry['scan']) for entry in dataset['scans']]
    for scan in scans:
        if scan.header.frame_id != 'base_footprint':
            raise SystemExit('base_footprint scan required')
        scan.ranges = [math.nan if value is None else value
                       for value in scan.ranges]

    tolerance = math.ceil(
        params['scan_match_tolerance_m'] / grid.info.resolution)
    cell_step = max(1, round(args.position_step / grid.info.resolution))
    yaw_count = max(1, math.ceil(360.0 / args.yaw_step_deg))
    coarse_scans = selected_frames(scans, args.coarse_frames)
    ref_x, ref_y, ref_yaw = args.reference
    candidates = []
    origin = grid.info.origin.position

    for cell_y in range(0, grid.info.height, cell_step):
        for cell_x in range(0, grid.info.width, cell_step):
            if grid.data[cell_y * grid.info.width + cell_x] != 0:
                continue
            x = origin.x + (cell_x + 0.5) * grid.info.resolution
            y = origin.y + (cell_y + 0.5) * grid.info.resolution
            reference_distance = math.hypot(x - ref_x, y - ref_y)
            if reference_distance < args.exclude_radius:
                continue
            if (args.search_radius is not None
                    and reference_distance > args.search_radius):
                continue
            for yaw_index in range(yaw_count):
                yaw = -math.pi + yaw_index * 2.0 * math.pi / yaw_count
                metrics = [
                    scan_map_metrics(
                        grid, scan, transform(x, y, yaw),
                        params['occupied_threshold'], tolerance,
                        args.coarse_beams,
                    )
                    for scan in coarse_scans
                ]
                candidates.append({
                    'x': x, 'y': y, 'yaw': yaw,
                    'coarse_score': sum(m['score'] for m in metrics)
                                    / len(metrics),
                })

    candidates.sort(key=lambda row: row['coarse_score'], reverse=True)
    refine_scans = selected_frames(scans, args.refine_frames)
    refined = []
    for candidate in candidates[:args.refine_count]:
        metrics = [
            scan_map_metrics(
                grid, scan,
                transform(candidate['x'], candidate['y'], candidate['yaw']),
                params['occupied_threshold'], tolerance,
                params['scan_score_max_beams'],
            )
            for scan in refine_scans
        ]
        passes = [
            metric['score'] >= params['min_scan_map_score']
            and metric['coverage'] >= params['min_scan_map_coverage']
            and metric['known'] >= params['min_valid_scan_beams']
            for metric in metrics
        ]
        refined.append({
            **candidate,
            'distance_from_reference_m': math.hypot(
                candidate['x'] - ref_x, candidate['y'] - ref_y),
            'yaw_error_from_reference_rad': abs(angular_difference(
                candidate['yaw'], ref_yaw)),
            'mean_score': sum(m['score'] for m in metrics) / len(metrics),
            'mean_coverage': sum(m['coverage'] for m in metrics) / len(metrics),
            'mean_wall_conflict_ratio': sum(
                m['wall_conflict_ratio'] for m in metrics) / len(metrics),
            'pass_fraction': sum(passes) / len(passes),
        })
    refined.sort(key=lambda row: row['mean_score'], reverse=True)
    report = {
        'navigation_accepted': False,
        'reference': {'x': ref_x, 'y': ref_y, 'yaw': ref_yaw},
        'search': {
            'position_step_m': args.position_step,
            'yaw_step_deg': args.yaw_step_deg,
            'excluded_reference_radius_m': args.exclude_radius,
            'search_reference_radius_m': args.search_radius,
            'coarse_candidate_count': len(candidates),
            'coarse_beams': args.coarse_beams,
            'coarse_frames': len(coarse_scans),
            'refined_candidate_count': len(refined),
            'refine_frames': len(refine_scans),
        },
        'candidates': refined,
        'warning': (
            'Coarse diagnostic search; absence of a candidate is not proof '
            'that the full continuous map has no ambiguity.'),
    }
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(args.output)


if __name__ == '__main__':
    main()
