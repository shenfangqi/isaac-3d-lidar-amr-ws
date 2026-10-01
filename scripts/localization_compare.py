#!/usr/bin/env python3
"""Compare explicit base_footprint hypotheses using identical stationary scans.

This is an offline measurement tool, never a navigation acceptance authority.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.automatic_localization_quality import scan_map_metrics


def namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{key: namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [namespace(item) for item in value]
    return value


def compare(dataset, hypotheses, params):
    grid = namespace(dataset['map'])
    scans = [namespace(entry['scan']) for entry in dataset['scans']]
    if not scans:
        raise ValueError('no scans')
    for scan in scans:
        if scan.header.frame_id != 'base_footprint':
            raise ValueError('base_footprint scan required')
        scan.ranges = [math.nan if r is None else r for r in scan.ranges]
    results = []
    for pose in hypotheses:
        x, y, yaw = [float(pose[key]) for key in ('x', 'y', 'yaw')]
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('nonfinite hypothesis')
        transform = namespace({'translation': {'x': x, 'y': y}, 'rotation': {
            'x': 0., 'y': 0., 'z': math.sin(yaw / 2), 'w': math.cos(yaw / 2)}})
        frames = []
        for scan in scans:
            metrics = scan_map_metrics(grid, scan, transform,
                params['occupied_threshold'],
                math.ceil(params['scan_match_tolerance_m'] / grid.info.resolution),
                params['scan_score_max_beams'])
            metrics['scan_gates_pass'] = (
                metrics['score'] >= params['min_scan_map_score']
                and metrics['coverage'] >= params['min_scan_map_coverage']
                and metrics['known'] >= params['min_valid_scan_beams'])
            frames.append(metrics)
        results.append({'hypothesis': pose, 'frames': frames,
                        'pass_fraction': sum(m['scan_gates_pass'] for m in frames) / len(frames)})
    return {'navigation_accepted': False, 'results': results,
            'warning': 'Scan gates only; reference, timing, ambiguity and physical acceptance required'}


def main():
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('hypotheses', type=Path,
                        help='JSON list of {name,x,y,yaw}; metres/radians in map frame')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] /
        'src/isaac_3d_lidar_bringup/config/nav2/carbot_auto_localization_real.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    params = yaml.safe_load(args.config.read_text())['automatic_localization_manager']['ros__parameters']
    report = compare(json.loads(args.dataset.read_text()),
                     json.loads(args.hypotheses.read_text()), params)
    report['sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (args.dataset, args.hypotheses, args.config)}
    report['parameters'] = params
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(args.output)


if __name__ == '__main__':
    main()
