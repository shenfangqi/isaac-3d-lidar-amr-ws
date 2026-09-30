#!/usr/bin/env python3
"""Benchmark the production deterministic global pose search offline."""

import argparse
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.automatic_localization_quality import (  # noqa: E402
    deterministic_global_search,
)


def namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{
            key: namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [namespace(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    dataset = json.loads(args.dataset.read_text())
    grid = namespace(dataset['map'])
    all_scans = [namespace(item['scan']) for item in dataset['scans']]
    for scan in all_scans:
        scan.ranges = [math.nan if item is None else item
                       for item in scan.ranges]
    indices = (0, len(all_scans) // 2, len(all_scans) - 1)
    scans = [all_scans[index] for index in indices]
    params = yaml.safe_load(args.config.read_text())[
        'automatic_localization_manager']['ros__parameters']
    started = time.monotonic()
    candidates = deterministic_global_search(
        grid, scans, params['occupied_threshold'],
        math.ceil(params['scan_match_tolerance_m'] / grid.info.resolution),
        params['global_search_position_step_m'],
        params['global_search_yaw_step_rad'],
        params['global_search_coarse_beams'],
        params['scan_score_max_beams'],
        params['global_search_refine_count'],
        params['global_search_fine_position_step_m'],
        params['global_search_fine_yaw_step_rad'],
        params['global_search_fine_position_radius_m'],
        params['global_search_fine_yaw_radius_rad'],
        params['global_search_fine_seed_count'],
        params['global_search_final_position_step_m'],
        params['global_search_final_yaw_step_rad'],
        params['global_search_final_position_radius_m'],
        params['global_search_final_yaw_radius_rad'],
    )
    best = candidates[0]
    separation = params['global_search_ambiguity_distance_m']
    runner_up = next((candidate for candidate in candidates[1:] if math.hypot(
        candidate['x'] - best['x'], candidate['y'] - best['y'])
        >= separation), None)
    print(json.dumps({
        'elapsed_sec': time.monotonic() - started,
        'best': best,
        'distant_runner_up': runner_up,
        'score_margin': (
            None if runner_up is None else best['score'] - runner_up['score']),
        'candidate_count': len(candidates),
    }, indent=2))


if __name__ == '__main__':
    main()
