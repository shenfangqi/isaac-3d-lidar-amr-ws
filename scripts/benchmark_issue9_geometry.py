#!/usr/bin/env python3
"""Benchmark Issue 9 swept-footprint checks on the target CPU."""

import argparse
import json
import math
import platform
import statistics
import time

from carbot_nav_recovery.swept_footprint import (
    CostmapSnapshot,
    Pose2D,
    check_swept_path,
)


FOOTPRINT = (
    (0.165, 0.143),
    (0.165, -0.143),
    (-0.140, -0.143),
    (-0.140, 0.143),
)


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def measure(operation, iterations, warmup):
    for _ in range(warmup):
        operation()
    samples = []
    for _ in range(iterations):
        started = time.perf_counter_ns()
        operation()
        samples.append((time.perf_counter_ns() - started) / 1e6)
    return {
        'iterations': iterations,
        'mean_ms': statistics.fmean(samples),
        'p50_ms': percentile(samples, 0.50),
        'p95_ms': percentile(samples, 0.95),
        'p99_ms': percentile(samples, 0.99),
        'max_ms': max(samples),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iterations', type=int, default=500)
    parser.add_argument('--warmup', type=int, default=25)
    args = parser.parse_args()
    if not 50 <= args.iterations <= 10000 or not 0 <= args.warmup <= 1000:
        parser.error('iterations must be 50..10000 and warmup 0..1000')

    size = 80
    grid = CostmapSnapshot(
        size, size, 0.05, -2.0, -2.0, (0,) * (size * size),
        'map', 10.0, 20.0)
    rotations = tuple(
        (Pose2D(0.0, 0.0, 0.0),
         Pose2D(0.0, 0.0, math.radians(angle)),
         Pose2D(0.0, 0.0, math.radians(angle) + math.copysign(0.08, angle)))
        for angle in (15, -15, 30, -30, 60, -60, 90, -90)
    )
    translation = (
        Pose2D(0.0, 0.0, 0.0),
        Pose2D(-0.10, 0.0, 0.0),
        Pose2D(-0.13, 0.0, 0.0),
    )

    worst_rotation = rotations[-1]

    def one_rotation_two_maps():
        for _ in range(2):
            result = check_swept_path(grid, FOOTPRINT, worst_rotation, 0.02)
            if not result.safe:
                raise RuntimeError(result.reason)

    def rotation_batch_two_maps():
        for path in rotations:
            for _ in range(2):
                result = check_swept_path(grid, FOOTPRINT, path, 0.02)
                if not result.safe:
                    raise RuntimeError(result.reason)

    def translation_two_maps():
        for _ in range(2):
            result = check_swept_path(grid, FOOTPRINT, translation, 0.02)
            if not result.safe:
                raise RuntimeError(result.reason)

    def refuge_rotation_one_direction_two_maps():
        target = translation[-2]
        path = (target, Pose2D(target.x, target.y, target.yaw + math.pi))
        for _ in range(2):
            result = check_swept_path(grid, FOOTPRINT, path, 0.02)
            if not result.safe:
                raise RuntimeError(result.reason)

    report = {
        'schema': 'carbot_issue9_geometry_benchmark_v1',
        'platform': platform.platform(),
        'python': platform.python_version(),
        'grid': {'width': size, 'height': size, 'resolution_m': 0.05},
        'footprint_padding_included': True,
        'position_margin_m': 0.02,
        'control_compute_budget_ms': 100.0,
        'benchmarks': {
            'one_worst_rotation_two_maps': measure(
                one_rotation_two_maps, args.iterations, args.warmup),
            'eight_rotations_two_maps': measure(
                rotation_batch_two_maps, args.iterations, args.warmup),
            'translation_with_braking_two_maps': measure(
                translation_two_maps, args.iterations, args.warmup),
            'one_refuge_rotation_two_maps': measure(
                refuge_rotation_one_direction_two_maps,
                args.iterations, args.warmup),
        },
    }
    report['control_cycle_benchmarks'] = [
        'one_worst_rotation_two_maps',
        'translation_with_braking_two_maps',
        'one_refuge_rotation_two_maps',
    ]
    report['eight_rotation_batch_is_diagnostic_only'] = True
    report['within_compute_budget'] = all(
        report['benchmarks'][name]['p99_ms']
        < report['control_compute_budget_ms']
        for name in report['control_cycle_benchmarks'])
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
