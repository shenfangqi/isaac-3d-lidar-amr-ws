#!/usr/bin/env python3
"""Desktop-only geometry timing baseline for Issue #12.

This does not represent Jetson or physical acceptance.  It deliberately
benchmarks the read-only swept-path risk calculation without a braking profile.
"""

import argparse
import json
import math
import platform
from statistics import median
import time

from carbot_nav_recovery.speed_advisor import evaluate_path_risk
from carbot_nav_recovery.swept_footprint import CostmapSnapshot, Pose2D


FOOTPRINT = ((0.155, 0.133), (0.155, -0.133),
             (-0.130, -0.133), (-0.130, 0.133))


def percentile(values, fraction):
    """Return a nearest-rank percentile for a nonempty sequence."""
    ordered = sorted(values)
    index = min(len(ordered) - 1,
                max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def scenario():
    """Construct a deterministic curved path and synthetic local costmap."""
    width = height = 80
    resolution = 0.05
    costs = [0] * (width * height)
    for y_index in range(32, 49):
        costs[y_index * width + 58] = 254
    snapshot = CostmapSnapshot(
        width, height, resolution, -1.0, -2.0, tuple(costs),
        'map', 10.0, 20.0)
    path = tuple(Pose2D(
        0.05 * index,
        0.28 * math.sin(index / 10.0),
        math.atan2(0.28 * math.cos(index / 10.0) / 10.0, 0.05))
        for index in range(31))
    return snapshot, path


def main(args=None):
    """Run and print the desktop-only timing baseline."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--iterations', type=int, default=200)
    parsed = parser.parse_args(args)
    if parsed.iterations < 1:
        parser.error('--iterations must be positive')
    snapshot, path = scenario()
    durations = []
    result = None
    for _ in range(parsed.iterations):
        started = time.perf_counter()
        result = evaluate_path_risk(snapshot, FOOTPRINT, path)
        durations.append((time.perf_counter() - started) * 1000.0)
    print(json.dumps({
        'schema': 'carbot_complex_route_desktop_benchmark_v1',
        'physical_acceptance': False,
        'platform': f'python_{platform.machine()}',
        'iterations': parsed.iterations,
        'median_ms': median(durations),
        'p95_ms': percentile(durations, 0.95),
        'p99_ms': percentile(durations, 0.99),
        'result': {
            'distance_to_unsafe_m': result.distance_to_unsafe_m,
            'unsafe_reason': result.unsafe_reason,
            'maximum_curvature_inv_m': result.maximum_curvature_inv_m,
        },
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
