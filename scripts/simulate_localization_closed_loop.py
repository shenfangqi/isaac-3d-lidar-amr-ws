#!/usr/bin/env python3
"""
Offline closed loop on the real map from given true start poses.

Read-only; never publishes.  The world is the navigation map itself (the
sensor sees exactly the mapped walls), so the result shows what the 2D
pipeline can do on this site's geometry.  Decisions never read the truth.
"""

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
sys.path.insert(0, str(ROOT / 'scripts'))
from diagnose_localization_bag import load_map  # noqa: E402
from isaac_3d_lidar_bringup import localization_hypotheses as lh  # noqa: E402
from isaac_3d_lidar_bringup.localization_closed_loop import (  # noqa: E402
    ClosedLoopSimulator, SimulationConfig,
)
from isaac_3d_lidar_bringup.localization_contracts import SE2  # noqa: E402
from isaac_3d_lidar_bringup.localization_route_planner import (  # noqa: E402
    MotionModel, PlannerConfig, static_map_from_grid,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--start', action='append', required=True,
                        help='x,y,yaw_deg of a true start pose; repeat')
    parser.add_argument('--steps', type=int, default=4)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')
    grid = load_map(args.map)
    static_map = static_map_from_grid(grid, lh.map_hash(grid))
    simulator = ClosedLoopSimulator(
        static_map, lh.SearchConfig(), lh.ValidationThresholds(), MotionModel(),
        PlannerConfig(rotations=(math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2),
                      forwards=(0.2, 0.4, 0.6), max_depth=2, min_difference=0.15,
                      time_budget_s=300),
        SimulationConfig(max_steps=args.steps))
    runs = []
    for text in args.start:
        x, y, yaw = (float(v) for v in text.split(','))
        result = simulator.run(SE2(x, y, math.radians(yaw)))
        runs.append({'start': [x, y, yaw], 'outcome': result.outcome,
                     'reason': result.reason, 'travel_m': result.travel_m,
                     'history': result.history})
        print(f'start ({x:.2f},{y:.2f},{yaw:.0f}deg): {result.outcome} {result.reason} '
              f'travel {result.travel_m:.2f} m')
        keys = ('step', 'candidates', 'contending', 'verdict', 'measured_margin',
                'plan_status', 'predicted_margin', 'primitive', 'unsafe_reasons')
        for step in result.history:
            print('   ', {k: step.get(k) for k in keys})
    args.output.write_text(json.dumps({
        'schema': 1, 'offline_only': True, 'map': str(args.map),
        'map_hash': static_map.map_hash,
        'note': 'World = navigation map; translation margins are assumed values.',
        'runs': runs}, indent=2, default=str))
    return 0


if __name__ == '__main__':
    sys.exit(main())
