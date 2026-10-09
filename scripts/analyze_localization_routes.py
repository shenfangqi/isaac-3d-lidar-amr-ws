#!/usr/bin/env python3
"""
Phase 1 site analysis: commonly safe disambiguating routes per recorded case.

Read-only.  For each diagnosis report (``diagnose_localization_bag.py``), the
plausible candidates of one view are taken at that view's time.  Two
observation sets are planned separately: the 2D localization band only, and
every height layer rasterised from the 3D mesh.  Motion margins for
translation are ASSUMED (no linear calibration); the report says so.
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
sys.path.insert(0, str(ROOT / 'scripts'))
from diagnose_localization_bag import load_map  # noqa: E402
from isaac_3d_lidar_bringup import localization_hypotheses as lh  # noqa: E402
from isaac_3d_lidar_bringup.localization_route_planner import (  # noqa: E402
    ClearanceField, MotionModel, ObservationModel, plan_route, PlannerConfig,
    PoseBounds, static_map_from_grid,
)
from recheck_candidates_3d import load_ply_vertices  # noqa: E402

BANDS = {'scan_0.22-0.35': (0.22, 0.35), 'band_0.35-0.60': (0.35, 0.6),
         'band_0.60-1.00': (0.6, 1.0), 'band_1.00-1.50': (1.0, 1.5),
         'band_1.50-2.00': (1.5, 2.0)}


def mesh_layers(static_map, vertices):
    """
    ``(scene, reference)`` grids per height band, dilated by one cell.

    The 2D localization band is scored against the navigation map, like the
    production 2D matcher; 3D bands are scored against themselves, like the
    3D re-check.
    """
    from scipy.ndimage import binary_dilation
    layers = {}
    for name, (lo, hi) in BANDS.items():
        band = vertices[(vertices[:, 2] >= lo) & (vertices[:, 2] < hi)]
        i, j = static_map.cells(band[:, 0], band[:, 1])
        keep = (i >= 0) & (i < static_map.width) & (j >= 0) & (j < static_map.height)
        grid = np.zeros(static_map.data.shape, bool)
        grid[j[keep], i[keep]] = True
        scene = binary_dilation(grid, iterations=1)
        reference = static_map.data == 100 if name.startswith('scan') else scene
        layers[name] = (scene, reference)
    return layers


def plausible(view, window, limit):
    """Candidates whose holdout score is within ``window`` of the best."""
    rows = sorted((c for c in view['candidates'] if c.get('holdout')),
                  key=lambda c: -c['holdout']['score'])
    best = rows[0]['holdout']['score']
    return [c for c in rows if c['holdout']['score'] >= best - window][:limit]


def describe(route):
    return ' -> '.join(f'{k} {math.degrees(v):+.0f}deg' if k == 'ROTATE'
                       else f'{k} {v:.2f}m' for k, v in route) or '(stay)'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', nargs='+', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--view', type=int, default=0)
    parser.add_argument('--window', type=float, default=0.25)
    parser.add_argument('--limit', type=int, default=6)
    parser.add_argument('--min-difference', type=float, default=0.25)
    parser.add_argument('--depth', type=int, default=3)
    parser.add_argument('--budget', type=float, default=120.0)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')
    grid = load_map(args.map)
    static_map = static_map_from_grid(grid, lh.map_hash(grid))
    fields = (ClearanceField(static_map),
              ClearanceField(static_map, blocked=static_map.data == 100,
                             outside_blocked=False))
    layers = mesh_layers(static_map, load_ply_vertices(args.mesh))
    observation = ObservationModel(static_map, layers)
    model, bounds = MotionModel(), PoseBounds()
    rows = []
    for path in args.reports:
        report = json.loads(path.read_text())
        view = next(v for v in report['views'] if v['view_id'] == args.view)
        chosen = plausible(view, args.window, args.limit)
        best = chosen[0]['holdout']['score'] if chosen else 0.0
        contend = report['thresholds']['min_margin'] + 0.05
        focus = tuple(i for i, c in enumerate(chosen)
                      if c['holdout']['score'] >= best - contend)
        poses = [(c['pose_at_view']['x'], c['pose_at_view']['y'],
                  c['pose_at_view']['yaw']) for c in chosen]
        entry = {'report': str(path), 'view': args.view, 'contenders': len(focus),
                 'measured_2d_margin': view['score_margin'],
                 'candidates': [{'pose_at_view': c['pose_at_view'],
                                 'holdout_2d': c['holdout']['score']} for c in chosen]}
        recheck = path.with_name(path.name.replace('_diagnosis_v2.json', '_recheck3d.json'))
        if recheck.exists():
            entry['measured_3d_gap'] = json.loads(recheck.read_text())[
                'comparison']['composite_gap']
        if len(poses) < 2:
            entry['status'] = 'SINGLE_PLAUSIBLE_CANDIDATE'
            rows.append(entry)
            continue
        for label, names in (('2d_scan_band', ('scan_0.22-0.35',)),
                             ('all_layers', tuple(layers))):
            config = PlannerConfig(max_depth=args.depth, min_difference=args.min_difference,
                                   time_budget_s=args.budget, layers=names)
            result = plan_route(fields, model, observation,
                                [(p, bounds) for p in poses], config,
                                focus if len(focus) >= 2 else None)
            entry[label] = {
                'status': result.status, 'route': describe(result.route),
                'route_primitives': [list(p) for p in result.route],
                'objective': round(result.objective, 3),
                'initial_objective': round(result.initial_objective, 3),
                'pair_differences': {f'{a}-{b}': round(v, 3)
                                     for (a, b), v in result.pair_differences.items()},
                'evaluated_routes': result.evaluated,
                'unsafe_reasons': result.unsafe_reasons,
            }
        rows.append(entry)
        name = Path(report['bag']).name.replace('2026-10-', '')
        measured = (f"measured 2D margin {entry['measured_2d_margin']:.3f}"
                    + (f", 3D gap {entry['measured_3d_gap']:.3f}"
                       if 'measured_3d_gap' in entry else ''))
        print(f'{name}: {len(poses)} candidates ({len(focus)} contending); {measured}')
        for label in ('2d_scan_band', 'all_layers'):
            e = entry[label]
            print(f'   {label:13s} {e["status"]:27s} now {e["initial_objective"]:.2f} '
                  f'-> {e["objective"]:.2f}  route {e["route"]}  '
                  f'({e["evaluated_routes"]} routes, unsafe {e["unsafe_reasons"]})')
    output = {
        'schema': 1, 'navigation_accepted': False, 'offline_only': True,
        'map': str(args.map), 'map_hash': static_map.map_hash, 'mesh': str(args.mesh),
        'motion_model': {k: (list(v) if isinstance(v, tuple) else v)
                         for k, v in MotionModel().__dict__.items()},
        'motion_model_note': 'Translation margins are ASSUMED; no linear calibration exists.',
        'pose_bounds': PoseBounds().__dict__,
        'candidate_window': args.window, 'min_difference': args.min_difference,
        'limitations': [
            'Mesh vertices rasterised per height band; 2D ray casts per band '
            'approximate the 3D sensor (no vertical field-of-view model).',
            'Predicted differences only choose routes; they never accept a pose.',
            'Static-map safety assumes the site matches the map (user premise).',
        ],
        'cases': rows,
    }
    args.output.write_text(json.dumps(output, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
