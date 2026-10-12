#!/usr/bin/env python3
"""
Draw an analyze_localizability_3d.py result over the navigation map.

Each analysed position is a dot: green if every heading was accepted at
the true pose, orange if some were, red if none; a black cross marks any
wrong acceptance.  Optional labelled poses (ground-truth JSON files) are
drawn as arrows.  Read-only.
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from diagnose_localization_bag import load_map  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('analysis', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--truth', type=Path, nargs='*', default=[])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    report = json.loads(args.analysis.read_text())
    grid = load_map(args.map)
    info = grid.info
    cells = np.asarray(grid.data, np.int8).reshape(info.height, info.width)
    image = np.full(cells.shape, .85)
    image[cells == 0] = 1.
    image[cells >= 65] = 0.
    x0, y0 = info.origin.position.x, info.origin.position.y
    extent = (x0, x0 + info.width * info.resolution, y0, y0 + info.height * info.resolution)
    figure, axes = plt.subplots(figsize=(8, 12))
    axes.imshow(image, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=1)
    for place in report['places']:
        accepted = sum(v == 'ACCEPT' for v in place['verdicts'])
        color = ('tab:green' if accepted == len(place['verdicts'])
                 else 'tab:orange' if accepted else 'tab:red')
        axes.plot(place['x'], place['y'], 'o', color=color, markersize=5)
        if 'WRONG_LEADER' in place['verdicts']:
            axes.plot(place['x'], place['y'], 'kx', markersize=9, mew=2)
    for path in args.truth:
        truth = json.loads(path.read_text())['true_candidate']
        yaw = math.radians(truth['yaw_deg'])
        axes.arrow(truth['x'], truth['y'], .35 * math.cos(yaw), .35 * math.sin(yaw),
                   width=.04, color='tab:blue')
    summary = report['summary']
    axes.set_title(f"3D localizability ({'degraded' if report.get('clutter') else 'map-perfect'}"
                   f" scans): {summary['places_all_accepted']}/{summary['places']} places all "
                   f"headings accepted, wrong {summary['wrong_leaders']}", fontsize=9)
    axes.set_xlabel('map x (m)')
    axes.set_ylabel('map y (m)')
    figure.tight_layout()
    figure.savefig(args.output, dpi=120)


if __name__ == '__main__':
    main()
