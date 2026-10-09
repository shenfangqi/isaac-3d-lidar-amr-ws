#!/usr/bin/env python3
"""Plot competing poses and margins from read-only localization diagnosis."""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import numpy as np

from diagnose_localization_bag import load_map


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reports', nargs='+', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; choose another path')
    grid = load_map(args.map)
    reports = [json.loads(p.read_text()) for p in args.reports]
    figure, axes = plt.subplots(2, len(reports), figsize=(5 * len(reports), 10), squeeze=False)
    image = np.array(grid.data).reshape(grid.info.height, grid.info.width)
    display = np.where(image < 0, .65, np.where(image >= 65, .05, 1.0))
    o = grid.info.origin
    yaw = 2 * math.atan2(o.orientation.z, o.orientation.w)
    transform = Affine2D().rotate(yaw).translate(o.position.x, o.position.y)
    width, height = grid.info.width * grid.info.resolution, grid.info.height * grid.info.resolution
    corners = transform.transform([[0, 0], [width, 0], [width, height], [0, height]])
    for column, report in enumerate(reports):
        ax, chart = axes[:, column]
        ax.imshow(display, cmap='gray', origin='lower', vmin=0, vmax=1,
                  extent=(0, width, 0, height), transform=transform + ax.transData)
        ax.set_xlim(corners[:, 0].min(), corners[:, 0].max())
        ax.set_ylim(corners[:, 1].min(), corners[:, 1].max())
        final = report['views'][-1]
        for index, candidate in enumerate(final['candidates'][:2]):
            pose = candidate['pose_at_reference']
            color = ('#cf4a32', '#2166ac')[index]
            ax.plot(pose['x'], pose['y'], 'o', color=color, markersize=9,
                    label=f"Rank {index + 1}: {candidate['holdout']['score']:.3f}")
            ax.arrow(pose['x'], pose['y'], .45 * math.cos(pose['yaw']),
                     .45 * math.sin(pose['yaw']), width=.025, color=color)
            ax.annotate(f"({pose['x']:.2f}, {pose['y']:.2f})", (pose['x'], pose['y']),
                        xytext=(8, 8), textcoords='offset points', fontsize=9)
        title = Path(report['bag']).name.replace('2026-10-09_issue13_', '')
        ax.set_title(title + '\nFinal replay candidates (reference frame)')
        ax.set_xlabel('Map x (m)')
        ax.set_ylabel('Map y (m)')
        ax.legend(loc='upper left')
        ax.set_aspect('equal')
        views = report['views']
        chart.plot([v['view_id'] for v in views], [v['score_margin'] for v in views],
                   'o-', color='#2166ac', label='Reconstructed holdout margin')
        chart.axhline(report['thresholds']['min_margin'], color='#cf4a32',
                      linestyle='--', label='Existing acceptance margin')
        chart.set_ylim(0, .15)
        chart.set_xticks([v['view_id'] for v in views])
        chart.set_xlabel('View (0 = before first probe)')
        chart.set_ylabel('First minus second candidate score')
        chart.grid(alpha=.25)
        chart.legend(fontsize=8)
    figure.suptitle('Offline diagnostic reconstruction; no ground-truth labels or navigation acceptance', fontsize=13)
    figure.tight_layout(rect=(0, 0, 1, .96))
    figure.savefig(args.output, dpi=150)
    print(args.output)


if __name__ == '__main__':
    main()
