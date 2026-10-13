#!/usr/bin/env python3
"""
What would a wrong pose do to the map overlay (#54)?  (offline, adversarial)

For every labelled sample, places the whole-bag submap at WRONG poses (2D
candidates farther than --exclude-m from the truth, or the truth shifted by
--shift if there is none) and feeds it to ``update_overlay`` twice with the
reliability gate bypassed, as if two sessions had been wrongly accepted:
once at the same wrong pose, once at two different ones.  Reports what the
update limits commit and whether ``self_check`` on the session's own cloud
would keep it.  The gate normally stops this first; this checks the limits
behind it.  No ROS node is started.
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from check_submap_changes import read_frames  # noqa: E402
from evaluate_surface_decisions import load_sample  # noqa: E402
from isaac_3d_lidar_bringup.localization_map_overlay import (  # noqa: E402
    observe, OverlayConfig, OverlayState, self_check, SessionQuality, update_overlay)
from isaac_3d_lidar_bringup.localization_submap import build_submap  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_additions import (  # noqa: E402
    load_surface_additions)
from isaac_3d_lidar_bringup.localization_surface_check import (  # noqa: E402
    planar, surface_tree)
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    _smooth_fit, refine_surface_pose, SurfaceRefineConfig)
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions)

FORCED = SessionQuality(True, 1., 0., 0.)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--variable-regions', type=Path, required=True)
    parser.add_argument('--base-additions', type=Path)
    parser.add_argument('--exclude-m', type=float, default=1.2)
    parser.add_argument('--shift', type=float, nargs=3, default=(.6, 0., 10.),
                        metavar=('DX', 'DY', 'DYAW_DEG'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error(f'{args.output} exists; remove it first')
    model = load_surface_model(args.mesh, .02)
    regions = load_variable_regions(args.variable_regions, model.mesh_sha256)
    base = (load_surface_additions(args.base_additions, model.mesh_sha256)
            if args.base_additions else np.zeros((0, 3)))
    tree = surface_tree(model.points, regions, base if len(base) else None)
    config = OverlayConfig()
    rows = []
    for entry in json.loads(args.samples.read_text()):
        sample = load_sample(ROOT / entry['diagnosis'])
        truth_doc = json.loads((ROOT / entry['truth']).read_text())['true_candidate']
        truth = (truth_doc['x'], truth_doc['y'], math.radians(truth_doc['yaw_deg']))
        far = [tuple(p) for p in sample['poses']
               if math.hypot(p[0] - truth[0], p[1] - truth[1]) > args.exclude_m]
        if not far:
            far = [(truth[0] + args.shift[0], truth[1] + args.shift[1],
                    truth[2] + math.radians(args.shift[2]))]
        frames, _ = read_frames(ROOT / sample['report']['bag'])
        reference = sample['train'][0].T_odom_base
        submap = build_submap(frames, planar(reference.x, reference.y, reference.yaw))
        # Two wrongly accepted sessions: the same wrong pose twice, and (if
        # there is one) two different wrong poses.
        for label, wrongs in (('same', far[:1] * 2), ('different', far[:2])):
            if len(wrongs) < 2:
                continue
            state = OverlayState(model.mesh_sha256)
            reports, shares = [], []
            for wrong in wrongs:
                evidence = observe(state, submap, wrong, tree, regions, config)
                shares.append(round(evidence.added_share, 3))
                new, report = update_overlay(state, FORCED, evidence, config)
                reports.append(report)
                state = new
            row = dict(name=entry['name'], case=label,
                       wrong=[[round(v, 3) for v in w] for w in wrongs], added_share=shares,
                       reasons=[r.reason for r in reports],
                       committed_cells=int(state.committed.sum()),
                       refused=[list(r.refused_components) for r in reports])
            if state.committed.any():
                cloud = np.vstack([sample['first'], sample['second']])
                new_tree = surface_tree(model.points, regions,
                                        np.vstack([base, state.surface()]))
                local = SurfaceRefineConfig(max_shift_m=.15, max_turn_rad=math.radians(5))
                sampled = cloud[np.linspace(0, len(cloud) - 1,
                                            min(len(cloud), 3000)).astype(int)]

                def fit(candidate, use_new):
                    chosen = new_tree if use_new else tree
                    refined, _ = refine_surface_pose(chosen, cloud, candidate, local,
                                                     lambda: False)
                    return _smooth_fit(chosen, np.c_[sampled, np.ones(len(sampled))],
                                       .05)(np.asarray(refined))
                # The wrong pose leads this session; the truth is its rival.
                check = self_check(fit, wrongs[-1], [truth])
                row['self_check'] = dict(keep=check.keep, reason=check.reason,
                                         leader_fit=check.leader_fit, lead=check.lead)
            rows.append(row)
            print(f"{entry['name']:11s} {label:9s} share {shares} {row['reasons']} committed "
                  f"{row['committed_cells']:4d} self-check "
                  f"{row.get('self_check', {}).get('reason', '-')}", flush=True)
    args.output.write_text(json.dumps(dict(
        schema=1, mode='offline_overlay_wrong_pose', config=config.__dict__,
        samples=rows), indent=1, default=float) + '\n')


if __name__ == '__main__':
    main()
