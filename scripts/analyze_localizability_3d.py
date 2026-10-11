#!/usr/bin/env python3
"""
Where can the robot localize from one stationary 3D view?  (read-only)

Part of the new-map initialization (plan stage D): for sampled free
positions and headings, render the scan a MID-360 would see if the scene
were exactly the map (ray casting against the map surface voxels), then
run the production 3D decision on it: map-wide 3D search, refinement of
every candidate, see-through exclusion, ranking on two noisy copies.

A place where even this map-perfect scan is ambiguous cannot be localized
from a stationary view; it needs motion or a fixed asymmetric feature.  A
place that passes is only a candidate for the supported region: real scans
add clutter, occlusion and scene changes, so it still needs robot evidence.
No ROS node is started.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from diagnose_localization_bag import load_map  # noqa: E402
from isaac_3d_lidar_bringup.localization_surface_check import (  # noqa: E402
    check_surfaces, planar, SurfaceCheckConfig, surface_tree)
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_search import (  # noqa: E402
    build_surface_field, free_positions, search_surface, SurfaceSearchConfig)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    FIT_CHECK, RANK_CHECK, SurfaceRefineConfig, validate_surface_evidence)
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions)

# MID-360: 360 deg horizontal, -7..+52 deg vertical (datasheet).
ELEVATIONS = np.radians(np.arange(-7, 52.1, 2.0))
AZIMUTHS = np.radians(np.arange(0, 360, 1.0))
VOXEL = .05
MAX_RANGE = 15.


class Renderer:
    """First hit of sensor rays on the map surface voxels."""

    def __init__(self, surface_points):
        self.origin = surface_points.min(0) - VOXEL
        index = np.floor((surface_points - self.origin) / VOXEL).astype(int)
        self.occupied = np.zeros(index.max(0) + 2, bool)
        self.occupied[tuple(index.T)] = True
        elevation, azimuth = np.meshgrid(ELEVATIONS, AZIMUTHS, indexing='ij')
        self.directions = np.c_[(np.cos(elevation) * np.cos(azimuth)).ravel(),
                                (np.cos(elevation) * np.sin(azimuth)).ravel(),
                                np.sin(elevation).ravel()]
        # Real MID-360 returns start at about 0.1 m: near objects must occlude.
        self.steps = np.arange(.05, MAX_RANGE, VOXEL / 2)

    def scan(self, pose, sensor):
        """Points in the base frame seen from ``pose`` (map) by ``sensor`` (base)."""
        matrix = planar(*pose)
        start = (matrix @ np.r_[sensor, 1.])[:3]
        world_dirs = self.directions @ matrix[:3, :3].T
        hits = np.empty(len(world_dirs))
        shape = np.array(self.occupied.shape)
        for lo in range(0, len(world_dirs), 2000):
            d = world_dirs[lo:lo + 2000]
            samples = start + d[:, None, :] * self.steps[None, :, None]
            index = np.floor((samples - self.origin) / VOXEL).astype(int)
            inside = ((index >= 0) & (index < shape)).all(-1)
            hit = np.zeros(inside.shape, bool)
            clipped = np.clip(index, 0, shape - 1)
            hit[inside] = self.occupied[clipped[inside][:, 0], clipped[inside][:, 1],
                                        clipped[inside][:, 2]]
            first = np.where(hit.any(1), hit.argmax(1), -1)
            hits[lo:lo + 2000] = np.where(first >= 0, self.steps[np.maximum(first, 0)], np.nan)
        keep = np.isfinite(hits)
        world = start + world_dirs[keep] * hits[keep, None]
        inverse = np.linalg.inv(matrix)
        return (inverse @ np.c_[world, np.ones(len(world))].T).T[:, :3]


_STATE = {}


def _added_surface(boxes):
    """Points filling ``xmin,ymin,zmin,xmax,ymax,zmax`` boxes (2 cm), e.g. a curtain."""
    parts = []
    for box in boxes or ():
        low, high = np.asarray(box[:3]), np.asarray(box[3:])
        axes = [np.arange(a, b + 1e-9, .02) if b - a > .02 else np.array([(a + b) / 2])
                for a, b in zip(low, high)]
        grid = np.meshgrid(*axes, indexing='ij')
        parts.append(np.c_[grid[0].ravel(), grid[1].ravel(), grid[2].ravel()])
    return np.vstack(parts) if parts else np.zeros((0, 3))


def _init(mesh, spacing, map_path, cache_dir=None, regions_path=None, added=None):
    model = load_surface_model(mesh, spacing, cache_dir)
    surface = model.points.astype(float)
    regions = (load_variable_regions(regions_path, model.mesh_sha256)
               if regions_path else None)
    config = SurfaceSearchConfig()
    # The renderer sees the whole mesh (the scene); scoring ignores regions.
    _STATE.update(surface=surface, tree=surface_tree(surface, regions),
                  field=build_surface_field(surface, config, regions), config=config,
                  # The scene may differ from the map (--add-surface): only
                  # the renderer sees the added objects.
                  renderer=Renderer(np.r_[surface, _added_surface(added)]),
                  positions=free_positions(load_map(map_path), config))


def degrade(points, rng, clutter, no_ceiling, sensor):
    """
    Make a map-perfect scan less perfect, as robot scans are (2026-10-10).

    ``no_ceiling`` drops points above 2 m (the mapped ceiling was 0.2 m off).
    ``clutter`` is the share of points hidden behind unmapped objects: random
    20 deg azimuth sectors get an occluder 0.8-2 m from the sensor, and
    their points between 0.2 and 1.5 m high move closer along their own
    3D rays.  Objects can only occlude: a point behind a mapped wall would
    be a see-through no real scene has.
    """
    if no_ceiling:
        points = points[points[:, 2] <= 2.0]
    if clutter <= 0:
        return points
    points = points.copy()
    sensor = np.asarray(sensor, float)
    rays = points - sensor
    azimuth = np.arctan2(rays[:, 1], rays[:, 0])
    reach = np.linalg.norm(rays, axis=1)
    band = (points[:, 2] >= .2) & (points[:, 2] <= 1.5)
    hidden = np.zeros(len(points), bool)
    for _ in range(200):
        if hidden.mean() >= clutter:
            break
        center, distance = rng.uniform(-math.pi, math.pi), rng.uniform(.8, 2.)
        inside = band & ~hidden & (np.abs(np.arctan2(np.sin(azimuth - center),
                                                     np.cos(azimuth - center)))
                                   <= math.radians(10))
        inside &= reach > distance
        points[inside] = sensor + rays[inside] * (distance / reach[inside])[:, None]
        hidden |= inside
    return points


def analyze(job):
    pose, sensor, seed, clutter, no_ceiling = job
    state = _STATE
    rng = np.random.default_rng(seed)
    started = time.monotonic()
    clean = state['renderer'].scan(pose, sensor)
    clean = degrade(clean, rng, clutter, no_ceiling, sensor)
    clean = clean[np.hypot(clean[:, 0], clean[:, 1]) >= .5]
    if len(clean) < 2000:
        return dict(pose=pose, verdict='TOO_FEW_POINTS', points=int(len(clean)))
    first = clean + rng.normal(0, .01, clean.shape)
    second = clean + rng.normal(0, .01, clean.shape)
    found = search_surface(state['field'], first, state['positions'], state['config'])
    if not found.complete:
        return dict(pose=pose, verdict='3D_SEARCH_' + found.reason,
                    search_candidates=len(found.candidates),
                    seconds=round(time.monotonic() - started, 2))
    starts = [c[:3] for c in found.candidates]
    check = SurfaceCheckConfig(tolerance_m=RANK_CHECK.tolerance_m, min_leader_composite=.01,
                               min_points=2000, min_band_points=200)
    validation = validate_surface_evidence(
        state['tree'], first, second, starts, deadline=time.monotonic() + 300,
        check_config=check, sensor_origin=sensor,
        # As in decide_surface: the leader may drift up to the 2D cluster radius.
        refine_config=SurfaceRefineConfig(leader_shift_m=.30, leader_turn_rad=math.radians(15)))
    reason = validation.reason
    leader = (validation.poses[validation.leader]
              if 0 <= validation.leader < len(validation.poses) else None)
    gap = (min(validation.train.composite_gap, validation.holdout.composite_gap)
           if validation.train and validation.holdout else None)
    if not reason and leader is not None:
        # As decide_surface: the absolute 3D fit of the leader (2D sanity is
        # not emulated: no 2D scan is rendered).
        fit = check_surfaces(state['tree'], second, (leader, leader),
                             SurfaceCheckConfig(min_points=2000, min_band_points=200))
        if fit.composite[0] < FIT_CHECK.min_leader_composite:
            reason = 'LEADER_FIT_TOO_LOW'
    correct = leader is not None and (
        math.hypot(leader[0] - pose[0], leader[1] - pose[1]) <= .15
        and abs(math.atan2(math.sin(leader[2] - pose[2]),
                           math.cos(leader[2] - pose[2]))) <= math.radians(6))
    verdict = ('ACCEPT' if not reason and correct else
               'WRONG_LEADER' if not reason else reason)
    return dict(pose=pose, verdict=verdict, gap=gap,
                search_candidates=len(found.candidates),
                seconds=round(time.monotonic() - started, 2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=.02)
    parser.add_argument('--step', type=float, default=.4, help='position step, m')
    parser.add_argument('--headings', type=int, default=4)
    parser.add_argument('--sensor', type=float, nargs=3, default=(.027, .023, .165),
                        help='sensor position in base_footprint')
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--variable-regions', type=Path,
                        help='variable-region YAML: ignored by scoring, still rendered')
    parser.add_argument('--add-surface', action='append', default=[],
                        type=lambda text: [float(v) for v in text.split(',')],
                        metavar='XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX',
                        help='an object in the scene but not in the map (repeatable)')
    parser.add_argument('--region-bounds', type=float, nargs=4,
                        metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'),
                        help='analyse only positions inside this map box')
    parser.add_argument('--cache-dir', type=Path,
                        help='surface-sample cache (default: next to the mesh)')
    parser.add_argument('--clutter', type=float, default=0.,
                        help='share of points hidden behind unmapped objects')
    parser.add_argument('--no-ceiling', action='store_true',
                        help='drop points above 2 m')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    load_surface_model(args.mesh, args.spacing, args.cache_dir)
    config = SurfaceSearchConfig(position_step_m=args.step, clearance_m=.25)
    positions = free_positions(load_map(args.map), config)
    if args.region_bounds:
        x0, y0, x1, y1 = args.region_bounds
        positions = positions[(positions[:, 0] >= x0) & (positions[:, 0] <= x1)
                              & (positions[:, 1] >= y0) & (positions[:, 1] <= y1)]
    yaws = np.radians(np.arange(args.headings) * 360. / args.headings + 7.)
    jobs = [((float(x), float(y), float(math.atan2(math.sin(a), math.cos(a)))),
             tuple(args.sensor), k, args.clutter, args.no_ceiling) for k, ((x, y), a) in
            enumerate((p, a) for p in positions for a in yaws)]
    print(f'{len(positions)} positions x {len(yaws)} headings = {len(jobs)} views', flush=True)
    rows = []
    with ProcessPoolExecutor(args.workers, initializer=_init,
                             initargs=(args.mesh, args.spacing, args.map,
                                       args.cache_dir, args.variable_regions,
                                       args.add_surface)) as pool:
        for row in pool.map(analyze, jobs, chunksize=4):
            rows.append(row)
    by_place = {}
    for row in rows:
        by_place.setdefault((round(row['pose'][0], 2), round(row['pose'][1], 2)), []).append(row)
    places = [dict(x=x, y=y, all_accepted=all(r['verdict'] == 'ACCEPT' for r in views),
                   verdicts=[r['verdict'] for r in views],
                   min_gap=min((r.get('gap') for r in views if r.get('gap') is not None),
                               default=None))
              for (x, y), views in sorted(by_place.items())]
    counts = {}
    for row in rows:
        counts[row['verdict']] = counts.get(row['verdict'], 0) + 1
    summary = dict(views=len(rows), verdicts=counts,
                   places=len(places), places_all_accepted=sum(p['all_accepted'] for p in places),
                   wrong_leaders=counts.get('WRONG_LEADER', 0))
    print(json.dumps(summary), flush=True)
    args.output.write_text(json.dumps(dict(
        schema=1, mode='offline_localizability_3d', navigation_accepted=False,
        clutter=args.clutter, no_ceiling=args.no_ceiling,
        variable_regions=str(args.variable_regions or ''), added_surface=args.add_surface,
        assumptions=['scene identical to the map apart from --clutter/--no-ceiling',
                     'MID-360 FOV -7..52 deg',
                     'no robot self-occlusion', '1 cm point noise, two noisy copies'],
        step_m=args.step, headings_deg=np.degrees(yaws).round(1).tolist(),
        summary=summary, places=places, views=rows), indent=1) + '\n')


if __name__ == '__main__':
    np.seterr(all='ignore')
    main()
