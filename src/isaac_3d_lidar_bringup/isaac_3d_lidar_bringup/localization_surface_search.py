"""
Issue #13 stage D: map-wide 3D candidate generation.

Pure logic (numpy/scipy), no ROS.  A stationary 3D cloud is scored at every
free position (from the navigation map) and heading of a coarse grid
against a precomputed fit field of the map surface: ``exp(-d^2 / 2 sigma^2)``
of the distance ``d`` to the nearest surface sample, cut to zero beyond
``cap_m``, with ``sigma`` matched to the grid (see SurfaceSearchConfig).
One table lookup per point and pose makes the whole map cheap (about 1 s
on the workstation for 610 positions x 36 headings x 1500 points,
2026-10-10).

This only *generates* candidates.  Every candidate within ``margin`` of
the best coarse score is returned, after one-per-neighbourhood suppression,
so that the 3D decision (``localization_surface_validation``) sees every
place that could still win once refined.  More than ``max_candidates``
such places is reported as incomplete, never truncated.
"""

from dataclasses import dataclass
import math

import numpy as np

from .localization_contracts import ContractError


@dataclass(frozen=True)
class SurfaceSearchConfig:
    """
    Coarse map-wide 3D search; provisional values (one site).

    The coarse fit is deliberately blurred (``sigma_m`` near the grid
    spacing) so that a node up to half a cell from the true pose still
    scores close to it.  With sigma 0.05 m the node next to a true pose in a
    corner (scan ranges of ~0.7 m) scored up to 0.26 below the best, so the
    truth fell outside the margin and a wrong corner was accepted
    (2026-10-10 synthetic analysis).  With 0.15 m the loss stayed <= 0.063
    in 1098 synthetic views and all 8 labelled bags, and every true pose
    was among the candidates (at most 26 of them).
    """

    voxel_m: float = .05
    sigma_m: float = .15
    cap_m: float = .45
    position_step_m: float = .20
    clearance_m: float = .15
    yaw_step_rad: float = math.radians(10)
    points: int = 1500
    min_range_m: float = .5
    margin: float = .10
    nms_xy_m: float = .30
    nms_yaw_rad: float = math.radians(15)
    max_candidates: int = 40
    occupied_threshold: int = 65

    def __post_init__(self):
        values = (self.voxel_m, self.sigma_m, self.cap_m, self.position_step_m,
                  self.yaw_step_rad, self.margin, self.nms_xy_m, self.nms_yaw_rad)
        if any(not math.isfinite(v) or v <= 0 for v in values) or self.clearance_m < 0:
            raise ContractError('invalid 3D search limits')
        if (type(self.points) is not int or self.points < 1
                or type(self.max_candidates) is not int or self.max_candidates < 1):
            raise ContractError('invalid 3D search budget')


@dataclass(frozen=True)
class SurfaceField:
    """Fit of every voxel to the nearest map surface sample."""

    origin: np.ndarray        # (3,) corner of voxel (0, 0, 0)
    voxel_m: float
    fit: np.ndarray           # (nx, ny, nz) float32 in [0, 1]


@dataclass(frozen=True)
class SurfaceSearchResult:
    """Candidates within the margin of the best coarse score."""

    complete: bool
    reason: str
    candidates: tuple         # ((x, y, yaw, coarse_score), ...) best first
    best: float
    evaluated: int


def build_surface_field(surface_points, config=SurfaceSearchConfig()):
    """Fit field of the surface points (map-derived; depends on the map only)."""
    from scipy import ndimage
    points = np.asarray(surface_points, float).reshape(-1, 3)
    if not len(points):
        raise ContractError('the 3D search needs a non-empty surface')
    pad = config.cap_m + config.voxel_m
    origin = points.min(0) - pad
    shape = np.ceil((points.max(0) + pad - origin) / config.voxel_m).astype(int) + 1
    empty = np.ones(shape, bool)
    index = np.floor((points - origin) / config.voxel_m).astype(int)
    empty[tuple(index.T)] = False
    distance = ndimage.distance_transform_edt(empty, sampling=config.voxel_m)
    fit = np.exp(-.5 * (np.minimum(distance, config.cap_m) / config.sigma_m) ** 2)
    fit[distance >= config.cap_m] = 0.
    return SurfaceField(origin, config.voxel_m, fit.astype(np.float32))


def free_positions(grid, config=SurfaceSearchConfig()):
    """Free cells of a Nav2 trinary map with clearance, every ``position_step_m``."""
    from scipy import ndimage
    info = grid.info
    cells = np.asarray(grid.data, dtype=np.int8).reshape(info.height, info.width)
    occupied = cells >= config.occupied_threshold
    clearance = ndimage.distance_transform_edt(~occupied, sampling=info.resolution)
    free = (cells == 0) & (clearance >= config.clearance_m)
    step = max(1, int(round(config.position_step_m / info.resolution)))
    rows, cols = np.nonzero(free[::step, ::step])
    origin = info.origin.position
    if getattr(info.origin.orientation, 'w', 1.0) < 1.0 - 1e-9:
        raise ContractError('rotated map origins are not supported by the 3D search')
    return np.c_[origin.x + (cols * step + .5) * info.resolution,
                 origin.y + (rows * step + .5) * info.resolution]


def coarse_scores(field, points, positions, yaws):
    """Mean field fit of ``points`` (base frame) at every position x yaw."""
    points = np.asarray(points, float).reshape(-1, 3)
    shape = np.array(field.fit.shape)
    height = np.floor((points[:, 2] - field.origin[2]) / field.voxel_m).astype(int)
    scores = np.empty((len(positions), len(yaws)), np.float32)
    for column, yaw in enumerate(yaws):
        c, s = math.cos(yaw), math.sin(yaw)
        planar = points[:, :2] @ np.array([[c, s], [-s, c]])
        world = planar[None, :, :] + np.asarray(positions)[:, None, :]
        index = np.floor((world - field.origin[:2]) / field.voxel_m).astype(int)
        z = np.broadcast_to(height, index.shape[:2])
        inside = ((index[..., 0] >= 0) & (index[..., 0] < shape[0])
                  & (index[..., 1] >= 0) & (index[..., 1] < shape[1])
                  & (z >= 0) & (z < shape[2]))
        values = np.zeros(inside.shape, np.float32)
        values[inside] = field.fit[index[..., 0][inside], index[..., 1][inside], z[inside]]
        scores[:, column] = values.mean(1)
    return scores


def _turn(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def search_surface(field, points, positions, config=SurfaceSearchConfig()):
    """
    Map-wide coarse 3D search of a stationary cloud.

    ``points`` are in the reference base frame.  Returns every
    neighbourhood-suppressed candidate within ``margin`` of the best
    coarse score, best first, or ``complete=False`` if there are more than
    ``max_candidates`` of them.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.hypot(points[:, 0], points[:, 1]) >= config.min_range_m]
    if len(points) > config.points:
        points = points[np.linspace(0, len(points) - 1, config.points).astype(int)]
    if not len(points) or not len(positions):
        return SurfaceSearchResult(False, 'NO_POINTS_OR_POSITIONS', (), 0., 0)
    yaws = np.arange(-math.pi, math.pi - 1e-9, config.yaw_step_rad)
    scores = coarse_scores(field, points, positions, yaws)
    best = float(scores.max())
    order = np.argsort(-scores, axis=None)
    chosen = []
    for flat in order:
        row, column = divmod(int(flat), len(yaws))
        score = float(scores[row, column])
        if score < best - config.margin:
            break
        x, y, yaw = float(positions[row][0]), float(positions[row][1]), float(yaws[column])
        if any(math.hypot(x - q[0], y - q[1]) < config.nms_xy_m
               and _turn(yaw, q[2]) < config.nms_yaw_rad for q in chosen):
            continue
        if len(chosen) == config.max_candidates:
            return SurfaceSearchResult(False, 'TOO_MANY_CANDIDATES', tuple(chosen), best,
                                       scores.size)
        chosen.append((x, y, yaw, round(score, 4)))
    return SurfaceSearchResult(True, '', tuple(chosen), best, scores.size)
