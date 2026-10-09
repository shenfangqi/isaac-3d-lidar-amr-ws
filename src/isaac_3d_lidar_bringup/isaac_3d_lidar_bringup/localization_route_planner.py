"""
Issue #13 phase 1: candidate-wide safe route planning on the static map.

No motion authority lives here.  A route is a sequence of body-relative
primitives (rotate in place, drive forward).  It is *commonly safe* only
if, from every plausible candidate pose, the whole swept footprint stays in
known-free map space with padding, the candidate pose bound, motion error,
the stopping extension and the sampling error.  Unknown cells and space
outside the map are never traversable.

Predicted score margins (synthetic scans ray cast per layer and scored like
the production matchers) are used only to *choose* a route that separates
the hardest candidate pair; they never accept a pose.

The safety check is conservative: clearance is an exact Euclidean distance
on a supersampled copy of the map minus one sub-cell diagonal.
"""

from dataclasses import dataclass, field
import itertools
import math
import time

import numpy as np

from .localization_contracts import ContractError

ROTATE = 'ROTATE'
FORWARD = 'FORWARD'

DISTINGUISHABLE_NOW = 'DISTINGUISHABLE_NOW'
ROUTE_FOUND = 'ROUTE_FOUND'
NO_COMMON_SAFE_ACTION = 'NO_COMMON_SAFE_ACTION'
NOT_FOUND_WITHIN_BUDGET = 'NOT_FOUND_WITHIN_BUDGET'
INDISTINGUISHABLE_IN_MODEL = 'INDISTINGUISHABLE_IN_MODEL'


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class StaticMap:
    """Nav2 trinary map: 0 free, 100 occupied, -1 unknown (row 0 = bottom)."""

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    map_hash: str = ''

    def __post_init__(self):
        if self.data.ndim != 2 or not self.data.size:
            raise ContractError('map data must be a non-empty 2D array')
        if not (math.isfinite(self.resolution) and self.resolution > 0):
            raise ContractError('map resolution must be positive')
        if not all(math.isfinite(v) for v in (self.origin_x, self.origin_y)):
            raise ContractError('map origin must be finite')

    @property
    def height(self):
        return self.data.shape[0]

    @property
    def width(self):
        return self.data.shape[1]

    def cells(self, xs, ys):
        """Integer cell indices (may lie outside the map)."""
        return (np.floor((np.asarray(xs) - self.origin_x) / self.resolution).astype(int),
                np.floor((np.asarray(ys) - self.origin_y) / self.resolution).astype(int))


def static_map_from_grid(grid, map_hash=''):
    """Convert an OccupancyGrid-like snapshot (origin yaw must be zero)."""
    info = grid.info
    q = info.origin.orientation
    if abs(2.0 * math.atan2(q.z, q.w)) > 1e-9:
        raise ContractError('rotated map origins are not supported')
    data = np.array(grid.data, dtype=np.int16).reshape(info.height, info.width)
    return StaticMap(data, float(info.resolution), float(info.origin.position.x),
                     float(info.origin.position.y), map_hash)


class ClearanceField:
    """
    Conservative distance (m) from a point to the nearest blocked cell.

    Blocked = occupied, unknown and (by default) outside the map.  The exact
    Euclidean transform is computed on a grid ``supersample`` times finer
    than the map, whose sub-cells tile every blocked cell exactly.  The
    distance from a point to blocked area is then at least the distance
    between the two sub-cell centres minus one sub-cell diagonal.
    """

    def __init__(self, static_map, blocked=None, outside_blocked=True, supersample=4):
        from scipy.ndimage import distance_transform_edt
        if type(supersample) is not int or supersample < 1:
            raise ContractError('supersample must be a positive integer')
        self.map = static_map
        if blocked is None:
            blocked = static_map.data != 0
        fine = np.repeat(np.repeat(blocked, supersample, 0), supersample, 1)
        padded = np.full((fine.shape[0] + 2, fine.shape[1] + 2), outside_blocked)
        padded[1:-1, 1:-1] = fine
        self._step = static_map.resolution / supersample
        self._edt = distance_transform_edt(~padded) * self._step
        self._slack = self._step * math.sqrt(2.0)
        self._outside_blocked = outside_blocked

    def clearance(self, xs, ys):
        """Lower bound of the distance to blocked space; <= 0 if inside it."""
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        i = np.floor((xs - self.map.origin_x) / self._step).astype(int)
        j = np.floor((ys - self.map.origin_y) / self._step).astype(int)
        rows, cols = self._edt.shape[0] - 2, self._edt.shape[1] - 2
        inside = (i >= 0) & (i < cols) & (j >= 0) & (j < rows)
        result = np.full(np.shape(i), -1.0 if self._outside_blocked else np.inf)
        result[inside] = self._edt[j[inside] + 1, i[inside] + 1] - self._slack
        return result


@dataclass(frozen=True)
class MotionModel:
    """
    Motion envelope used for every candidate.

    Defaults for translation are ASSUMED (no linear calibration exists);
    ``calibrated`` must stay False until a reviewed linear profile replaces
    them.  Rotation defaults follow the accepted 2026-10-06 rotation profile:
    0.40 rad/s held for the 0.6 s chassis watchdog plus a 5 deg margin.
    """

    footprint: tuple = ((0.155, 0.133), (0.155, -0.133),
                        (-0.130, -0.133), (-0.130, 0.133))
    padding_m: float = 0.05
    rotate_overshoot_rad: float = 0.40 * 0.6 + 0.001 + math.radians(5.0)
    rotate_center_drift_m: float = 0.03
    forward_stop_extension_m: float = 0.12
    forward_lateral_error_m: float = 0.03
    forward_heading_error_rad: float = math.radians(1.5)
    odom_drift_per_m: float = 0.02
    calibrated: bool = False

    def __post_init__(self):
        if not 3 <= len(self.footprint) <= 64:
            raise ContractError('footprint needs 3..64 vertices')
        for name in ('padding_m', 'rotate_overshoot_rad', 'rotate_center_drift_m',
                     'forward_stop_extension_m', 'forward_lateral_error_m',
                     'forward_heading_error_rad', 'odom_drift_per_m'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0):
                raise ContractError(f'{name} must be finite and >= 0')


@dataclass(frozen=True)
class PoseBounds:
    """Hard bound of a candidate's pose error (not a standard deviation)."""

    xy_m: float = 0.05
    yaw_rad: float = math.radians(3.0)


def _inside(point, polygon):
    x, y = point
    result = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            result = not result
    return result


def body_samples(footprint, spacing):
    """Points on the footprint boundary and inside it, at most ``spacing`` apart."""
    points = []
    for (x1, y1), (x2, y2) in zip(footprint, footprint[1:] + footprint[:1]):
        count = max(1, math.ceil(math.hypot(x2 - x1, y2 - y1) / spacing))
        points += [(x1 + (x2 - x1) * k / count, y1 + (y2 - y1) * k / count)
                   for k in range(count)]
    xs = [p[0] for p in footprint]
    ys = [p[1] for p in footprint]
    for x in np.arange(min(xs), max(xs) + spacing, spacing):
        for y in np.arange(min(ys), max(ys) + spacing, spacing):
            if _inside((x, y), footprint):
                points.append((float(x), float(y)))
    return np.array(points)


@dataclass(frozen=True)
class RouteCheck:
    """Safety of one route from one candidate pose."""

    safe: bool
    reason: str
    primitive_index: int
    min_margin_m: float
    stops: tuple          # nominal pose after each primitive
    bounds: tuple         # (xy, yaw) bound after each primitive


def _check_points(fields, points, required):
    """Return (min margin, reason) over all fields; reason names the blocker."""
    blocked, occupied = fields
    clear = blocked.clearance(points[:, 0], points[:, 1])
    margin = clear - required
    worst = float(margin.min())
    if worst >= 0:
        return worst, ''
    # Any violation that is close to an occupied cell is an obstacle: the
    # body would hit it before reaching unknown space behind it.
    violating = margin < 0
    occ = occupied.clearance(points[violating, 0], points[violating, 1])
    reason = ('OBSTACLE_IN_SWEEP' if np.any(occ - required[violating] < 0)
              else 'UNKNOWN_OR_OUTSIDE_IN_SWEEP')
    return worst, reason


def check_route(fields, model, pose, bounds, route):
    """
    Check a body-relative route from ``pose`` = (x, y, yaw) in the map.

    ``fields`` is ``(blocked_field, occupied_field)``; both share the map.
    """
    resolution = fields[0].map.resolution
    samples = body_samples(model.footprint, resolution / 2.0)
    radius = np.hypot(samples[:, 0], samples[:, 1])
    r_max = float(radius.max())
    x, y, yaw = (float(v) for v in pose)
    xy_bound, yaw_bound = bounds.xy_m, bounds.yaw_rad
    stops, bound_trace, worst = [], [], math.inf
    for index, (kind, value) in enumerate(route):
        if kind == ROTATE:
            if not math.isfinite(value) or value == 0:
                raise ContractError('rotation must be finite and nonzero')
            sweep = value + math.copysign(model.rotate_overshoot_rad, value)
            count = max(1, math.ceil(abs(sweep) * r_max / (resolution / 2.0)))
            angles = yaw + np.linspace(0.0, sweep, count + 1)
            step_chord = abs(sweep) / count * r_max
            cosine, sine = np.cos(angles)[:, None], np.sin(angles)[:, None]
            wx = x + cosine * samples[:, 0] - sine * samples[:, 1]
            wy = y + sine * samples[:, 0] + cosine * samples[:, 1]
            required = (model.padding_m + xy_bound + model.rotate_center_drift_m
                        + radius * yaw_bound + step_chord / 2.0)
            required = np.broadcast_to(required, wx.shape)
            margin, reason = _check_points(
                fields, np.stack([wx.ravel(), wy.ravel()], 1), required.ravel())
            yaw = _wrap(yaw + value)
            xy_bound += model.rotate_center_drift_m
        elif kind == FORWARD:
            if not math.isfinite(value) or value <= 0:
                raise ContractError('forward distance must be positive')
            reach = value + model.forward_stop_extension_m
            count = max(1, math.ceil(reach / (resolution / 2.0)))
            distances = np.linspace(0.0, reach, count + 1)[:, None]
            heading = yaw_bound + model.forward_heading_error_rad
            cosine, sine = math.cos(yaw), math.sin(yaw)
            wx = x + cosine * (distances + samples[:, 0]) - sine * samples[:, 1]
            wy = y + sine * (distances + samples[:, 0]) + cosine * samples[:, 1]
            lateral = distances * math.sin(heading) + model.forward_lateral_error_m
            required = (model.padding_m + xy_bound + lateral + radius * yaw_bound
                        + reach / count / 2.0)
            margin, reason = _check_points(
                fields, np.stack([wx.ravel(), wy.ravel()], 1), required.ravel())
            x, y = x + cosine * value, y + sine * value
            xy_bound += (value * model.odom_drift_per_m + model.forward_lateral_error_m
                         + value * math.sin(heading))
            yaw_bound += model.forward_heading_error_rad
        else:
            raise ContractError(f'unknown primitive {kind!r}')
        worst = min(worst, margin)
        if reason:
            return RouteCheck(False, reason, index, worst, tuple(stops),
                              tuple(bound_trace))
        stops.append((x, y, yaw))
        bound_trace.append((xy_bound, yaw_bound))
    return RouteCheck(True, '', -1, worst, tuple(stops), tuple(bound_trace))


class ObservationModel:
    """
    Predict how well candidates explain each other's scans, per layer.

    ``layers`` maps a name to ``(scene, reference)`` boolean grids (True =
    surface) sharing the map geometry.  A synthetic scan is ray cast in the
    *scene* grid (what the sensor would see from a candidate) and scored like
    the production matchers: the share of its endpoints within ``tolerance_m``
    of a *reference* surface (the map the matcher uses).  For the 2D band the
    reference is the navigation map; for 3D bands it is the band itself.
    """

    def __init__(self, static_map, layers, sensor_offset=(0.03, 0.02), beams=180,
                 min_range_m=0.5, max_range_m=8.0, tolerance_m=0.15):
        from scipy.ndimage import distance_transform_edt
        self.static_map = static_map
        self.layers = dict(layers)
        self.sensor_offset = sensor_offset
        self.beams = beams
        self.min_range_m = min_range_m
        self.max_range_m = max_range_m
        self.tolerance_m = tolerance_m
        self._reference_distance = {
            name: distance_transform_edt(~reference) * static_map.resolution
            for name, (_scene, reference) in self.layers.items()}
        self._angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)

    def _sensor(self, pose):
        x, y, yaw = pose
        ox, oy = self.sensor_offset
        return (x + math.cos(yaw) * ox - math.sin(yaw) * oy,
                y + math.sin(yaw) * ox + math.cos(yaw) * oy, yaw)

    def ranges(self, layer, pose):
        """Ray cast the scene grid of ``layer``; inf means no return."""
        grid = self.layers[layer][0]
        sx, sy, yaw = self._sensor(pose)
        angles = yaw + self._angles
        step = self.static_map.resolution / 2.0
        distances = np.arange(self.min_range_m, self.max_range_m, step)
        px = sx + np.cos(angles)[:, None] * distances[None, :]
        py = sy + np.sin(angles)[:, None] * distances[None, :]
        i, j = self.static_map.cells(px, py)
        inside = (i >= 0) & (i < self.static_map.width) & (j >= 0) & (j < self.static_map.height)
        hit = np.zeros(px.shape, bool)
        hit[inside] = grid[j[inside], i[inside]]
        first = np.where(hit.any(1), hit.argmax(1), -1)
        return np.where(first >= 0, distances[np.maximum(first, 0)], np.inf)

    def hit_share(self, layer, ranges, pose):
        """Share of finite endpoints, placed at ``pose``, near reference surfaces."""
        finite = np.isfinite(ranges)
        if not finite.any():
            return None
        sx, sy, yaw = self._sensor(pose)
        angles = yaw + self._angles[finite]
        ex = sx + np.cos(angles) * ranges[finite]
        ey = sy + np.sin(angles) * ranges[finite]
        i, j = self.static_map.cells(ex, ey)
        inside = (i >= 0) & (i < self.static_map.width) & (j >= 0) & (j < self.static_map.height)
        near = np.zeros(ex.shape, bool)
        near[inside] = (self._reference_distance[layer][j[inside], i[inside]]
                        <= self.tolerance_m)
        return float(near.mean())

    def separability(self, layer, first, second):
        """
        Predicted score margin that holds whichever of the two is the truth.

        If ``first`` is true its scan should fit ``first`` better than
        ``second`` and vice versa; the smaller of the two margins counts.
        """
        margins = []
        for truth, other in ((first, second), (second, first)):
            scan = self.ranges(layer, truth)
            own, rival = self.hit_share(layer, scan, truth), self.hit_share(layer, scan, other)
            if own is None:
                return 0.0
            margins.append(own - rival)
        return max(0.0, min(margins))


@dataclass(frozen=True)
class PlannerConfig:
    """Primitive set and budgets of the bounded look-ahead search."""

    rotations: tuple = (math.pi / 4, -math.pi / 4, math.pi / 2, -math.pi / 2, math.pi)
    forwards: tuple = (0.2, 0.4, 0.6)
    max_depth: int = 3
    max_total_forward_m: float = 1.5
    max_total_rotation_rad: float = 2.0 * math.pi
    min_difference: float = 0.15
    time_budget_s: float = 30.0
    layers: tuple = ()         # empty = every layer of the observation model


@dataclass(frozen=True)
class PlanResult:
    """Outcome of :func:`plan_route`; ``route`` is empty when none is needed."""

    status: str
    route: tuple
    objective: float
    pair_differences: dict
    evaluated: int
    unsafe_reasons: dict = field(default_factory=dict)
    initial_objective: float = 0.0


def _pairwise(observation, layers, poses_per_stop, focus=None):
    """
    Predicted margin per candidate pair.

    Layers are averaged (the 3D re-check combines bands).  Over the stops
    the best one counts: validation runs after every segment and its margin
    uses the current view's HOLDOUT, so one telling stop is enough.
    """
    if focus is None:
        pairs = itertools.combinations(range(len(poses_per_stop[0])), 2)
    else:
        # The leader (first focus index) against every other contender: the
        # route must tell the leader apart from each of them, whichever is
        # true; if the leader is wrong it falls behind and the next segment
        # is planned around the new leader.
        pairs = ((focus[0], other) for other in focus[1:])
    best = {pair: 0.0 for pair in pairs}
    for stop in poses_per_stop:
        for a, b in best:
            best[(a, b)] = max(best[(a, b)], sum(
                observation.separability(layer, stop[a], stop[b])
                for layer in layers) / len(layers))
    return best


def plan_route(fields, model, observation, candidates, config=PlannerConfig(),
               focus=None):
    """
    Search a commonly safe route that separates the contending candidates.

    ``candidates`` is a sequence of ``(pose, PoseBounds)`` with poses at the
    current time.  Safety is checked for *every* candidate; none is dropped
    to make a route feasible.  The objective is the smallest predicted
    margin (hardest pair), taking for each pair its best stop (the current
    view included).  Pairs are all candidates by default; with ``focus`` (the
    leader first, then the candidates close enough to keep validation
    ambiguous) they are the leader against each other contender.
    """
    candidates = tuple(candidates)
    if len(candidates) < 2:
        raise ContractError('route planning needs at least two candidates')
    if focus is not None:
        focus = tuple(dict.fromkeys(focus))
        if len(focus) < 2 or any(not 0 <= i < len(candidates) for i in focus):
            raise ContractError('focus needs at least two valid candidate indices')
    layers = config.layers or tuple(observation.layers)
    start = [tuple(float(v) for v in pose) for pose, _ in candidates]
    initial = _pairwise(observation, layers, [start], focus)
    initial_objective = min(initial.values())
    if initial_objective >= config.min_difference:
        return PlanResult(DISTINGUISHABLE_NOW, (), initial_objective, initial, 0,
                          {}, initial_objective)
    primitives = ([(ROTATE, a) for a in config.rotations]
                  + [(FORWARD, d) for d in config.forwards])
    deadline = time.monotonic() + config.time_budget_s
    unsafe, evaluated = {}, 0
    best = None              # (reaches, objective, -cost, route, pairs)
    any_safe_first = False
    frontier = [()]
    exhausted = True
    for depth in range(1, config.max_depth + 1):
        next_frontier = []
        for prefix in frontier:
            for primitive in primitives:
                if (prefix and prefix[-1][0] == ROTATE and primitive[0] == ROTATE
                        and prefix[-1][1] * primitive[1] < 0):
                    continue             # opposite turns cancel each other
                # Same-direction turns may follow each other: a runtime turn
                # is at most 90 deg, so a half turn needs two segments.
                route = prefix + (primitive,)
                forward = sum(v for k, v in route if k == FORWARD)
                turning = sum(abs(v) for k, v in route if k == ROTATE)
                if (forward > config.max_total_forward_m + 1e-9
                        or turning > config.max_total_rotation_rad + 1e-9):
                    continue
                if time.monotonic() > deadline:
                    exhausted = False
                    break
                evaluated += 1
                checks = [check_route(fields, model, pose, bounds, route)
                          for pose, bounds in candidates]
                failed = [c for c in checks if not c.safe]
                if failed:
                    for check in failed:
                        unsafe[check.reason] = unsafe.get(check.reason, 0) + 1
                    continue
                if depth == 1:
                    any_safe_first = True
                next_frontier.append(route)
                stops = [start] + [[c.stops[k] for c in checks]
                                   for k in range(len(route))]
                pairs = _pairwise(observation, layers, stops, focus)
                objective = min(pairs.values())
                cost = forward + 0.2 * turning
                key = (objective >= config.min_difference, objective, -cost)
                if best is None or key > best[:3]:
                    best = key + (route, pairs)
            if not exhausted:
                break
        if not exhausted:
            break
        frontier = next_frontier
        if not frontier:
            break
    if best is not None and best[0]:
        return PlanResult(ROUTE_FOUND, best[3], best[1], best[4], evaluated,
                          unsafe, initial_objective)
    if not any_safe_first and exhausted:
        return PlanResult(NO_COMMON_SAFE_ACTION, (), initial_objective, initial,
                          evaluated, unsafe, initial_objective)
    if not exhausted:
        route, objective, pairs = ((best[3], best[1], best[4]) if best
                                   else ((), initial_objective, initial))
        return PlanResult(NOT_FOUND_WITHIN_BUDGET, route, objective, pairs,
                          evaluated, unsafe, initial_objective)
    return PlanResult(INDISTINGUISHABLE_IN_MODEL, best[3] if best else (),
                      best[1] if best else initial_objective,
                      best[4] if best else initial, evaluated, unsafe,
                      initial_objective)


def run_route_plan_job(static_map, layers, candidates, model, config, focus=None):
    """Worker entry point: build the fields and observation model, then plan."""
    fields = (ClearanceField(static_map),
              ClearanceField(static_map, blocked=static_map.data == 100,
                             outside_blocked=False))
    observation = ObservationModel(static_map, layers)
    return plan_route(fields, model, observation, candidates, config, focus)
