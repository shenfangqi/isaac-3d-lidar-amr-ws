"""
Local straight-line probe previews in odom, without a global pose.

No ROS publisher or motion authorization lives here. A preview covers the
entire footprint extrusion including a supplied worst-case stopping tail.
Only observed free space is usable; a rotation attestation is not accepted.
"""

from dataclasses import dataclass
import math
import time

from .localization_contracts import ContractError, SE2
from .localization_rotation_policy import (
    FREE, OCCUPIED, UNKNOWN, _inside, _polygon_distance, _transform,
)


@dataclass(frozen=True)
class TranslationConfig:
    min_distance_m: float = 0.15
    max_distance_m: float = 0.60
    padding_m: float = 0.05
    max_source_age_s: float = 0.5
    openness_radius_m: float = 0.75
    max_evaluated_cells: int = 100000

    def __post_init__(self):
        values = (self.min_distance_m, self.max_distance_m, self.padding_m,
                  self.max_source_age_s, self.openness_radius_m)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ContractError('translation limits must be finite and positive')
        if self.min_distance_m > self.max_distance_m or self.max_source_age_s > .5:
            raise ContractError('invalid translation distance/freshness bounds')
        if type(self.max_evaluated_cells) is not int or self.max_evaluated_cells <= 0:
            raise ContractError('cell budget must be a positive integer')


@dataclass(frozen=True)
class TranslationPreview:
    geometry_clear: bool
    reason: str
    distance_m: float
    stop_extension_m: float
    start: SE2
    target: SE2
    snapshot_stamp_ns: int
    occupied_cells: int = 0
    unknown_cells: int = 0
    endpoint_openness: float = 0.0

    def __post_init__(self):
        if (type(self.geometry_clear) is not bool
                or not isinstance(self.start, SE2) or not isinstance(self.target, SE2)
                or type(self.snapshot_stamp_ns) is not int or self.snapshot_stamp_ns <= 0
                or any(not math.isfinite(v) for v in
                       (self.distance_m, self.stop_extension_m, self.endpoint_openness))
                or self.distance_m <= 0 or self.stop_extension_m < 0
                or not 0 <= self.endpoint_openness <= 1
                or any(type(v) is not int or v < 0 for v in
                       (self.occupied_cells, self.unknown_cells))):
            raise ContractError('invalid translation preview')
        if self.geometry_clear and (self.reason or self.occupied_cells or self.unknown_cells):
            raise ContractError('clear preview cannot contain rejection evidence')


def _convex_hull(points):
    """Conservative for concave bodies; exact extrusion for convex bodies."""
    points = sorted(set(points))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    lower, upper = [], []
    for sequence, result in ((points, lower), (reversed(points), upper)):
        for p in sequence:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
    return tuple(lower[:-1] + upper[:-1])


def _bounds(evidence, polygon, padding):
    first = evidence.cell(min(x for x, _ in polygon) - padding,
                          min(y for _, y in polygon) - padding)
    last = evidence.cell(max(x for x, _ in polygon) + padding,
                         max(y for _, y in polygon) + padding)
    return first[0], first[1], last[0], last[1]


def preview_translation(evidence, footprint, start, distance_m, *,
                        stop_extension_m, now_ns,
                        config=TranslationConfig(), deadline=None):
    """
    Preview forward travel from the current heading, never an unchecked turn.

    stop_extension_m must be supplied by a separately validated linear stop
    envelope, including watchdog/latency, tail and pose uncertainty. Zero is
    allowed for geometry tests only; this function never grants authority.
    """
    if (not math.isfinite(distance_m) or distance_m <= 0
            or distance_m > config.max_distance_m
            or not math.isfinite(stop_extension_m) or stop_extension_m < 0):
        raise ContractError('invalid forward distance or stop extension')
    if type(now_ns) is not int or now_ns <= 0:
        raise ContractError('now_ns must be a positive source-clock timestamp')
    if not (3 <= len(footprint) <= 64 and all(
            len(p) == 2 and all(math.isfinite(v) for v in p) for p in footprint)):
        raise ContractError('invalid footprint')
    body_local = _convex_hull(footprint)
    # Self masking is valid only for an ordered convex physical body. A hull
    # of a concave or crossed polygon would erase unknown space outside it.
    area = abs(sum(a[0]*b[1] - b[0]*a[1]
                   for a, b in zip(footprint, footprint[1:] + footprint[:1])))
    hull_area = abs(sum(a[0]*b[1] - b[0]*a[1]
                        for a, b in zip(body_local, body_local[1:] + body_local[:1])))
    if not math.isclose(area, hull_area, rel_tol=1e-9, abs_tol=1e-12):
        raise ContractError('footprint must be an ordered convex polygon')
    if len(body_local) < 3 or not _inside((0., 0.), body_local):
        raise ContractError('footprint must enclose the base origin')
    target = start.compose(SE2(distance_m, 0., 0.))

    def result(reason, occupied=0, unknown=0, openness=0.):
        return TranslationPreview(not reason, reason, distance_m,
                                  stop_extension_m, start, target,
                                  evidence.stamp_ns, occupied, unknown, openness)
    if evidence.frame_id != 'odom':
        return result('WRONG_EVIDENCE_FRAME')
    if not 0 <= (now_ns - evidence.stamp_ns) / 1e9 <= config.max_source_age_s:
        return result('SENSOR_STALE')
    body = _transform(body_local, start.x, start.y, start.yaw)
    end = start.compose(SE2(distance_m + stop_extension_m, 0., 0.))
    sweep = _convex_hull(body + _transform(body_local, end.x, end.y, end.yaw))
    half = evidence.resolution / 2
    threshold = config.padding_m + half * math.sqrt(2)
    x0, y0, x1, y1 = _bounds(evidence, sweep, threshold)
    radius = config.openness_radius_m
    view_box = ((target.x - radius, target.y - radius),
                (target.x + radius, target.y + radius))
    vx0, vy0, vx1, vy1 = _bounds(evidence, view_box, 0.)
    if ((x1 - x0 + 1) * (y1 - y0 + 1)
            + (vx1 - vx0 + 1) * (vy1 - vy0 + 1) > config.max_evaluated_cells):
        return result('PREVIEW_BUDGET_EXHAUSTED')
    occupied = unknown = 0
    for my in range(y0, y1 + 1):
        if deadline is not None and time.monotonic() >= deadline:
            return result('PREVIEW_BUDGET_EXHAUSTED')
        for mx in range(x0, x1 + 1):
            cx, cy = evidence.center(mx, my)
            if _polygon_distance((cx, cy), sweep) > threshold:
                continue
            state = evidence.state(mx, my)
            # Even a return inside the current body cannot be silently erased.
            if state == OCCUPIED:
                occupied += 1
            elif state == UNKNOWN:
                corners = ((cx - half, cy - half), (cx - half, cy + half),
                           (cx + half, cy - half), (cx + half, cy + half))
                if not all(_inside(p, body) for p in corners):
                    unknown += 1
    if occupied:
        return result('OBSTACLE_IN_SWEEP', occupied, unknown)
    if unknown:
        return result('UNKNOWN_SWEEP', occupied, unknown)
    free = total = 0
    for my in range(vy0, vy1 + 1):
        if deadline is not None and time.monotonic() >= deadline:
            return result('PREVIEW_BUDGET_EXHAUSTED')
        for mx in range(vx0, vx1 + 1):
            cx, cy = evidence.center(mx, my)
            if math.hypot(cx - target.x, cy - target.y) <= radius:
                total += 1
                free += evidence.state(mx, my) == FREE
    return result('', openness=free / total if total else 0.)


def choose_translation(previews, config=TranslationConfig()):
    """
    Prefer observed-open endpoints, then a larger useful viewpoint change.

    This heuristic does not predict information gain. Candidates must all
    describe the same starting pose and source scan; stale mixtures fail.
    """
    previews = tuple(previews)
    if len({(p.start, p.snapshot_stamp_ns) for p in previews}) > 1:
        raise ContractError('cannot mix preview poses or source stamps')
    admissible = [p for p in previews if p.geometry_clear
                  and config.min_distance_m <= p.distance_m <= config.max_distance_m]
    return max(admissible, key=lambda p: (p.endpoint_openness, p.distance_m),
               default=None)
