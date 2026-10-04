"""Conservative swept-footprint checks against Nav2-style 2D costmaps.

This module is deliberately ROS-independent and never issues motion commands.
Unknown cells, map exits and lethal cells fail closed. Inscribed cost at the
robot center is rejected; inscribed inflation under a footprint edge is scored
without treating that inflated cell as a second physical obstacle.
"""

from dataclasses import dataclass
import math
import time
from numbers import Integral
from typing import Iterable, Optional, Sequence, Tuple


Point = Tuple[float, float]

FREE_SPACE = 0
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


@dataclass(frozen=True)
class Pose2D:
    """A planar pose in the costmap frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class CostmapSnapshot:
    """Immutable row-major costmap snapshot with its source timestamps."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    costs: Tuple[int, ...]
    frame_id: str
    stamp_sec: float
    received_monotonic_sec: float

    def __post_init__(self):
        object.__setattr__(self, 'costs', tuple(self.costs))
        if self.width <= 0 or self.height <= 0:
            raise ValueError('costmap dimensions must be positive')
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError('costmap resolution must be finite and positive')
        if (not math.isfinite(self.origin_x)
                or not math.isfinite(self.origin_y)):
            raise ValueError('costmap origin must be finite')
        if len(self.costs) != self.width * self.height:
            raise ValueError('cost array length does not match dimensions')
        if not self.frame_id:
            raise ValueError('costmap frame_id must not be empty')
        if any(not isinstance(cost, Integral)
               or cost < 0 or cost > NO_INFORMATION
               for cost in self.costs):
            raise ValueError('cost values must be in [0, 255]')

    def cost_at(self, mx: int, my: int) -> int:
        """Return the cell cost at integer map coordinates."""
        return int(self.costs[my * self.width + mx])


@dataclass(frozen=True)
class SnapshotFreshness:
    """Time and localization checks required before evaluating motion."""

    now_ros_sec: float
    now_monotonic_sec: float
    max_source_age_sec: float
    max_receive_age_sec: float
    localization_valid: bool
    tf_valid: bool


@dataclass(frozen=True)
class ObservedFreeSpaceSnapshot:
    """Per-cell recent sensor evidence; false means unknown or not visible.

    Producers must only mark a cell true when sensor rays have established
    free space there. A static map's free cells are not valid input here.
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    observed_free: Tuple[bool, ...]
    frame_id: str
    stamp_sec: float
    received_monotonic_sec: float

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError('observation dimensions must be positive')
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError(
                'observation resolution must be finite and positive')
        if (not math.isfinite(self.origin_x)
                or not math.isfinite(self.origin_y)):
            raise ValueError('observation origin must be finite')
        object.__setattr__(self, 'observed_free', tuple(self.observed_free))
        if any(type(value) is not bool for value in self.observed_free):
            raise ValueError('visibility cells must be explicit booleans')
        if len(self.observed_free) != self.width * self.height:
            raise ValueError(
                'observation array length does not match dimensions')
        if not self.frame_id:
            raise ValueError('observation frame_id must not be empty')


@dataclass(frozen=True)
class CheckResult:
    """A check outcome with a stable diagnostic reason and cost summary."""

    safe: bool
    reason: str
    min_cost: int = NO_INFORMATION
    max_cost: int = NO_INFORMATION
    inflation_cost_sum: int = 0
    samples: int = 0
    blocked_cell: Optional[Point] = None


@dataclass(frozen=True)
class RotationCandidate:
    """One independently evaluated signed rotation candidate."""

    target_yaw: float
    delta_yaw: float
    sweep: CheckResult
    departure: CheckResult

    @property
    def safe(self) -> bool:
        return self.sweep.safe and self.departure.safe


def check_snapshot_freshness(
    snapshot: CostmapSnapshot,
    freshness: SnapshotFreshness,
) -> CheckResult:
    """Reject old time, invalid localization, or invalid TF evidence."""
    data_result = check_snapshot_data_freshness(snapshot, freshness)
    if not data_result.safe:
        return data_result
    if not freshness.localization_valid:
        return CheckResult(False, 'LOCALIZATION_INVALID')
    return CheckResult(True, 'OK')


def check_snapshot_data_freshness(
    snapshot: CostmapSnapshot,
    freshness: SnapshotFreshness,
) -> CheckResult:
    """Check time and TF freshness, independent of localization quality."""
    values = (
        freshness.now_ros_sec,
        freshness.now_monotonic_sec,
        freshness.max_source_age_sec,
        freshness.max_receive_age_sec,
        snapshot.stamp_sec,
        snapshot.received_monotonic_sec,
    )
    if not all(math.isfinite(value) for value in values):
        return CheckResult(False, 'STALE_SENSOR')
    if snapshot.stamp_sec <= 0.0:
        return CheckResult(False, 'STALE_SENSOR')
    if (freshness.max_source_age_sec < 0.0
            or freshness.max_receive_age_sec < 0.0):
        return CheckResult(False, 'STALE_SENSOR')
    source_age = freshness.now_ros_sec - snapshot.stamp_sec
    receive_age = freshness.now_monotonic_sec - snapshot.received_monotonic_sec
    if (source_age < 0.0 or receive_age < 0.0
            or source_age > freshness.max_source_age_sec
            or receive_age > freshness.max_receive_age_sec):
        return CheckResult(False, 'STALE_SENSOR')
    if not freshness.tf_valid:
        return CheckResult(False, 'TF_INVALID')
    return CheckResult(True, 'OK')


def check_swept_path(
    snapshot: CostmapSnapshot,
    footprint: Sequence[Point],
    poses: Sequence[Pose2D],
    safety_margin: float = 0.0,
    *,
    deadline_monotonic: Optional[float] = None,
    max_pose_samples: int = 4096,
) -> CheckResult:
    """Check the swept polygon along poses, including edges and interiors.

    Adjacent poses are interpolated in position and explicitly unwrapped yaw.
    The interpolation interval is bounded so no footprint vertex moves more
    than half a costmap cell per sample. Each checked footprint is rasterized
    conservatively using polygon/cell intersection and distance. Half of each
    interpolation interval's vertex travel bound is added as error padding.
    Thus every continuous pose is covered by at least one checked endpoint.
    """
    if not 3 <= len(footprint) <= 64 or len(poses) < 1:
        return CheckResult(False, 'INVALID_GEOMETRY')
    if not math.isfinite(safety_margin) or safety_margin < 0.0:
        return CheckResult(False, 'INVALID_GEOMETRY')
    if any(not all(math.isfinite(v) for v in point) for point in footprint):
        return CheckResult(False, 'INVALID_GEOMETRY')
    if not _valid_polygon(footprint):
        return CheckResult(False, 'INVALID_GEOMETRY')
    if any(not all(math.isfinite(v) for v in (p.x, p.y, p.yaw))
           for p in poses):
        return CheckResult(False, 'INVALID_GEOMETRY')

    radius = max(math.hypot(x, y) for x, y in footprint)
    max_vertex_step = snapshot.resolution * 0.5
    sampled_poses = [(poses[0], safety_margin)]
    for start, end in zip(poses, poses[1:]):
        delta_yaw = end.yaw - start.yaw
        translation = math.hypot(end.x - start.x, end.y - start.y)
        bound = translation + radius * abs(delta_yaw)
        if not math.isfinite(bound / max_vertex_step):
            return CheckResult(False, 'COMPUTE_BUDGET_EXCEEDED')
        count = max(1, int(math.ceil(bound / max_vertex_step)))
        if len(sampled_poses) + count + 1 > max_pose_samples:
            return CheckResult(False, 'COMPUTE_BUDGET_EXCEEDED')
        interval_margin = safety_margin + bound / count * 0.5
        for index in range(count + 1):
            fraction = index / count
            sampled_poses.append((Pose2D(
                start.x + (end.x - start.x) * fraction,
                start.y + (end.y - start.y) * fraction,
                start.yaw + delta_yaw * fraction,
            ), interval_margin))

    min_cost = NO_INFORMATION
    max_cost = FREE_SPACE
    inflation_sum = 0
    touched = 0
    for pose, margin in sampled_poses:
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            return CheckResult(False, 'COMPUTE_BUDGET_EXCEEDED')
        mx = math.floor((pose.x - snapshot.origin_x) / snapshot.resolution)
        my = math.floor((pose.y - snapshot.origin_y) / snapshot.resolution)
        if (0 <= mx < snapshot.width and 0 <= my < snapshot.height
                and snapshot.cost_at(mx, my) == INSCRIBED_INFLATED_OBSTACLE):
            return CheckResult(False, 'INSCRIBED_CENTER', max_cost=253,
                               blocked_cell=(pose.x, pose.y))
        polygon = _transform_polygon(footprint, pose)
        result = _check_polygon(snapshot, polygon, margin, deadline_monotonic)
        if not result.safe:
            return CheckResult(
                False, result.reason, result.min_cost, result.max_cost,
                inflation_sum + result.inflation_cost_sum,
                touched + result.samples, result.blocked_cell,
            )
        min_cost = min(min_cost, result.min_cost)
        max_cost = max(max_cost, result.max_cost)
        inflation_sum += result.inflation_cost_sum
        touched += result.samples
    return CheckResult(True, 'OK', min_cost, max_cost, inflation_sum, touched)


def check_observed_free_path(
    snapshot: ObservedFreeSpaceSnapshot,
    footprint: Sequence[Point],
    poses: Sequence[Pose2D],
    freshness: SnapshotFreshness,
    safety_margin: float = 0.0,
    *,
    deadline_monotonic: Optional[float] = None,
) -> CheckResult:
    """Require recent sensor-cleared evidence across a complete swept path.

    This is intended as an additional gate for near-field translation,
    especially reverse motion. Missing cells fail as `UNKNOWN_SPACE`; callers
    must not substitute static-map free space or an absent/short-range scan.
    """
    source = CostmapSnapshot(
        width=snapshot.width,
        height=snapshot.height,
        resolution=snapshot.resolution,
        origin_x=snapshot.origin_x,
        origin_y=snapshot.origin_y,
        costs=tuple(FREE_SPACE if seen else NO_INFORMATION
                    for seen in snapshot.observed_free),
        frame_id=snapshot.frame_id,
        stamp_sec=snapshot.stamp_sec,
        received_monotonic_sec=snapshot.received_monotonic_sec,
    )
    fresh = check_snapshot_freshness(source, freshness)
    if not fresh.safe:
        return fresh
    return check_swept_path(source, footprint, poses, safety_margin,
                            deadline_monotonic=deadline_monotonic)


def evaluate_rotation_candidates(
    snapshot: CostmapSnapshot,
    footprint: Sequence[Point],
    start: Pose2D,
    candidate_angles: Iterable[float],
    safety_margin: float,
    departure_path_factory=None,
) -> Tuple[RotationCandidate, ...]:
    """Check signed rotations and optional post-rotation departure paths.

    ``departure_path_factory(target_pose)`` may return poses from the rotated
    pose along a freshly planned initial path segment. If no departure path is
    provided, the candidate is rejected; geometry alone does not prove RPP can
    leave in the requested route.
    """
    output = []
    for angle in candidate_angles:
        if not math.isfinite(angle) or angle == 0.0:
            sweep = CheckResult(False, 'INVALID_GEOMETRY')
            departure = CheckResult(False, 'INVALID_GEOMETRY')
            output.append(RotationCandidate(
                start.yaw, angle, sweep, departure))
            continue
        target = Pose2D(start.x, start.y, start.yaw + angle)
        sweep = check_swept_path(
            snapshot, footprint, (start, target), safety_margin)
        if departure_path_factory is None:
            departure = CheckResult(False, 'NO_DEPARTURE_PATH')
        else:
            departure_poses = tuple(departure_path_factory(target))
            if len(departure_poses) < 2 or all(
                    math.hypot(p.x - target.x, p.y - target.y) < 1.0e-6
                    for p in departure_poses):
                departure = CheckResult(False, 'NO_DEPARTURE_PATH')
            elif (math.hypot(departure_poses[0].x - target.x,
                             departure_poses[0].y - target.y) > 1.0e-6
                  or abs(departure_poses[0].yaw - target.yaw) > 1.0e-6):
                departure = CheckResult(False, 'DEPARTURE_DISCONTINUITY')
            else:
                departure = check_swept_path(
                    snapshot, footprint, departure_poses, safety_margin)
        output.append(RotationCandidate(target.yaw, angle, sweep, departure))
    return tuple(output)


def rank_rotation_candidates(
    candidates: Iterable[RotationCandidate],
) -> Tuple[RotationCandidate, ...]:
    """Put safe candidates first, then prefer less inflated/shorter sweeps.

    This deterministic geometry-only ordering is not a navigation policy:
    route alignment and replanned departure feasibility must be supplied by
    the integrating evaluator before any future motion action is authorized.
    """
    return tuple(sorted(
        candidates,
        key=lambda candidate: (
            not candidate.safe,
            candidate.sweep.inflation_cost_sum
            + candidate.departure.inflation_cost_sum,
            abs(candidate.delta_yaw),
        ),
    ))


def _check_polygon(
    snapshot: CostmapSnapshot,
    polygon: Sequence[Point],
    safety_margin: float,
    deadline_monotonic: Optional[float] = None,
) -> CheckResult:
    half_cell_diagonal = snapshot.resolution * math.sqrt(2.0) * 0.5
    threshold = safety_margin + half_cell_diagonal
    min_x = min(x for x, _ in polygon)
    max_x = max(x for x, _ in polygon)
    min_y = min(y for _, y in polygon)
    max_y = max(y for _, y in polygon)
    map_max_x = snapshot.origin_x + snapshot.width * snapshot.resolution
    map_max_y = snapshot.origin_y + snapshot.height * snapshot.resolution
    if (min_x - safety_margin < snapshot.origin_x
            or min_y - safety_margin < snapshot.origin_y
            or max_x + safety_margin > map_max_x
            or max_y + safety_margin > map_max_y):
        return CheckResult(False, 'MAP_BOUNDARY')

    first_x = max(0, int(math.floor(
        (min_x - threshold - snapshot.origin_x) / snapshot.resolution)))
    last_x = min(snapshot.width - 1, int(math.floor(
        (max_x + threshold - snapshot.origin_x) / snapshot.resolution)))
    first_y = max(0, int(math.floor(
        (min_y - threshold - snapshot.origin_y) / snapshot.resolution)))
    last_y = min(snapshot.height - 1, int(math.floor(
        (max_y + threshold - snapshot.origin_y) / snapshot.resolution)))
    min_cost = NO_INFORMATION
    max_cost = FREE_SPACE
    inflation_sum = 0
    samples = 0
    for my in range(first_y, last_y + 1):
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            return CheckResult(False, 'COMPUTE_BUDGET_EXCEEDED')
        cy = snapshot.origin_y + (my + 0.5) * snapshot.resolution
        for mx in range(first_x, last_x + 1):
            cx = snapshot.origin_x + (mx + 0.5) * snapshot.resolution
            if _distance_point_to_polygon((cx, cy), polygon) > threshold:
                continue
            cost = snapshot.cost_at(mx, my)
            if cost and _polygon_cell_distance(
                    polygon, cx, cy,
                    snapshot.resolution * 0.5) > safety_margin:
                continue
            samples += 1
            min_cost = min(min_cost, cost)
            max_cost = max(max_cost, cost)
            if cost == NO_INFORMATION:
                return CheckResult(False, 'UNKNOWN_SPACE', min_cost, max_cost,
                                   inflation_sum, samples, (cx, cy))
            if cost == LETHAL_OBSTACLE:
                return CheckResult(False, 'LETHAL_OBSTACLE',
                                   min_cost, max_cost, inflation_sum, samples,
                                   (cx, cy))
            if cost > FREE_SPACE:
                inflation_sum += cost
    if samples == 0:
        return CheckResult(False, 'MAP_BOUNDARY')
    return CheckResult(True, 'OK', min_cost, max_cost, inflation_sum, samples)


def _distance_point_to_polygon(
    point: Point,
    polygon: Sequence[Point],
) -> float:
    if _point_in_polygon(point, polygon):
        return 0.0
    return min(_point_segment_distance(point, polygon[i],
                                       polygon[(i + 1) % len(polygon)])
               for i in range(len(polygon)))


def _segments_intersect(a, b, c, d):
    def cross(p, q, r):
        return ((q[0] - p[0]) * (r[1] - p[1])
                - (q[1] - p[1]) * (r[0] - p[0]))

    ab_c, ab_d = cross(a, b, c), cross(a, b, d)
    cd_a, cd_b = cross(c, d, a), cross(c, d, b)
    if ab_c * ab_d < 0.0 and cd_a * cd_b < 0.0:
        return True
    return any(abs(value) <= 1.0e-12
               and min(p[0], q[0]) - 1.0e-12 <= r[0]
               <= max(p[0], q[0]) + 1.0e-12
               and min(p[1], q[1]) - 1.0e-12 <= r[1]
               <= max(p[1], q[1]) + 1.0e-12
               for value, p, q, r in (
                   (ab_c, a, b, c), (ab_d, a, b, d),
                   (cd_a, c, d, a), (cd_b, c, d, b)))


def _valid_polygon(polygon):
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    if any(math.dist(a, b) < 1.0e-9 for a, b in edges):
        return False
    if abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in edges)) < 1.0e-10:
        return False
    for i, (a, b) in enumerate(edges):
        for j, (c, d) in enumerate(edges[i + 1:], i + 1):
            if j == i + 1 or (i == 0 and j == len(edges) - 1):
                continue
            if _segments_intersect(a, b, c, d):
                return False
    return _distance_point_to_polygon((0.0, 0.0), polygon) < 1.0e-9


def _polygon_cell_distance(polygon, cx, cy, half_size):
    corners = ((cx - half_size, cy - half_size),
               (cx + half_size, cy - half_size),
               (cx + half_size, cy + half_size),
               (cx - half_size, cy + half_size))
    if (any(abs(x - cx) <= half_size and abs(y - cy) <= half_size
            for x, y in polygon)
            or any(_point_in_polygon(corner, polygon) for corner in corners)):
        return 0.0
    cell_edges = tuple(zip(corners, corners[1:] + corners[:1]))
    polygon_edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
    distance = math.inf
    for a, b in polygon_edges:
        for c, d in cell_edges:
            if _segments_intersect(a, b, c, d):
                return 0.0
            distance = min(distance, _point_segment_distance(a, c, d),
                           _point_segment_distance(b, c, d),
                           _point_segment_distance(c, a, b),
                           _point_segment_distance(d, a, b))
    return distance


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if ((y1 > y) != (y2 > y)
                and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
        previous = current
    return inside


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.dist(point, start)
    fraction = max(0.0, min(1.0,
                            ((point[0] - start[0]) * dx
                             + (point[1] - start[1]) * dy)
                            / length_squared))
    closest = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.dist(point, closest)


def _transform_polygon(
    polygon: Sequence[Point],
    pose: Pose2D,
) -> Tuple[Point, ...]:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return tuple((pose.x + x * cosine - y * sine,
                  pose.y + x * sine + y * cosine)
                 for x, y in polygon)


def _shortest_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
