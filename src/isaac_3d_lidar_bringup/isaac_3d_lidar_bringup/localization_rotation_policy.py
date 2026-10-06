"""
Read-only in-place rotation safety evaluation for Issue #13 (PR2).

Evidence lives in a local odom-frame grid built from one LaserScan.  A cell
is OBSERVED_FREE only when finite rays establish it (``scan_visibility``),
OBSERVED_OCCUPIED when a finite return or an obstacle point falls in it, and
UNKNOWN otherwise.  Static-map free space is never an input, 3D points only
add obstacles, and range_min/blind-zone cells stay unknown.

The swept region covers the commanded rotation plus the braking extension,
sampled so that no footprint vertex moves more than half a cell between
samples.  Cells completely inside the current body form the self mask; the
padding ring around the body is still checked.

This module never commands motion.
"""

from dataclasses import dataclass
import hashlib
import json
import math

from carbot_nav_recovery.sensor_visibility import scan_visibility
from carbot_nav_recovery.swept_footprint import Pose2D

from .localization_contracts import (
    attestation_covers,
    ContractError,
    DEFAULT_MAX_PROBE_SPEED_RAD_S,
    MAX_PROBE_ANGLE_RAD,
    profile_permits_motion,
    REJECT_REASON_TEXT,
    RejectReason,
    RotationDecision,
    SCHEMA_VERSION,
)


FREE = 'OBSERVED_FREE'
OCCUPIED = 'OBSERVED_OCCUPIED'
UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class SweepEvidence:
    """Row-major odom-frame evidence grid with its source scan stamp."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    observed_free: tuple
    occupied: frozenset
    frame_id: str
    stamp_ns: int

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0:
            raise ContractError('evidence dimensions must be positive')
        for name in ('resolution', 'origin_x', 'origin_y'):
            if not math.isfinite(getattr(self, name)):
                raise ContractError(f'{name} must be finite')
        if self.resolution <= 0.0:
            raise ContractError('resolution must be positive')
        object.__setattr__(self, 'observed_free', tuple(self.observed_free))
        object.__setattr__(self, 'occupied', frozenset(self.occupied))
        if len(self.observed_free) != self.width * self.height:
            raise ContractError('observed_free length does not match grid')
        if any(type(value) is not bool for value in self.observed_free):
            raise ContractError('observed_free cells must be booleans')
        size = self.width * self.height
        if any(not 0 <= index < size for index in self.occupied):
            raise ContractError('occupied index outside the grid')
        if any(self.observed_free[index] for index in self.occupied):
            raise ContractError('a cell cannot be both free and occupied')
        if not self.frame_id:
            raise ContractError('frame_id must not be empty')
        if self.stamp_ns <= 0:
            raise ContractError('evidence needs a source stamp')

    def cell(self, x, y):
        """Return integer cell coordinates (possibly outside the grid)."""
        return (math.floor((x - self.origin_x) / self.resolution),
                math.floor((y - self.origin_y) / self.resolution))

    def center(self, mx, my):
        return (self.origin_x + (mx + 0.5) * self.resolution,
                self.origin_y + (my + 0.5) * self.resolution)

    def state(self, mx, my):
        """Cells outside the evidence window are unknown, never free."""
        if not (0 <= mx < self.width and 0 <= my < self.height):
            return UNKNOWN
        index = my * self.width + mx
        if index in self.occupied:
            return OCCUPIED
        return FREE if self.observed_free[index] else UNKNOWN


def _local_grid(center_x, center_y, half_extent_m, resolution, frame_id):
    if not (math.isfinite(half_extent_m) and half_extent_m > 0.0
            and math.isfinite(resolution) and resolution > 0.0):
        raise ContractError('invalid evidence window')
    cells = max(1, math.ceil(2.0 * half_extent_m / resolution))
    return _Grid(cells, cells, resolution,
                 center_x - cells * resolution / 2.0,
                 center_y - cells * resolution / 2.0, frame_id)


@dataclass(frozen=True)
class _Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str


def evidence_from_scan(scan, sensor_pose, center, *, half_extent_m=1.0,
                       resolution=0.05, frame_id='odom', received_mono=0.0,
                       deadline=None):
    """
    Build evidence from one LaserScan placed at ``sensor_pose`` (odom).

    ``center`` is the odom point the window is centred on (the robot).
    Free cells come only from ``scan_visibility``; every finite return in
    [range_min, range_max] marks its endpoint cell occupied.
    """
    grid = _local_grid(center[0], center[1], half_extent_m, resolution,
                       frame_id)
    pose = Pose2D(sensor_pose.x, sensor_pose.y, sensor_pose.yaw)
    try:
        observed = list(scan_visibility(
            scan, pose, grid, received_mono, deadline=deadline).observed_free)
        # scan_visibility refuses cells whose corners straddle the scan
        # seam (angle_min/angle_max).  For a full-circle scan, a copy
        # indexed from the opposite side has its seam elsewhere; a cell is
        # free when either copy's real rays establish it.
        rotated = _rotated_full_circle(scan)
        if rotated is not None:
            other = scan_visibility(rotated, pose, grid, received_mono,
                                    deadline=deadline).observed_free
            observed = [a or b for a, b in zip(observed, other)]
    except ValueError as error:
        raise ContractError(str(error)) from error
    occupied = set()
    for index, distance in enumerate(scan.ranges):
        if not (math.isfinite(distance)
                and scan.range_min <= distance <= scan.range_max):
            continue
        angle = pose.yaw + scan.angle_min + index * scan.angle_increment
        mx, my = (
            math.floor((pose.x + distance * math.cos(angle) - grid.origin_x)
                       / resolution),
            math.floor((pose.y + distance * math.sin(angle) - grid.origin_y)
                       / resolution))
        if 0 <= mx < grid.width and 0 <= my < grid.height:
            occupied.add(my * grid.width + mx)
    free = tuple(seen and index not in occupied
                 for index, seen in enumerate(observed))
    stamp = scan.header.stamp
    return SweepEvidence(
        grid.width, grid.height, resolution, grid.origin_x, grid.origin_y,
        free, frozenset(occupied), frame_id,
        stamp.sec * 1_000_000_000 + stamp.nanosec)


def _rotated_full_circle(scan):
    """Return the same rays indexed from the opposite side, or None."""
    count = len(scan.ranges)
    if abs(count * scan.angle_increment - 2.0 * math.pi) > (
            1.5 * scan.angle_increment):
        return None
    half = count // 2
    return _Scan(scan.header, scan.angle_min + half * scan.angle_increment,
                 scan.angle_increment, scan.range_min, scan.range_max,
                 tuple(scan.ranges[half:]) + tuple(scan.ranges[:half]))


@dataclass(frozen=True)
class _Scan:
    header: object
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple


def add_obstacle_points(evidence, points, z_min, z_max):
    """
    Add 3D points inside the collision height band as obstacles.

    Points are obstacle evidence only: empty space between sparse returns
    is not free, and points outside the band neither clear nor block.
    """
    occupied = set(evidence.occupied)
    free = list(evidence.observed_free)
    for x, y, z in points:
        if not all(math.isfinite(v) for v in (x, y, z)):
            continue
        if not z_min <= z <= z_max:
            continue
        mx, my = evidence.cell(x, y)
        if 0 <= mx < evidence.width and 0 <= my < evidence.height:
            index = my * evidence.width + mx
            occupied.add(index)
            free[index] = False
    return SweepEvidence(
        evidence.width, evidence.height, evidence.resolution,
        evidence.origin_x, evidence.origin_y, tuple(free),
        frozenset(occupied), evidence.frame_id, evidence.stamp_ns)


def footprint_geometry_hash(footprint, padding_m):
    """Stable hash binding a motion profile to footprint and padding."""
    data = {'footprint': [[float(x), float(y)] for x, y in footprint],
            'padding_m': float(padding_m)}
    text = json.dumps(data, separators=(',', ':'), sort_keys=True)
    return hashlib.sha256(text.encode()).hexdigest()


@dataclass(frozen=True)
class RotationGateConfig:
    """Geometry settings; braking values apply only without a profile."""

    padding_m: float = 0.05
    yaw_margin_rad: float = math.radians(5.0)
    speed_rad_s: float = DEFAULT_MAX_PROBE_SPEED_RAD_S
    # Used only while no motion profile carries measured values.  The
    # 2026-09 real stop tails were 0.57-0.92 s, i.e. 0.23-0.37 rad at the
    # 0.40 rad/s probe speed.
    unmeasured_braking_rad: float = 0.40
    unmeasured_center_drift_m: float = 0.02
    # If the guard process dies, the chassis keeps the last command until
    # its own watchdog.  Measured 2026-10-06: wheels stopped 0.49-0.57 s
    # after the last command (canonical cmd_vel_timeout_s 0.50).
    chassis_watchdog_s: float = 0.60

    def __post_init__(self):
        for name in ('padding_m', 'yaw_margin_rad', 'unmeasured_braking_rad',
                     'unmeasured_center_drift_m', 'chassis_watchdog_s'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value >= 0.0):
                raise ContractError(f'{name} must be finite and >= 0')
        if not (math.isfinite(self.speed_rad_s) and self.speed_rad_s > 0.0):
            raise ContractError('speed_rad_s must be positive')


def braking_extension(profile, config):
    """
    Return ``(extra_rad, center_drift_m, measured)`` past the target.

    Normally the guard zeroes after the profile's stop latency.  If the
    guard itself dies, the last command runs until the chassis watchdog
    instead, so the sweep covers the longer of the two holds at the
    commanded rate, plus the stop tail and the yaw margin.
    """
    measured = profile is not None and None not in (
        profile.stop_tail_rad, profile.center_drift_m, profile.latency_s)
    if measured:
        latency, tail, drift = (profile.latency_s, profile.stop_tail_rad,
                                profile.center_drift_m)
    else:
        latency, tail, drift = (0.0, config.unmeasured_braking_rad,
                                config.unmeasured_center_drift_m)
    hold = max(latency, config.chassis_watchdog_s)
    return (config.speed_rad_s * hold + tail + config.yaw_margin_rad,
            drift, measured)


def _transform(footprint, x, y, yaw):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple((x + cosine * px - sine * py, y + sine * px + cosine * py)
                 for px, py in footprint)


def _inside(point, polygon):
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y):
            if x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
                inside = not inside
    return inside


def _segment_distance(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length = dx * dx + dy * dy
    t = 0.0 if length == 0.0 else max(0.0, min(1.0, (
        (px - ax) * dx + (py - ay) * dy) / length))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _polygon_distance(point, polygon):
    if _inside(point, polygon):
        return 0.0
    return min(_segment_distance(point, polygon[i],
                                 polygon[(i + 1) % len(polygon)])
               for i in range(len(polygon)))


def _valid_footprint(footprint):
    return (3 <= len(footprint) <= 64
            and all(len(p) == 2 and all(math.isfinite(v) for v in p)
                    for p in footprint))


def sweep_polygons(footprint, pose, swept_angle, resolution):
    """
    Footprints along the rotation and the per-sample error margin.

    Samples are spaced so the outermost vertex moves at most half a cell;
    half of that spacing is the margin that covers poses between samples.
    """
    radius = max(math.hypot(x, y) for x, y in footprint)
    count = max(1, math.ceil(abs(swept_angle) * radius / (resolution * 0.5)))
    margin = radius * abs(swept_angle) / count * 0.5
    polygons = tuple(
        _transform(footprint, pose.x, pose.y,
                   pose.yaw + swept_angle * index / count)
        for index in range(count + 1))
    return polygons, margin


def classify_sweep(evidence, footprint, pose, swept_angle, padding_m):
    """
    Return ``(cells_by_state, self_cells)`` for one swept rotation.

    A cell is in the sweep when its centre is within padding, the sampling
    margin and half a cell diagonal of any sampled footprint, which is a
    superset of the cells the padded swept area touches.  Cells whose four
    corners all lie inside the current (unpadded) body are the self mask.
    """
    resolution = evidence.resolution
    polygons, margin = sweep_polygons(footprint, pose, swept_angle,
                                      resolution)
    threshold = padding_m + margin + resolution * math.sqrt(2.0) / 2.0
    reach = max(math.hypot(x, y) for x, y in footprint) + threshold
    first_x, first_y = evidence.cell(pose.x - reach, pose.y - reach)
    last_x, last_y = evidence.cell(pose.x + reach, pose.y + reach)
    body = polygons[0]
    half = resolution / 2.0
    cells = {FREE: [], OCCUPIED: [], UNKNOWN: []}
    self_cells = []
    for my in range(first_y, last_y + 1):
        for mx in range(first_x, last_x + 1):
            cx, cy = evidence.center(mx, my)
            if all(_polygon_distance((cx, cy), polygon) > threshold
                   for polygon in polygons):
                continue
            corners = ((cx - half, cy - half), (cx - half, cy + half),
                       (cx + half, cy - half), (cx + half, cy + half))
            if all(_inside(corner, body) for corner in corners):
                self_cells.append((mx, my))
                continue
            cells[evidence.state(mx, my)].append((mx, my))
    return cells, self_cells


def _min_clearance(evidence, footprint, pose, swept_angle):
    if not evidence.occupied:
        return None
    polygons, _ = sweep_polygons(footprint, pose, swept_angle,
                                 evidence.resolution)
    half_diagonal = evidence.resolution * math.sqrt(2.0) / 2.0
    best = math.inf
    for index in evidence.occupied:
        point = evidence.center(index % evidence.width,
                                index // evidence.width)
        for polygon in polygons:
            best = min(best, _polygon_distance(point, polygon))
    return max(0.0, best - half_diagonal)


@dataclass(frozen=True)
class RotationEvaluation:
    """A RotationDecision plus the cell evidence behind it."""

    decision: RotationDecision
    swept_angle_rad: float
    braking_rad: float
    braking_measured: bool
    cells: dict
    attested: tuple
    self_cell_count: int


def evaluate_localization_rotation(
        evidence, footprint, pose, delta_yaw, *, profile=None, hashes=None,
        config=RotationGateConfig(), attestation=None, session=None,
        now_mono=None):
    """
    Read-only verdict for one in-place rotation of ``delta_yaw`` from ``pose``.

    Order: any observed obstacle in the sweep -> OBSTACLE_IN_SWEEP; any
    unobserved cell not covered by a valid RotationAttestation ->
    UNKNOWN_SWEEP; a profile that does not permit motion (missing, not
    ACCEPTED, or hashes that do not match ``hashes`` =
    (geometry, extrinsics, control_chain)) -> PROFILE_INVALID.
    """
    if not _valid_footprint(footprint):
        raise ContractError('invalid footprint')
    if not (math.isfinite(delta_yaw) and delta_yaw != 0.0
            and abs(delta_yaw) <= MAX_PROBE_ANGLE_RAD + 1e-9):
        raise ContractError('delta_yaw must be nonzero and within pi/2')
    braking, drift, measured = braking_extension(profile, config)
    swept = delta_yaw + math.copysign(braking, delta_yaw)
    cells, self_cells = classify_sweep(
        evidence, footprint, pose, swept, config.padding_m + drift)
    covered = attestation_covers(attestation, session, pose, now_mono)
    attested = tuple(cells[UNKNOWN]) if covered else ()
    unknown = 0 if covered else len(cells[UNKNOWN])
    if hashes is None:
        permitted = False
    else:
        permitted, _reason = profile_permits_motion(profile, *hashes)
    if cells[OCCUPIED]:
        reason = RejectReason.OBSTACLE_IN_SWEEP.value
    elif unknown:
        reason = RejectReason.UNKNOWN_SWEEP.value
    elif not permitted:
        reason = RejectReason.PROFILE_INVALID.value
    else:
        reason = ''
    decision = RotationDecision(
        allowed=not reason, delta_yaw=delta_yaw, reason=reason,
        min_clearance_m=_min_clearance(evidence, footprint, pose, swept),
        unknown_cells=unknown, snapshot_stamp_ns=evidence.stamp_ns,
        attested_cells=len(attested))
    return RotationEvaluation(decision, swept, braking, measured, cells,
                              attested, len(self_cells))


def preview_payload(evaluations, *, geometry_hash, profile_state):
    """JSON-ready summary of probe evaluations for the read-only preview."""
    probes = []
    for item in evaluations:
        decision = item.decision
        reason = decision.reason
        probes.append({
            'delta_yaw_rad': decision.delta_yaw,
            'allowed': decision.allowed,
            'reason': reason,
            'reason_text': (REJECT_REASON_TEXT[RejectReason(reason)][0]
                            if reason else ''),
            'swept_angle_rad': item.swept_angle_rad,
            'braking_rad': item.braking_rad,
            'braking_measured': item.braking_measured,
            'observed_free_cells': len(item.cells[FREE]),
            'occupied_cells': len(item.cells[OCCUPIED]),
            'unknown_cells': decision.unknown_cells,
            'attested_cells': decision.attested_cells,
            'self_mask_cells': item.self_cell_count,
            'min_clearance_m': decision.min_clearance_m,
        })
    stamps = {item.decision.snapshot_stamp_ns for item in evaluations}
    return {
        'schema_version': SCHEMA_VERSION,
        'snapshot_stamp_ns': max(stamps) if stamps else None,
        'geometry_hash': geometry_hash,
        'profile_state': profile_state,
        'motion_commanded': False,
        'probes': probes,
    }


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RotationProgress:
    """
    Continuous yaw bookkeeping for one probe segment.

    Yaw is unwrapped sample to sample, so +pi/-pi crossings are continuous.
    ``signed_progress`` is the net change since the segment started; it
    decides completion.  ``abs_travel`` accumulates |d yaw| including noise
    and reversals; it is what the motion budget is charged.  Positive noise
    alone therefore never completes a segment, and back-and-forth jitter
    still spends budget.
    """

    def __init__(self, start_yaw):
        if not math.isfinite(start_yaw):
            raise ContractError('start yaw must be finite')
        self._last = start_yaw
        self.signed_progress = 0.0
        self.abs_travel = 0.0

    def update(self, yaw):
        if not math.isfinite(yaw):
            raise ContractError('yaw must be finite')
        step = _wrap(yaw - self._last)
        self._last = yaw
        self.signed_progress += step
        self.abs_travel += abs(step)
        return step


def predicted_stop_angle(measured_rate, latency_s, stop_tail_rad,
                         yaw_margin_rad):
    """Angle still travelled after a zero command: rate*latency + tail."""
    return (abs(measured_rate) * latency_s + stop_tail_rad
            + yaw_margin_rad)


@dataclass(frozen=True)
class ProbeBudgetLimits:
    """Session limits from spec 6.3; all monotonic."""

    max_segments: int = 6
    max_total_abs_yaw_rad: float = 2.0 * math.pi
    max_motion_time_s: float = 45.0
    max_session_s: float = 240.0

    def __post_init__(self):
        if isinstance(self.max_segments, bool) or self.max_segments < 1:
            raise ContractError('max_segments must be a positive integer')
        for name in ('max_total_abs_yaw_rad', 'max_motion_time_s',
                     'max_session_s'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ContractError(f'{name} must be positive')
        if self.max_total_abs_yaw_rad > 2.0 * math.pi + 1e-9:
            raise ContractError('max_total_abs_yaw_rad cannot exceed 2*pi')


class ProbeBudget:
    """
    Spend-only budget for one localization session.

    Segments, absolute yaw and nonzero-command time only grow.  A session
    started at ``started_mono`` never resets; a new session needs a new
    budget object (and, at the guard, a fresh STOP handshake).
    """

    def __init__(self, limits, started_mono):
        if not math.isfinite(started_mono):
            raise ContractError('started_mono must be finite')
        self.limits = limits
        self.started_mono = started_mono
        self.segments = 0
        self.abs_yaw = 0.0
        self.motion_time = 0.0

    def admit(self, delta_yaw, now_mono):
        """Return '' if a new segment of ``delta_yaw`` fits, else a reason."""
        limits = self.limits
        if not math.isfinite(now_mono) or now_mono < self.started_mono:
            return RejectReason.MOTION_BUDGET_EXHAUSTED.value
        if (self.segments >= limits.max_segments
                or self.abs_yaw + abs(delta_yaw)
                > limits.max_total_abs_yaw_rad + 1e-9
                or self.motion_time >= limits.max_motion_time_s
                or now_mono - self.started_mono >= limits.max_session_s):
            return RejectReason.MOTION_BUDGET_EXHAUSTED.value
        return ''

    def start_segment(self):
        self.segments += 1

    def charge(self, abs_yaw_step, motion_dt):
        if abs_yaw_step < 0.0 or motion_dt < 0.0:
            raise ContractError('budget charges are non-negative')
        self.abs_yaw += abs_yaw_step
        self.motion_time += motion_dt

    def exhausted(self, now_mono):
        limits = self.limits
        return (self.abs_yaw > limits.max_total_abs_yaw_rad + 1e-9
                or self.motion_time >= limits.max_motion_time_s
                or not math.isfinite(now_mono)
                or now_mono < self.started_mono
                or now_mono - self.started_mono >= limits.max_session_s)


# Most specific first: an observed obstacle outranks missing evidence.
_REFUSAL_PRIORITY = (RejectReason.OBSTACLE_IN_SWEEP.value,
                     RejectReason.UNKNOWN_SWEEP.value,
                     RejectReason.PROFILE_INVALID.value)


def choose_probe(decisions, view_headings, current_heading,
                 min_view_separation_rad=math.radians(20.0)):
    """
    Pick the next probe rotation, or explain why there is none.

    Only allowed decisions are candidates.  A probe whose resulting heading
    lies within ``min_view_separation_rad`` of an already collected view
    adds no new view and is skipped.  Among the rest the largest distance
    to every collected view wins; ties prefer the smaller rotation.
    Returns ``(decision, '')`` or ``(None, reason)``.
    """
    allowed = [item for item in decisions if item.allowed]
    if not allowed:
        reasons = {item.reason for item in decisions}
        for reason in _REFUSAL_PRIORITY:
            if reason in reasons:
                return None, reason
        return None, (sorted(reasons)[0] if reasons
                      else RejectReason.UNKNOWN_SWEEP.value)
    best = None
    for item in allowed:
        heading = current_heading + item.delta_yaw
        novelty = min((abs(_wrap(heading - seen)) for seen in view_headings),
                      default=math.pi)
        if novelty < min_view_separation_rad:
            continue
        key = (novelty, -abs(item.delta_yaw))
        if best is None or key > best[0]:
            best = (key, item)
    if best is None:
        return None, RejectReason.AMBIGUOUS_LOCATION.value
    return best[1], ''
