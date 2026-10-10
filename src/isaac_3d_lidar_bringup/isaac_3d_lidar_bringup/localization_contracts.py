"""
Versioned contracts for confined-space automatic localization (Issue #13).

Pure Python: no ROS imports, so the manager, the motion guard, offline
analysis scripts and tests share one definition.  Every decoder is strict:
unknown fields, missing fields, wrong types, NaN/Infinity and unknown schema
versions are rejected instead of being defaulted.  A missing measurement is
``None``; it is never replaced by ``0``.
"""

from dataclasses import dataclass
from enum import Enum
import json
import math


SCHEMA_VERSION = 1
MAX_SESSION_LENGTH = 64
# Initial command ceiling from the issue specification; not a safety rating.
DEFAULT_MAX_PROBE_SPEED_RAD_S = 0.40
MAX_PROBE_ANGLE_RAD = math.pi / 2.0


class ContractError(ValueError):
    """Raised when data violates a v1 confined-localization contract."""


class Strategy(str, Enum):
    """Startup localization strategy selected once at launch."""

    LEGACY_FULL_ROTATION = 'legacy_full_rotation'
    STATIONARY_ONLY = 'stationary_only'
    SEGMENTED_ROTATION = 'segmented_rotation'


class MotionPolicy(str, Enum):
    """Whether a new strategy may command any startup rotation."""

    FORBID = 'forbid'
    GUARDED = 'guarded'


class FrameRole(str, Enum):
    """Independent use of a keyframe inside one decision."""

    TRAIN = 'TRAIN'
    HOLDOUT = 'HOLDOUT'


class RejectReason(str, Enum):
    """Explicit refusal reasons; at least the v1 specification set."""

    SEARCH_INCOMPLETE = 'SEARCH_INCOMPLETE'
    AMBIGUOUS_LOCATION = 'AMBIGUOUS_LOCATION'
    UNOBSERVABLE_AXIS = 'UNOBSERVABLE_AXIS'
    TF_AT_SOURCE_MISSING = 'TF_AT_SOURCE_MISSING'
    SENSOR_STALE = 'SENSOR_STALE'
    MAP_CHANGED = 'MAP_CHANGED'
    UNKNOWN_SWEEP = 'UNKNOWN_SWEEP'
    OBSTACLE_IN_SWEEP = 'OBSTACLE_IN_SWEEP'
    PROFILE_INVALID = 'PROFILE_INVALID'
    CONTROL_CONFLICT = 'CONTROL_CONFLICT'
    ODOM_JUMP = 'ODOM_JUMP'
    MOTION_BUDGET_EXHAUSTED = 'MOTION_BUDGET_EXHAUSTED'
    CANCELED = 'CANCELED'
    NO_VALID_CANDIDATE = 'NO_VALID_CANDIDATE'
    LOCALIZATION_FAULT = 'LOCALIZATION_FAULT'
    CHASSIS_BLOCKED = 'CHASSIS_BLOCKED'


# Operator-facing text.  The flag says whether a manual 2D Pose remains a
# valid way forward without restarting the stack.
REJECT_REASON_TEXT = {
    RejectReason.SEARCH_INCOMPLETE: ('全图搜索未完成，不能接受暂时第一名', True),
    RejectReason.AMBIGUOUS_LOCATION: ('存在多个相近位置/朝向候选，无法确定唯一位置', True),
    RejectReason.UNOBSERVABLE_AXIS: ('环境在某个方向上退化（如长走廊），该方向不可观测', True),
    RejectReason.TF_AT_SOURCE_MISSING: ('缺少传感器源时间对应的坐标变换', True),
    RejectReason.SENSOR_STALE: ('传感器数据过期', True),
    RejectReason.MAP_CHANGED: ('地图已变更，搜索结果作废', True),
    RejectReason.UNKNOWN_SWEEP: ('旋转扫掠区域存在未观测（未知）区域，不能证明安全', True),
    RejectReason.OBSTACLE_IN_SWEEP: ('旋转扫掠区域内有障碍', True),
    RejectReason.PROFILE_INVALID: ('运动标定配置缺失、未验收或与当前车辆不匹配', True),
    RejectReason.CONTROL_CONFLICT: ('存在其他速度控制源或控制权冲突', False),
    RejectReason.ODOM_JUMP: ('里程计跳变或车辆被搬动', True),
    RejectReason.MOTION_BUDGET_EXHAUSTED: ('旋转段数、角度或时间预算已用尽', True),
    RejectReason.CANCELED: ('定位已被取消', True),
    RejectReason.NO_VALID_CANDIDATE: ('没有候选通过独立验证帧的匹配门槛', True),
    RejectReason.LOCALIZATION_FAULT: ('定位急停已触发或状态未知，需排查并重启后才能运动', False),
    RejectReason.CHASSIS_BLOCKED: ('底盘报告运动锁止、连接中断，或底盘状态缺失/过期', True),
}


class MotionOperation(str, Enum):
    """Operations accepted by the motion guard."""

    STOP = 'STOP'
    ROTATE = 'ROTATE'
    RELEASE = 'RELEASE'


class GuardState(str, Enum):
    """Externally reported motion guard state."""

    IDLE = 'IDLE'
    STOPPING = 'STOPPING'
    STOPPED = 'STOPPED'
    ROTATING = 'ROTATING'
    RELEASED = 'RELEASED'
    FAULT_STOPPED = 'FAULT_STOPPED'


class ProfileStatus(str, Enum):
    """
    Lifecycle of a rotation motion profile.

    Automatic analysis may produce only INSUFFICIENT or ESTIMATED.  REVIEWED
    requires an external cross-check record; ACCEPTED is a separate explicit
    human decision and is never assigned by analysis code.
    """

    INSUFFICIENT = 'INSUFFICIENT'
    ESTIMATED = 'ESTIMATED'
    REVIEWED = 'REVIEWED'
    ACCEPTED = 'ACCEPTED'


AUTOMATIC_PROFILE_STATUSES = frozenset(
    (ProfileStatus.INSUFFICIENT, ProfileStatus.ESTIMATED))


# Fields added to the existing /automatic_localization/status JSON.  Existing
# fields are retained unchanged.
STATUS_EXTENSION_FIELDS = (
    'schema_version',
    'strategy',
    'motion_policy',
    'search_complete',
    'hypothesis_count',
    'ambiguity_reason',
    'motion_guard_state',
    'unknown_sweep_cells',
    'total_abs_yaw',
    'map_hash',
)


def _fail(message):
    raise ContractError(message)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(f'{name} must be a number')
    if not math.isfinite(value):
        _fail(f'{name} must be finite')
    return float(value)


def _optional_nonnegative(value, name):
    if value is None:
        return None
    value = _finite(value, name)
    if value < 0.0:
        _fail(f'{name} must be non-negative')
    return value


def _integer(value, name, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int):
        _fail(f'{name} must be an integer')
    if value < minimum:
        _fail(f'{name} must be >= {minimum}')
    return value


def _unit_interval(value, name):
    value = _finite(value, name)
    if not 0.0 <= value <= 1.0:
        _fail(f'{name} must be within [0, 1]')
    return value


def _session(value):
    if (not isinstance(value, str) or not value
            or len(value) > MAX_SESSION_LENGTH or not value.isprintable()):
        _fail('session must be a printable non-empty string of at most '
              f'{MAX_SESSION_LENGTH} characters')
    return value


def _hex_digest(value, name):
    if (not isinstance(value, str) or len(value) < 8
            or any(c not in '0123456789abcdef' for c in value)):
        _fail(f'{name} must be a lowercase hexadecimal digest')
    return value


def _enum(enum_type, value, name):
    try:
        return enum_type(value)
    except ValueError:
        _fail(f'{name} has unsupported value {value!r}')


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class SE2:
    """Planar rigid transform; ``T_A_B`` maps coordinates from B into A."""

    x: float
    y: float
    yaw: float

    def __post_init__(self):
        for name in ('x', 'y', 'yaw'):
            _finite(getattr(self, name), name)

    def compose(self, other):
        """Return ``self * other``."""
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(
            self.x + cosine * other.x - sine * other.y,
            self.y + sine * other.x + cosine * other.y,
            normalize_angle(self.yaw + other.yaw),
        )

    def inverse(self):
        """Return the inverse transform."""
        cosine, sine = math.cos(self.yaw), math.sin(self.yaw)
        return SE2(
            -cosine * self.x - sine * self.y,
            sine * self.x - cosine * self.y,
            normalize_angle(-self.yaw),
        )


@dataclass(frozen=True)
class Keyframe:
    """One stopped localization scan with its source-time transforms."""

    id: int
    session: str
    stamp_ns: int
    view_id: int
    scan: object
    T_odom_base: SE2
    T_base_scan: SE2
    receipt_mono: float
    role: FrameRole

    def __post_init__(self):
        _integer(self.id, 'id')
        _session(self.session)
        _integer(self.stamp_ns, 'stamp_ns', minimum=1)
        _integer(self.view_id, 'view_id')
        if self.scan is None:
            _fail('scan is required')
        for name in ('T_odom_base', 'T_base_scan'):
            if not isinstance(getattr(self, name), SE2):
                _fail(f'{name} must be SE2')
        _finite(self.receipt_mono, 'receipt_mono')
        object.__setattr__(
            self, 'role', _enum(FrameRole, self.role, 'role'))


@dataclass(frozen=True)
class Hypothesis:
    """A clustered map->base pose candidate at the reference keyframe."""

    x: float
    y: float
    yaw: float
    score: float
    coverage: float
    conflict: float
    cluster_id: int
    per_view: tuple
    support_bounds: tuple

    def __post_init__(self):
        _finite(self.x, 'x')
        _finite(self.y, 'y')
        yaw = _finite(self.yaw, 'yaw')
        if not -math.pi <= yaw <= math.pi:
            _fail('yaw must be normalized to [-pi, pi]')
        _unit_interval(self.score, 'score')
        _unit_interval(self.coverage, 'coverage')
        _unit_interval(self.conflict, 'conflict')
        _integer(self.cluster_id, 'cluster_id')
        if not isinstance(self.per_view, tuple):
            _fail('per_view must be a tuple')
        for index, value in enumerate(self.per_view):
            _unit_interval(value, f'per_view[{index}]')
        if (not isinstance(self.support_bounds, tuple)
                or len(self.support_bounds) not in (0, 3)):
            _fail('support_bounds must be () or (x_m, y_m, yaw_rad)')
        for index, value in enumerate(self.support_bounds):
            if _finite(value, f'support_bounds[{index}]') < 0.0:
                _fail('support_bounds must be non-negative')


@dataclass(frozen=True)
class SearchResult:
    """Outcome of one bounded multi-view search request."""

    session: str
    map_hash: str
    complete: bool
    hypotheses: tuple
    evaluated: int
    duration_s: float
    reason: str
    # Coarse seed poses (x, y, yaw) of clusters that could still compete
    # when the search stopped only because its refinement budget ran out.
    # Empty when the coarse scan itself did not finish.
    unrefined: tuple = ()

    def __post_init__(self):
        _session(self.session)
        _hex_digest(self.map_hash, 'map_hash')
        if not isinstance(self.complete, bool):
            _fail('complete must be a boolean')
        if (not isinstance(self.hypotheses, tuple)
                or not all(isinstance(h, Hypothesis)
                           for h in self.hypotheses)):
            _fail('hypotheses must be a tuple of Hypothesis')
        _integer(self.evaluated, 'evaluated')
        if _finite(self.duration_s, 'duration_s') < 0.0:
            _fail('duration_s must be non-negative')
        if self.reason:
            _enum(RejectReason, self.reason, 'reason')
        if not self.complete and not self.reason:
            # A canceled or budget-limited search can never be accepted.
            _fail('an incomplete search must carry a reject reason')
        if not isinstance(self.unrefined, tuple) or any(
                not isinstance(seed, tuple) or len(seed) != 3
                or not all(isinstance(v, float) and math.isfinite(v) for v in seed)
                for seed in self.unrefined):
            _fail('unrefined must be a tuple of finite (x, y, yaw) floats')
        if self.complete and self.unrefined:
            _fail('a complete search has no unrefined competitors')


@dataclass(frozen=True)
class RotationAttestation:
    """
    Operator statement that the startup placement can rotate in place.

    The MID-360 cannot observe most of the sweep band next to the body (PR0
    audit), so the operator's placement promise is the evidence for that
    blind zone.  It is bound to one session and to the odom pose at which it
    was given; it never covers cells the sensors actually observe occupied
    and it is never persisted across sessions.
    """

    session: str
    odom_pose: SE2
    issued_mono: float
    max_translation_m: float
    max_age_s: float

    def __post_init__(self):
        _session(self.session)
        if not isinstance(self.odom_pose, SE2):
            _fail('odom_pose must be SE2')
        _finite(self.issued_mono, 'issued_mono')
        for name in ('max_translation_m', 'max_age_s'):
            if _finite(getattr(self, name), name) <= 0.0:
                _fail(f'{name} must be positive')


def attestation_covers(attestation, session, current_odom, now_mono):
    """
    Return whether the attestation still applies to this placement.

    Rotation in place is allowed to change yaw; any translation beyond the
    bound means the robot was moved or slid and the promise no longer holds.
    """
    if attestation is None or attestation.session != session:
        return False
    age = now_mono - attestation.issued_mono
    if not 0.0 <= age <= attestation.max_age_s:
        return False
    moved = math.hypot(current_odom.x - attestation.odom_pose.x,
                       current_odom.y - attestation.odom_pose.y)
    return moved <= attestation.max_translation_m


@dataclass(frozen=True)
class RotationDecision:
    """
    Read-only verdict for one candidate in-place rotation.

    ``unknown_cells`` counts unresolved unobserved sweep cells;
    ``attested_cells`` counts unobserved cells covered by a valid
    RotationAttestation.  Observed obstacles are never attested away.
    """

    allowed: bool
    delta_yaw: float
    reason: str
    min_clearance_m: object
    unknown_cells: int
    snapshot_stamp_ns: int
    attested_cells: int = 0

    def __post_init__(self):
        if not isinstance(self.allowed, bool):
            _fail('allowed must be a boolean')
        delta = _finite(self.delta_yaw, 'delta_yaw')
        if abs(delta) > MAX_PROBE_ANGLE_RAD + 1e-9:
            _fail('delta_yaw exceeds the pi/2 probe limit')
        _optional_nonnegative(self.min_clearance_m, 'min_clearance_m')
        _integer(self.unknown_cells, 'unknown_cells')
        _integer(self.attested_cells, 'attested_cells')
        _integer(self.snapshot_stamp_ns, 'snapshot_stamp_ns')
        if self.allowed:
            if self.reason:
                _fail('an allowed rotation cannot carry a reject reason')
            if delta == 0.0:
                _fail('an allowed rotation must have nonzero delta_yaw')
            if self.unknown_cells:
                _fail('an allowed rotation cannot sweep unknown cells')
            if self.snapshot_stamp_ns <= 0:
                _fail('an allowed rotation needs a source snapshot stamp')
        else:
            _enum(RejectReason, self.reason, 'reason')


@dataclass(frozen=True)
class MotionProfile:
    """Versioned physical rotation response used by the motion guard."""

    schema_version: int
    geometry_hash: str
    extrinsics_hash: str
    control_chain_hash: str
    evidence_ids: tuple
    stop_tail_rad: object
    center_drift_m: object
    latency_s: object
    externally_reviewed: bool
    status: ProfileStatus

    def __post_init__(self):
        if (isinstance(self.schema_version, bool)
                or self.schema_version != SCHEMA_VERSION):
            _fail(f'unsupported profile schema {self.schema_version!r}')
        for name in ('geometry_hash', 'extrinsics_hash',
                     'control_chain_hash'):
            _hex_digest(getattr(self, name), name)
        if (not isinstance(self.evidence_ids, tuple)
                or not all(isinstance(item, str) and item
                           for item in self.evidence_ids)):
            _fail('evidence_ids must be a tuple of non-empty strings')
        for name in ('stop_tail_rad', 'center_drift_m', 'latency_s'):
            _optional_nonnegative(getattr(self, name), name)
        if not isinstance(self.externally_reviewed, bool):
            _fail('externally_reviewed must be a boolean')
        status = _enum(ProfileStatus, self.status, 'status')
        object.__setattr__(self, 'status', status)
        if status in (ProfileStatus.REVIEWED, ProfileStatus.ACCEPTED):
            if not self.externally_reviewed:
                _fail(f'{status.value} requires an external review record')
            if not self.evidence_ids:
                _fail(f'{status.value} requires evidence ids')
            if None in (self.stop_tail_rad, self.center_drift_m,
                        self.latency_s):
                _fail(f'{status.value} cannot contain null measurements')


def _strict_json_object(text):
    def reject_constant(value):
        _fail(f'non-finite JSON constant {value} is not allowed')

    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                _fail(f'duplicate JSON field {key!r}')
            result[key] = value
        return result

    if not isinstance(text, str):
        _fail('JSON payload must be a string')
    try:
        data = json.loads(text, parse_constant=reject_constant,
                          object_pairs_hook=unique_object)
    except json.JSONDecodeError as error:
        raise ContractError(f'invalid JSON: {error}') from error
    if not isinstance(data, dict):
        _fail('JSON payload must be an object')
    return data


def _exact_fields(data, fields):
    missing = [name for name in fields if name not in data]
    extra = sorted(set(data) - set(fields))
    if missing or extra:
        _fail(f'field mismatch: missing={missing} undeclared={extra}')


def _encode(data):
    return json.dumps(data, allow_nan=False, separators=(',', ':'),
                      sort_keys=True)


MOTION_PROFILE_FIELDS = (
    'schema_version', 'geometry_hash', 'extrinsics_hash',
    'control_chain_hash', 'evidence_ids', 'stop_tail_rad',
    'center_drift_m', 'latency_s', 'externally_reviewed', 'status',
)


def decode_motion_profile(text):
    """Parse a profile JSON document strictly."""
    data = _strict_json_object(text)
    _exact_fields(data, MOTION_PROFILE_FIELDS)
    if not isinstance(data['evidence_ids'], list):
        _fail('evidence_ids must be a list')
    data['evidence_ids'] = tuple(data['evidence_ids'])
    return MotionProfile(**data)


def encode_motion_profile(profile):
    """Serialize a profile, preserving null measurements."""
    data = {name: getattr(profile, name) for name in MOTION_PROFILE_FIELDS}
    data['evidence_ids'] = list(profile.evidence_ids)
    data['status'] = profile.status.value
    return _encode(data)


def profile_permits_motion(profile, geometry_hash, extrinsics_hash,
                           control_chain_hash):
    """Return ``(allowed, reason)`` for guarded nonzero output."""
    if profile is None:
        return False, RejectReason.PROFILE_INVALID
    if profile.status != ProfileStatus.ACCEPTED:
        return False, RejectReason.PROFILE_INVALID
    if (profile.geometry_hash != geometry_hash
            or profile.extrinsics_hash != extrinsics_hash
            or profile.control_chain_hash != control_chain_hash):
        return False, RejectReason.PROFILE_INVALID
    return True, None


@dataclass(frozen=True)
class MotionRequest:
    """Manager -> guard lease message (10 Hz refresh)."""

    schema_version: int
    session: str
    sequence: int
    operation: MotionOperation
    delta_yaw_rad: float
    speed_rad_s: float
    profile_hash: str


MOTION_REQUEST_FIELDS = (
    'schema_version', 'session', 'sequence', 'operation',
    'delta_yaw_rad', 'speed_rad_s', 'profile_hash',
)


def make_motion_request(data,
                        max_speed_rad_s=DEFAULT_MAX_PROBE_SPEED_RAD_S):
    """Validate a decoded request mapping and return a MotionRequest."""
    if not isinstance(data, dict):
        _fail('motion request must be an object')
    _exact_fields(data, MOTION_REQUEST_FIELDS)
    if (isinstance(data['schema_version'], bool)
            or data['schema_version'] != SCHEMA_VERSION):
        _fail(f'unsupported request schema {data["schema_version"]!r}')
    session = _session(data['session'])
    sequence = _integer(data['sequence'], 'sequence')
    operation = _enum(MotionOperation, data['operation'], 'operation')
    delta = _finite(data['delta_yaw_rad'], 'delta_yaw_rad')
    speed = _finite(data['speed_rad_s'], 'speed_rad_s')
    profile_hash = data['profile_hash']
    if not isinstance(profile_hash, str):
        _fail('profile_hash must be a string')
    if operation == MotionOperation.ROTATE:
        if delta == 0.0 or abs(delta) > MAX_PROBE_ANGLE_RAD + 1e-9:
            _fail('ROTATE delta_yaw_rad must be nonzero and within pi/2')
        if not 0.0 < speed <= max_speed_rad_s:
            _fail(f'ROTATE speed_rad_s must be within (0, {max_speed_rad_s}]')
        _hex_digest(profile_hash, 'profile_hash')
    elif delta != 0.0 or speed != 0.0:
        _fail(f'{operation.value} must carry zero delta and speed')
    return MotionRequest(SCHEMA_VERSION, session, sequence, operation,
                         delta, speed, profile_hash)


def decode_motion_request(text,
                          max_speed_rad_s=DEFAULT_MAX_PROBE_SPEED_RAD_S):
    """Parse a std_msgs/String motion request payload strictly."""
    return make_motion_request(_strict_json_object(text), max_speed_rad_s)


def encode_motion_request(request):
    """Serialize a validated request."""
    data = {name: getattr(request, name) for name in MOTION_REQUEST_FIELDS}
    data['operation'] = request.operation.value
    # Round-trip through the validator so invalid objects never leave.
    make_motion_request(dict(data), max_speed_rad_s=math.inf)
    return _encode(data)


@dataclass(frozen=True)
class MotionStatus:
    """Guard -> manager status (10 Hz)."""

    schema_version: int
    session: str
    sequence: int
    state: GuardState
    reason: str
    signed_progress_rad: float
    abs_travel_rad: float
    stopped: bool
    source_ages: tuple


# schema_version is added to the issue's field list so consumers can reject
# a future incompatible guard instead of misreading it.
MOTION_STATUS_FIELDS = (
    'schema_version', 'session', 'sequence', 'state', 'reason',
    'signed_progress_rad', 'abs_travel_rad', 'stopped', 'source_ages',
)


def make_motion_status(data):
    """Validate a decoded status mapping and return a MotionStatus."""
    if not isinstance(data, dict):
        _fail('motion status must be an object')
    _exact_fields(data, MOTION_STATUS_FIELDS)
    if (isinstance(data['schema_version'], bool)
            or data['schema_version'] != SCHEMA_VERSION):
        _fail(f'unsupported status schema {data["schema_version"]!r}')
    # Before the first STOP handshake the guard has no session.
    session = data['session']
    if session != '':
        _session(session)
    sequence = _integer(data['sequence'], 'sequence')
    state = _enum(GuardState, data['state'], 'state')
    reason = data['reason']
    if not isinstance(reason, str):
        _fail('reason must be a string')
    if reason:
        _enum(RejectReason, reason, 'reason')
    progress = _finite(data['signed_progress_rad'], 'signed_progress_rad')
    travel = _finite(data['abs_travel_rad'], 'abs_travel_rad')
    if travel < 0.0 or abs(progress) > travel + 1e-9:
        _fail('abs_travel_rad must bound |signed_progress_rad|')
    if not isinstance(data['stopped'], bool):
        _fail('stopped must be a boolean')
    if state == GuardState.ROTATING and data['stopped']:
        _fail('a ROTATING guard cannot report stopped')
    ages = data['source_ages']
    if not isinstance(ages, dict):
        _fail('source_ages must be an object')
    checked = []
    for key in sorted(ages):
        if not isinstance(key, str) or not key:
            _fail('source_ages keys must be non-empty strings')
        # None means "never received", never "age zero".
        checked.append((key, _optional_nonnegative(
            ages[key], f'source_ages[{key}]')))
    return MotionStatus(SCHEMA_VERSION, session, sequence, state, reason,
                        progress, travel, data['stopped'], tuple(checked))


def decode_motion_status(text):
    """Parse a std_msgs/String motion status payload strictly."""
    return make_motion_status(_strict_json_object(text))


def encode_motion_status(status):
    """Serialize a validated status."""
    data = {name: getattr(status, name) for name in MOTION_STATUS_FIELDS}
    data['state'] = status.state.value
    data['source_ages'] = dict(status.source_ages)
    make_motion_status(dict(data))
    return _encode(data)


def validate_strategy_configuration(strategy, motion_policy, validation_only,
                                    motion_profile_path,
                                    operator_rotation_clear=False):
    """
    Reject conflicting launch-time strategy combinations.

    ``motion_policy`` governs only the new strategies.  The legacy strategy
    keeps its existing full-rotation interlocks unchanged; requesting
    ``guarded`` with it is a conflict because the guard never drives it.
    ``validation_only`` stays orthogonal: it only controls Nav2 activation.
    ``operator_rotation_clear`` is the per-launch placement attestation and
    is meaningful only for guarded segmented rotation.
    """
    strategy = _enum(Strategy, strategy, 'localization_strategy')
    policy = _enum(MotionPolicy, motion_policy, 'motion_policy')
    if not isinstance(validation_only, bool):
        _fail('validation_only must be a boolean')
    if not isinstance(motion_profile_path, str):
        _fail('motion_profile_path must be a string')
    if not isinstance(operator_rotation_clear, bool):
        _fail('operator_rotation_clear must be a boolean')
    if policy == MotionPolicy.GUARDED:
        if strategy != Strategy.SEGMENTED_ROTATION:
            _fail(f'motion_policy=guarded is invalid for {strategy.value}')
        if not motion_profile_path:
            _fail('motion_policy=guarded requires motion_profile_path')
    elif operator_rotation_clear:
        _fail('operator_rotation_clear requires motion_policy=guarded')
    return strategy, policy


def validate_confined_parameters(parameters):
    """Validate the numeric Issue #13 parameters; return a normalized copy."""
    p = dict(parameters)
    for name in ('train_frames_per_view', 'holdout_frames_per_view',
                 'max_views', 'max_probe_segments', 'max_refined_clusters'):
        _integer(p[name], name, minimum=1)
    _integer(p['max_extra_refined_clusters'], 'max_extra_refined_clusters',
             minimum=0)
    angles = p['probe_angles_rad']
    if not isinstance(angles, (list, tuple)) or not angles:
        _fail('probe_angles_rad must be a non-empty list')
    for angle in angles:
        angle = _finite(angle, 'probe_angles_rad')
        if angle == 0.0 or abs(angle) > MAX_PROBE_ANGLE_RAD + 1e-9:
            _fail('probe angles must be nonzero with |angle| <= pi/2')
    p['probe_angles_rad'] = tuple(float(angle) for angle in angles)
    positive = (
        'max_total_probe_yaw_rad', 'probe_motion_timeout_sec',
        'motion_request_timeout_sec', 'sensor_freshness_sec',
        'search_timeout_sec', 'session_timeout_sec',
        'independent_cluster_xy_m', 'independent_cluster_yaw_rad',
    )
    for name in positive:
        if _finite(p[name], name) <= 0.0:
            _fail(f'{name} must be positive')
        p[name] = float(p[name])
    if p['max_total_probe_yaw_rad'] > 2.0 * math.pi + 1e-9:
        _fail('max_total_probe_yaw_rad cannot exceed 2*pi')
    if p['sensor_freshness_sec'] > 0.5:
        _fail('sensor_freshness_sec cannot be relaxed above 0.5 s')
    if p['search_timeout_sec'] > p['session_timeout_sec']:
        _fail('search_timeout_sec cannot exceed session_timeout_sec')
    return p
