"""
Issue #13 phase 2 wire contracts for guarded translation probes.

Separate from the rotation contract (``localization_contracts`` schema 1),
which stays unchanged: a rotation-only guard never sees these messages, and
a translation can never be smuggled through a ROTATE request.  All JSON is
decoded strictly (exact fields, finite numbers, known enums).

* :class:`TranslationRequest` - manager -> guard lease (10 Hz), STOP / MOVE
  / RELEASE, with a target distance, a speed cap and the linear profile hash.
* :class:`RouteEvidence` - manager -> guard, every cycle: the remaining
  forward path re-checked on the static map under *every* plausible
  candidate from the current odometry pose.  The guard refuses motion when
  it is stale, at another pose, too short or not clear.
* :class:`TranslationStatus` - guard -> manager.
"""

from dataclasses import asdict, dataclass
import json
import math

from .localization_contracts import ContractError, SE2
from .localization_translation_guard import LinearProfile
from .localization_translation_policy import TranslationPreview

TRANSLATION_SCHEMA = 1
OPERATIONS = ('STOP', 'MOVE', 'RELEASE')
STATES = ('IDLE', 'STOPPING', 'STOPPED', 'MOVING', 'FAULT', 'RELEASED')
MAX_SEGMENT_M = 0.6
MAX_SPEED_MPS = 0.10


def _fail(message):
    raise ContractError(message)


def _exact(data, fields, kind):
    if not isinstance(data, dict):
        _fail(f'{kind} must be an object')
    if set(data) != set(fields):
        _fail(f'{kind} fields must be exactly {sorted(fields)}')


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) \
            or not math.isfinite(value):
        _fail(f'{name} must be a finite number')
    return float(value)


def _integer(value, name, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        _fail(f'{name} must be an integer >= {minimum}')
    return value


def _session(value):
    if not isinstance(value, str) or not 8 <= len(value) <= 64 or not value.isalnum():
        _fail('session must be 8-64 alphanumeric characters')
    return value


def _digest(value, name, allow_empty=False):
    if allow_empty and value == '':
        return value
    if not (isinstance(value, str) and len(value) == 64
            and all(c in '0123456789abcdef' for c in value)):
        _fail(f'{name} must be a 64-hex digest')
    return value


@dataclass(frozen=True)
class TranslationRequest:
    """Manager -> guard translation lease."""

    session: str
    sequence: int
    operation: str
    distance_m: float = 0.0
    speed_mps: float = 0.0
    profile_hash: str = ''

    def __post_init__(self):
        _session(self.session)
        _integer(self.sequence, 'sequence')
        if self.operation not in OPERATIONS:
            _fail(f'operation must be one of {OPERATIONS}')
        distance = _number(self.distance_m, 'distance_m')
        speed = _number(self.speed_mps, 'speed_mps')
        if self.operation == 'MOVE':
            if not 0 < distance <= MAX_SEGMENT_M:
                _fail(f'MOVE distance_m must be within (0, {MAX_SEGMENT_M}]')
            if not 0 < speed <= MAX_SPEED_MPS:
                _fail(f'MOVE speed_mps must be within (0, {MAX_SPEED_MPS}]')
            _digest(self.profile_hash, 'profile_hash')
        elif distance != 0 or speed != 0 or self.profile_hash:
            _fail(f'{self.operation} cannot carry a target')


def encode_translation_request(request):
    return json.dumps({'schema_version': TRANSLATION_SCHEMA,
                       'kind': 'TRANSLATION_REQUEST', **asdict(request)},
                      sort_keys=True)


def decode_translation_request(text):
    data = _load(text)
    _exact(data, ('schema_version', 'kind', 'session', 'sequence', 'operation',
                  'distance_m', 'speed_mps', 'profile_hash'), 'translation request')
    _header(data, 'TRANSLATION_REQUEST')
    return TranslationRequest(data['session'], data['sequence'], data['operation'],
                              data['distance_m'], data['speed_mps'],
                              data['profile_hash'])


@dataclass(frozen=True)
class RouteEvidence:
    """
    Remaining forward path, checked on the static map under all candidates.

    ``start_*`` is the odometry pose the check used (odom frame), at source
    stamp ``odom_stamp_ns``.  ``candidate_digest`` binds the evidence to the
    candidate set that was checked.
    """

    session: str
    sequence: int
    odom_stamp_ns: int
    start_x: float
    start_y: float
    start_yaw: float
    remaining_m: float
    stop_extension_m: float
    clear: bool
    reason: str
    map_hash: str
    candidate_digest: str

    def __post_init__(self):
        _session(self.session)
        _integer(self.sequence, 'sequence')
        _integer(self.odom_stamp_ns, 'odom_stamp_ns', minimum=1)
        for name in ('start_x', 'start_y', 'start_yaw', 'remaining_m',
                     'stop_extension_m'):
            _number(getattr(self, name), name)
        if self.remaining_m < 0 or self.stop_extension_m < 0:
            _fail('remaining_m and stop_extension_m cannot be negative')
        if type(self.clear) is not bool or not isinstance(self.reason, str):
            _fail('clear must be a bool and reason a string')
        if self.clear and self.reason:
            _fail('clear evidence cannot carry a refusal reason')
        if not self.clear and not self.reason:
            _fail('refusing evidence needs a reason')
        _digest(self.map_hash, 'map_hash')
        _digest(self.candidate_digest, 'candidate_digest')

    def to_preview(self):
        """Convert to the guard preview form (pose and stamp checked there)."""
        start = SE2(self.start_x, self.start_y, self.start_yaw)
        distance = max(self.remaining_m, 1e-3)
        return TranslationPreview(
            self.clear, self.reason, distance, self.stop_extension_m, start,
            start.compose(SE2(distance, 0.0, 0.0)), self.odom_stamp_ns)


def encode_route_evidence(evidence):
    return json.dumps({'schema_version': TRANSLATION_SCHEMA,
                       'kind': 'ROUTE_EVIDENCE', **asdict(evidence)}, sort_keys=True)


def decode_route_evidence(text):
    data = _load(text)
    _exact(data, ('schema_version', 'kind', 'session', 'sequence', 'odom_stamp_ns',
                  'start_x', 'start_y', 'start_yaw', 'remaining_m',
                  'stop_extension_m', 'clear', 'reason', 'map_hash',
                  'candidate_digest'), 'route evidence')
    _header(data, 'ROUTE_EVIDENCE')
    data = {k: v for k, v in data.items() if k not in ('schema_version', 'kind')}
    return RouteEvidence(**data)


@dataclass(frozen=True)
class TranslationStatus:
    """Guard -> manager translation state."""

    session: str
    sequence: int
    state: str
    reason: str
    progress_m: float
    total_travel_m: float
    segments: int
    command_mps: float
    permitted: bool

    def __post_init__(self):
        if self.session:
            _session(self.session)
        if not isinstance(self.sequence, int) or isinstance(self.sequence, bool) \
                or self.sequence < -1:
            _fail('sequence must be an integer >= -1')
        if self.state not in STATES:
            _fail(f'state must be one of {STATES}')
        if not isinstance(self.reason, str) or type(self.permitted) is not bool:
            _fail('reason must be a string and permitted a bool')
        for name in ('progress_m', 'total_travel_m', 'command_mps'):
            _number(getattr(self, name), name)
        _integer(self.segments, 'segments')


def translation_status(guard):
    """Status of a :class:`LinearProbeGuard`."""
    return TranslationStatus(guard.session, guard.sequence, guard.state, guard.reason,
                             float(guard.progress_m), float(guard.total_travel_m),
                             guard.segments, float(guard.command_mps), guard.permitted)


def encode_translation_status(status):
    return json.dumps({'schema_version': TRANSLATION_SCHEMA,
                       'kind': 'TRANSLATION_STATUS', **asdict(status)}, sort_keys=True)


def decode_translation_status(text):
    data = _load(text)
    _exact(data, ('schema_version', 'kind', 'session', 'sequence', 'state', 'reason',
                  'progress_m', 'total_travel_m', 'segments', 'command_mps',
                  'permitted'), 'translation status')
    _header(data, 'TRANSLATION_STATUS')
    data = {k: v for k, v in data.items() if k not in ('schema_version', 'kind')}
    return TranslationStatus(**data)


def encode_linear_profile(profile):
    data = asdict(profile)
    data['evidence_ids'] = list(profile.evidence_ids)
    return json.dumps({'schema_version': TRANSLATION_SCHEMA,
                       'kind': 'LINEAR_PROFILE', **data}, sort_keys=True, indent=2)


def decode_linear_profile(text):
    """Strict decode; an ACCEPTED status alone does not permit motion."""
    data = _load(text)
    _exact(data, ('schema_version', 'kind', 'geometry_hash', 'extrinsics_hash',
                  'control_chain_hash', 'evidence_ids', 'max_speed_mps', 'reaction_s',
                  'stop_tail_m', 'watchdog_s', 'lateral_error_m', 'status'),
           'linear profile')
    _header(data, 'LINEAR_PROFILE')
    if not isinstance(data['evidence_ids'], list):
        _fail('evidence_ids must be a list')
    data = {k: v for k, v in data.items() if k not in ('schema_version', 'kind')}
    return LinearProfile(**data)


def _load(text):
    try:
        return json.loads(text)
    except (TypeError, ValueError) as error:
        raise ContractError(f'not JSON: {error}') from error


def _header(data, kind):
    if data['schema_version'] != TRANSLATION_SCHEMA or isinstance(
            data['schema_version'], bool):
        _fail(f'unsupported schema {data["schema_version"]!r}')
    if data['kind'] != kind:
        _fail(f'expected {kind}, got {data["kind"]!r}')


class TranslationLink:
    """
    Manager side of the translation lease for one localization session.

    Every change of intent uses a new, strictly increasing sequence;
    ``request`` is re-sent unchanged at the control rate as a renewal.
    The first request is the STOP handshake.
    """

    def __init__(self, session):
        self.session = _session(session)
        self.sequence = 1
        self._request = TranslationRequest(session, 1, 'STOP')

    @property
    def operation(self):
        return self._request.operation

    def request(self):
        return self._request

    def _next(self, operation, distance=0.0, speed=0.0, profile_hash=''):
        self.sequence += 1
        self._request = TranslationRequest(self.session, self.sequence, operation,
                                           distance, speed, profile_hash)

    def stop(self):
        self._next('STOP')

    def move(self, distance_m, speed_mps, profile_hash):
        self._next('MOVE', distance_m, speed_mps, profile_hash)

    def release(self):
        self._next('RELEASE')

    def owns(self, status):
        """Whether ``status`` reports on this link's latest request."""
        return (status is not None and status.session == self.session
                and status.sequence == self.sequence)
