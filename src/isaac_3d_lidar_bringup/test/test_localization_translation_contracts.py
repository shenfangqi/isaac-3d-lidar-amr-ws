"""Phase 2: translation wire contracts, command merging and map evidence."""

import json
import math

import pytest

from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError, encode_motion_request, MotionOperation, MotionRequest,
)
from isaac_3d_lidar_bringup.localization_motion_guard import OdomSample
from isaac_3d_lidar_bringup.localization_translation_contracts import (
    decode_linear_profile, decode_route_evidence, decode_translation_request,
    decode_translation_status, encode_linear_profile, encode_route_evidence,
    encode_translation_request, encode_translation_status, RouteEvidence,
    translation_status, TranslationRequest,
)
from isaac_3d_lidar_bringup.localization_translation_guard import (
    combine_commands, LinearProbeGuard, LinearProfile,
)

SESSION = 'session0000000001'
HASHES = ('a' * 64, 'b' * 64, 'c' * 64)
MAP, CANDIDATES = 'd' * 64, 'e' * 64


def _profile(status='ACCEPTED'):
    return LinearProfile(*HASHES, ('bag-linear-1',), 0.08, 0.1, 0.02, 0.6, 0.03,
                         status)


def test_requests_round_trip_and_are_strict():
    move = TranslationRequest(SESSION, 3, 'MOVE', 0.4, 0.05, 'f' * 64)
    assert decode_translation_request(encode_translation_request(move)) == move
    data = json.loads(encode_translation_request(move))
    for change in ({'extra': 1}, {'kind': 'ROUTE_EVIDENCE'}, {'schema_version': 2},
                   {'distance_m': 0.9}, {'speed_mps': 0.2}, {'operation': 'TURN'},
                   {'profile_hash': 'xyz'}, {'distance_m': float('nan')}):
        with pytest.raises(ContractError):
            decode_translation_request(json.dumps({**data, **change}))
    with pytest.raises(ContractError):
        TranslationRequest(SESSION, 4, 'STOP', 0.1)


def test_a_rotation_request_cannot_be_read_as_a_translation():
    rotate = encode_motion_request(MotionRequest(
        1, SESSION, 2, MotionOperation.ROTATE, 0.5, 0.4, 'f' * 64))
    with pytest.raises(ContractError):
        decode_translation_request(rotate)


def _evidence(**changes):
    values = dict(session=SESSION, sequence=3, odom_stamp_ns=10**9, start_x=1.0,
                  start_y=2.0, start_yaw=0.5, remaining_m=0.4, stop_extension_m=0.2,
                  clear=True, reason='', map_hash=MAP, candidate_digest=CANDIDATES)
    values.update(changes)
    return RouteEvidence(**values)


def test_route_evidence_round_trip_and_preview_form():
    evidence = _evidence()
    assert decode_route_evidence(encode_route_evidence(evidence)) == evidence
    preview = evidence.to_preview()
    assert preview.geometry_clear and preview.distance_m == pytest.approx(0.4)
    assert preview.target.x == pytest.approx(1.0 + 0.4 * math.cos(0.5))
    assert preview.snapshot_stamp_ns == 10**9
    refused = _evidence(clear=False, reason='OBSTACLE_IN_SWEEP').to_preview()
    assert not refused.geometry_clear and refused.reason == 'OBSTACLE_IN_SWEEP'
    with pytest.raises(ContractError):
        _evidence(clear=True, reason='OBSTACLE_IN_SWEEP')
    with pytest.raises(ContractError):
        _evidence(clear=False, reason='')
    with pytest.raises(ContractError):
        _evidence(remaining_m=-0.1)


def test_linear_profile_round_trip_and_status():
    profile = _profile()
    assert decode_linear_profile(encode_linear_profile(profile)) == profile
    guard = LinearProbeGuard(profile, HASHES)
    status = translation_status(guard)
    assert decode_translation_status(encode_translation_status(status)) == status
    assert status.permitted and status.state == 'IDLE'


@pytest.mark.parametrize('profile, hashes, permitted', [
    (None, HASHES, False),
    (_profile('REVIEWED'), HASHES, False),
    (_profile(), ('a' * 64, 'b' * 64, '0' * 64), False),
    (_profile(), HASHES, True),
])
def test_only_an_accepted_matching_linear_profile_permits_motion(profile, hashes,
                                                                 permitted):
    assert LinearProbeGuard(profile, hashes).permitted is permitted


@pytest.mark.parametrize('rotation, angular, linear_state, linear, expected', [
    ('STOPPED', 0.0, 'MOVING', 0.05, (0.05, 0.0, False)),
    ('ROTATING', 0.3, 'STOPPED', 0.0, (0.0, 0.3, False)),
    ('ROTATING', 0.3, 'MOVING', 0.05, (0.0, 0.0, True)),
    ('ROTATING', 0.0, 'MOVING', 0.0, (0.0, 0.0, True)),
    ('IDLE', 0.0, 'IDLE', 0.0, (0.0, 0.0, False)),
])
def test_rotation_and_translation_never_move_together(rotation, angular,
                                                      linear_state, linear, expected):
    assert combine_commands(rotation, angular, linear_state, linear) == expected


class _Clock:
    def __init__(self, guard):
        self.guard, self.now, self.stamp, self.x = guard, 100.0, 10**9, 0.0
        guard.on_emergency(False, self.now)

    def step(self, seconds, evidence=None, dt=0.05):
        for _ in range(round(seconds / dt)):
            self.now += dt
            self.stamp += int(dt * 1e9)
            self.x += self.guard.command_mps * dt
            self.guard.on_odom(OdomSample(self.stamp, self.x, 0.0, 0.0,
                                          self.guard.command_mps, 0.0, self.now))
            self.guard.on_chassis(True, False, self.now)
            if self.guard.state == 'MOVING' or evidence == 'always':
                self.guard.on_preview(_evidence(
                    odom_stamp_ns=self.stamp, start_x=self.x, start_y=0.0, start_yaw=0.0,
                    remaining_m=max(0.0, 0.4 - self.x) + 1e-3,
                    stop_extension_m=self.guard.stop_extension_m).to_preview(), self.now)
            if self.guard.session:
                self.guard.request(*self.lease, self.now, **self.target)
            self.guard.tick(self.now)


def test_map_evidence_drives_a_guarded_segment_to_a_stop():
    profile = _profile()
    guard = LinearProbeGuard(profile, HASHES)
    clock = _Clock(guard)
    clock.lease, clock.target = ('STOP', SESSION, 1), {}
    guard.request('STOP', SESSION, 1, clock.now)
    clock.step(1.5)
    assert guard.state == 'STOPPED'
    clock.lease = ('MOVE', SESSION, 2)
    clock.target = dict(distance_m=0.4, speed_mps=0.05, profile_hash=profile.digest)
    guard.request('MOVE', SESSION, 2, clock.now, **clock.target)
    clock.step(12.0)
    assert guard.state in ('STOPPING', 'STOPPED'), guard.reason
    assert 0.0 < clock.x < 0.4                 # stops early by the stop extension
    assert guard.command_mps == 0.0


def test_missing_or_refusing_map_evidence_stops_the_segment():
    profile = _profile()
    guard = LinearProbeGuard(profile, HASHES)
    clock = _Clock(guard)
    clock.lease, clock.target = ('STOP', SESSION, 1), {}
    guard.request('STOP', SESSION, 1, clock.now)
    clock.step(1.5)
    clock.lease = ('MOVE', SESSION, 2)
    clock.target = dict(distance_m=0.4, speed_mps=0.05, profile_hash=profile.digest)
    guard.request('MOVE', SESSION, 2, clock.now, **clock.target)
    guard.on_preview(_evidence(clear=False, reason='OBSTACLE_IN_SWEEP',
                               odom_stamp_ns=clock.stamp, start_x=clock.x,
                               start_y=0.0, start_yaw=0.0).to_preview(), clock.now)
    guard.tick(clock.now + 0.01)
    assert guard.state == 'FAULT' and guard.reason == 'OBSTACLE_IN_SWEEP'
    assert guard.command_mps == 0.0
