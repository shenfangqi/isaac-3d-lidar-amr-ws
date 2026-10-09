"""Unit tests for the Issue #13 confined-localization v1 contracts."""

import json
import math

import pytest

from isaac_3d_lidar_bringup.localization_contracts import (
    attestation_covers,
    ContractError,
    decode_motion_profile,
    decode_motion_request,
    decode_motion_status,
    encode_motion_profile,
    encode_motion_request,
    encode_motion_status,
    FrameRole,
    GuardState,
    Hypothesis,
    Keyframe,
    make_motion_request,
    MotionOperation,
    MotionPolicy,
    MotionProfile,
    profile_permits_motion,
    ProfileStatus,
    REJECT_REASON_TEXT,
    RejectReason,
    RotationAttestation,
    RotationDecision,
    SE2,
    SearchResult,
    Strategy,
    validate_confined_parameters,
    validate_strategy_configuration,
)


HASH = 'a' * 64


def _request(**overrides):
    data = {
        'schema_version': 1, 'session': 's1', 'sequence': 3,
        'operation': 'ROTATE', 'delta_yaw_rad': math.pi / 6,
        'speed_rad_s': 0.4, 'profile_hash': HASH,
    }
    data.update(overrides)
    return data


def _status(**overrides):
    data = {
        'schema_version': 1, 'session': 's1', 'sequence': 3,
        'state': 'ROTATING', 'reason': '', 'signed_progress_rad': -0.2,
        'abs_travel_rad': 0.25, 'stopped': False,
        'source_ages': {'odom': 0.05, 'scan': None},
    }
    data.update(overrides)
    return data


def _profile(**overrides):
    data = {
        'schema_version': 1, 'geometry_hash': HASH, 'extrinsics_hash': HASH,
        'control_chain_hash': HASH, 'evidence_ids': ['bag-1'],
        'stop_tail_rad': 0.05, 'center_drift_m': 0.01, 'latency_s': 0.1,
        'externally_reviewed': True, 'status': 'ACCEPTED',
    }
    data.update(overrides)
    return data


def test_every_reject_reason_has_operator_text():
    assert set(REJECT_REASON_TEXT) == set(RejectReason)
    assert len(RejectReason) >= 13


def test_se2_compose_and_inverse_round_trip():
    transform = SE2(1.0, -2.0, 2.5)
    point = SE2(0.3, 0.4, -1.0)
    restored = transform.inverse().compose(transform.compose(point))
    assert restored.x == pytest.approx(point.x, abs=1e-12)
    assert restored.y == pytest.approx(point.y, abs=1e-12)
    assert restored.yaw == pytest.approx(point.yaw, abs=1e-12)


def test_se2_rejects_non_finite_values():
    with pytest.raises(ContractError):
        SE2(math.nan, 0.0, 0.0)


def test_keyframe_requires_known_role_and_source_stamp():
    identity = SE2(0.0, 0.0, 0.0)
    frame = Keyframe(0, 's1', 10, 0, object(), identity, identity, 1.0,
                     'HOLDOUT')
    assert frame.role is FrameRole.HOLDOUT
    with pytest.raises(ContractError):
        Keyframe(0, 's1', 10, 0, object(), identity, identity, 1.0, 'BOTH')
    with pytest.raises(ContractError):
        Keyframe(0, 's1', 0, 0, object(), identity, identity, 1.0, 'TRAIN')


def test_hypothesis_rejects_out_of_range_metrics():
    Hypothesis(0.0, 0.0, math.pi, 0.8, 0.7, 0.1, 0, (0.8,), ())
    with pytest.raises(ContractError):
        Hypothesis(0.0, 0.0, 0.0, 1.2, 0.7, 0.1, 0, (), ())
    with pytest.raises(ContractError):
        Hypothesis(0.0, 0.0, 4.0, 0.8, 0.7, 0.1, 0, (), ())


def test_incomplete_search_must_carry_reason():
    with pytest.raises(ContractError):
        SearchResult('s1', HASH, False, (), 10, 1.0, '')
    result = SearchResult('s1', HASH, False, (), 10, 1.0,
                          'SEARCH_INCOMPLETE')
    assert not result.complete


def test_allowed_rotation_cannot_sweep_unknown_cells():
    with pytest.raises(ContractError):
        RotationDecision(True, 0.5, '', 0.3, 1, 5)
    with pytest.raises(ContractError):
        RotationDecision(True, 0.0, '', 0.3, 0, 5)
    with pytest.raises(ContractError):
        RotationDecision(False, 0.5, '', None, 3, 5)
    denied = RotationDecision(False, 0.5, 'UNKNOWN_SWEEP', None, 3, 5)
    assert denied.min_clearance_m is None


def test_attested_cells_do_not_count_as_unknown():
    decision = RotationDecision(True, 0.5, '', 0.3, 0, 5, attested_cells=40)
    assert decision.attested_cells == 40
    with pytest.raises(ContractError):
        RotationDecision(True, 0.5, '', 0.3, 0, 5, attested_cells=-1)


def _attestation(**overrides):
    data = {'session': 's1', 'odom_pose': SE2(1.0, 2.0, 0.3),
            'issued_mono': 100.0, 'max_translation_m': 0.05,
            'max_age_s': 240.0}
    data.update(overrides)
    return RotationAttestation(**data)


def test_attestation_survives_rotation_in_place():
    attestation = _attestation()
    rotated = SE2(1.02, 2.0, -2.5)
    assert attestation_covers(attestation, 's1', rotated, 150.0)


@pytest.mark.parametrize('session, pose, now', [
    ('s2', SE2(1.0, 2.0, 0.3), 150.0),
    ('s1', SE2(1.2, 2.0, 0.3), 150.0),
    ('s1', SE2(1.0, 2.0, 0.3), 400.0),
    ('s1', SE2(1.0, 2.0, 0.3), 50.0),
])
def test_attestation_expires_on_session_move_or_age(session, pose, now):
    assert not attestation_covers(_attestation(), session, pose, now)
    assert not attestation_covers(None, 's1', pose, 150.0)


def test_attestation_rejects_invalid_bounds():
    with pytest.raises(ContractError):
        _attestation(max_translation_m=0.0)
    with pytest.raises(ContractError):
        _attestation(odom_pose=(1.0, 2.0, 0.3))


def test_rotation_attestation_requires_guarded_policy():
    validate_strategy_configuration(
        'segmented_rotation', 'guarded', True, '/tmp/profile.json', True)
    with pytest.raises(ContractError):
        validate_strategy_configuration(
            'segmented_rotation', 'forbid', True, '', True)
    with pytest.raises(ContractError):
        validate_strategy_configuration(
            'stationary_only', 'forbid', True, '', 'yes')


def test_rotation_decision_enforces_probe_limit():
    with pytest.raises(ContractError):
        RotationDecision(False, 2.0, 'UNKNOWN_SWEEP', None, 0, 5)


def test_profile_keeps_null_and_never_defaults_to_zero():
    encoded = encode_motion_profile(decode_motion_profile(json.dumps(_profile(
        status='ESTIMATED', externally_reviewed=False, stop_tail_rad=None))))
    assert json.loads(encoded)['stop_tail_rad'] is None


def test_reviewed_profile_requires_external_record_and_values():
    with pytest.raises(ContractError):
        decode_motion_profile(json.dumps(_profile(externally_reviewed=False)))
    with pytest.raises(ContractError):
        decode_motion_profile(json.dumps(_profile(center_drift_m=None)))
    with pytest.raises(ContractError):
        decode_motion_profile(json.dumps(_profile(evidence_ids=[])))


def test_profile_rejects_unknown_schema_and_fields():
    with pytest.raises(ContractError):
        decode_motion_profile(json.dumps(_profile(schema_version=2)))
    with pytest.raises(ContractError):
        decode_motion_profile(json.dumps(_profile(extra=1)))


def test_profile_permits_motion_only_when_accepted_and_matching():
    accepted = decode_motion_profile(json.dumps(_profile()))
    assert profile_permits_motion(accepted, HASH, HASH, HASH) == (True, None)
    other = 'b' * 64
    assert profile_permits_motion(accepted, other, HASH, HASH) == (
        False, RejectReason.PROFILE_INVALID)
    estimated = MotionProfile(1, HASH, HASH, HASH, (), None, None, None,
                              False, ProfileStatus.ESTIMATED)
    assert not profile_permits_motion(estimated, HASH, HASH, HASH)[0]
    assert not profile_permits_motion(None, HASH, HASH, HASH)[0]


def test_motion_request_round_trip():
    request = decode_motion_request(json.dumps(_request()))
    assert request.operation is MotionOperation.ROTATE
    assert decode_motion_request(encode_motion_request(request)) == request


@pytest.mark.parametrize('payload', [
    'NaN',
    '{"schema_version": 1, "session": "s1", "sequence": 3, '
    '"operation": "ROTATE", "delta_yaw_rad": NaN, "speed_rad_s": 0.4, '
    '"profile_hash": "' + HASH + '"}',
    '{"schema_version": 1, "schema_version": 1}',
    '[]',
])
def test_motion_request_rejects_malformed_json(payload):
    with pytest.raises(ContractError):
        decode_motion_request(payload)


@pytest.mark.parametrize('overrides', [
    {'schema_version': 2},
    {'schema_version': True},
    {'sequence': -1},
    {'sequence': True},
    {'session': ''},
    {'operation': 'SPIN'},
    {'delta_yaw_rad': 0.0},
    {'delta_yaw_rad': 2.0},
    {'speed_rad_s': 0.0},
    {'speed_rad_s': 0.41},
    {'profile_hash': ''},
    {'extra': 1},
])
def test_motion_request_rejects_invalid_fields(overrides):
    with pytest.raises(ContractError):
        make_motion_request(_request(**overrides))


def test_stop_and_release_carry_zero_motion():
    stop = make_motion_request(_request(
        operation='STOP', delta_yaw_rad=0.0, speed_rad_s=0.0,
        profile_hash=''))
    assert stop.operation is MotionOperation.STOP
    with pytest.raises(ContractError):
        make_motion_request(_request(operation='RELEASE'))


def test_motion_request_missing_field_is_rejected():
    data = _request()
    del data['profile_hash']
    with pytest.raises(ContractError):
        make_motion_request(data)


def test_motion_status_round_trip_keeps_unknown_age():
    status = decode_motion_status(json.dumps(_status()))
    assert status.state is GuardState.ROTATING
    assert dict(status.source_ages)['scan'] is None
    assert decode_motion_status(encode_motion_status(status)) == status


@pytest.mark.parametrize('overrides', [
    {'abs_travel_rad': 0.1},
    {'state': 'ROTATING', 'stopped': True},
    {'reason': 'TOO_CLOSE'},
    {'source_ages': {'odom': -1.0}},
    {'state': 'MOVING'},
])
def test_motion_status_rejects_inconsistent_fields(overrides):
    with pytest.raises(ContractError):
        decode_motion_status(json.dumps(_status(**overrides)))


def test_status_before_handshake_has_empty_session():
    status = decode_motion_status(json.dumps(_status(
        session='', state='IDLE', stopped=True, signed_progress_rad=0.0,
        abs_travel_rad=0.0)))
    assert status.session == ''


@pytest.mark.parametrize('strategy', list(Strategy))
@pytest.mark.parametrize('validation_only', [True, False])
def test_strategy_matrix(strategy, validation_only):
    # forbid is valid for every strategy and never needs a profile.
    assert validate_strategy_configuration(
        strategy.value, 'forbid', validation_only, '') == (
            strategy, MotionPolicy.FORBID)
    if strategy is Strategy.SEGMENTED_ROTATION:
        validate_strategy_configuration(
            strategy.value, 'guarded', validation_only, '/tmp/profile.json')
        with pytest.raises(ContractError):
            validate_strategy_configuration(
                strategy.value, 'guarded', validation_only, '')
    else:
        with pytest.raises(ContractError):
            validate_strategy_configuration(
                strategy.value, 'guarded', validation_only,
                '/tmp/profile.json')


def test_strategy_configuration_rejects_unknown_values():
    with pytest.raises(ContractError):
        validate_strategy_configuration('auto', 'forbid', True, '')
    with pytest.raises(ContractError):
        validate_strategy_configuration('stationary_only', 'allow', True, '')
    with pytest.raises(ContractError):
        validate_strategy_configuration('stationary_only', 'forbid', 1, '')


def _parameters(**overrides):
    data = {
        'train_frames_per_view': 3, 'holdout_frames_per_view': 3,
        'max_views': 8, 'max_probe_segments': 6, 'max_refined_clusters': 8,
        'max_extra_refined_clusters': 16,
        'probe_angles_rad': [math.pi / 6, -math.pi / 6, math.pi / 3,
                             -math.pi / 3, math.pi / 2, -math.pi / 2],
        'max_total_probe_yaw_rad': 2.0 * math.pi,
        'probe_motion_timeout_sec': 45.0,
        'motion_request_timeout_sec': 0.30, 'sensor_freshness_sec': 0.5,
        'search_timeout_sec': 120.0, 'session_timeout_sec': 240.0,
        'independent_cluster_xy_m': 0.30,
        'independent_cluster_yaw_rad': math.pi / 12,
    }
    data.update(overrides)
    return data


def test_default_confined_parameters_are_valid():
    normalized = validate_confined_parameters(_parameters())
    assert normalized['probe_angles_rad'][0] == pytest.approx(math.pi / 6)


@pytest.mark.parametrize('overrides', [
    {'train_frames_per_view': 0},
    {'max_views': 2.5},
    {'probe_angles_rad': [0.0]},
    {'probe_angles_rad': [2.0]},
    {'probe_angles_rad': []},
    {'max_total_probe_yaw_rad': 7.0},
    {'sensor_freshness_sec': 0.8},
    {'search_timeout_sec': 300.0},
    {'motion_request_timeout_sec': 0.0},
])
def test_confined_parameters_reject_relaxed_or_invalid(overrides):
    with pytest.raises(ContractError):
        validate_confined_parameters(_parameters(**overrides))
