import math

import pytest

from carbot_nav_recovery.speed_advisor import (
    BrakingProfile,
    advise_speed,
    braking_speed_limit,
    curvature_speed_limit,
    evaluate_path_risk,
    maximum_path_curvature,
    stopping_distance,
)
from carbot_nav_recovery.swept_footprint import (
    CostmapSnapshot,
    Pose2D,
    SnapshotFreshness,
)


FOOTPRINT = ((0.10, 0.08), (0.10, -0.08),
             (-0.10, -0.08), (-0.10, 0.08))


def snapshot(*, obstacle=None, unknown=None):
    width = height = 80
    values = [0] * (width * height)
    for point, cost in ((obstacle, 254), (unknown, 255)):
        if point is not None:
            mx = int((point[0] + 2.0) / 0.05)
            my = int((point[1] + 2.0) / 0.05)
            values[my * width + mx] = cost
    return CostmapSnapshot(
        width, height, 0.05, -2.0, -2.0, tuple(values), 'map', 10.0, 20.0)


def freshness(**changes):
    values = dict(now_ros_sec=10.1, now_monotonic_sec=20.1,
                  max_source_age_sec=0.5, max_receive_age_sec=0.5,
                  localization_valid=True, tf_valid=True)
    values.update(changes)
    return SnapshotFreshness(**values)


def accepted_profile(**changes):
    values = dict(
        physical_acceptance_complete=True,
        evidence_directory='/tmp/physical-evidence',
        minimum_deceleration_mps2=0.2,
        command_latency_sec=0.5,
        position_margin_m=0.02,
        max_linear_speed_mps=0.10,
        max_angular_speed_radps=0.50,
    )
    values.update(changes)
    return BrakingProfile(**values)


def straight(length=1.0, step=0.05):
    return tuple(Pose2D(index * step, 0.0, 0.0)
                 for index in range(int(length / step) + 1))


def test_stopping_model_and_inverse_agree():
    profile = accepted_profile()
    distance = stopping_distance(0.08, profile)
    assert distance == pytest.approx(0.076)
    assert braking_speed_limit(distance, profile) == pytest.approx(0.08)


def test_curvature_limit_uses_accepted_angular_speed():
    profile = accepted_profile(max_linear_speed_mps=0.5,
                               max_angular_speed_radps=0.4)
    assert curvature_speed_limit(2.0, profile) == pytest.approx(0.2)
    assert curvature_speed_limit(0.0, profile) == pytest.approx(0.5)


def test_three_point_curvature_detects_quarter_circle():
    radius = 0.5
    path = tuple(Pose2D(radius * math.cos(angle),
                        radius * math.sin(angle), angle + math.pi / 2)
                 for angle in (0.0, math.pi / 4, math.pi / 2))
    assert maximum_path_curvature(path) == pytest.approx(2.0)


def test_path_risk_finds_obstacle_before_end_of_horizon():
    risk = evaluate_path_risk(
        snapshot(obstacle=(0.55, 0.0)), FOOTPRINT, straight())
    assert risk.unsafe_reason == 'LETHAL_OBSTACLE'
    assert 0.35 <= risk.distance_to_unsafe_m <= 0.50


def test_advisor_requires_external_physical_acceptance():
    unaccepted = accepted_profile(
        physical_acceptance_complete=False, evidence_directory='')
    advice = advise_speed(snapshot(), FOOTPRINT, straight(), freshness(),
                          0.10, unaccepted)
    assert advice.reason == 'CALIBRATION_REQUIRED'
    assert advice.recommended_speed_mps is None
    assert advice.motion_eligible is False
    assert advice.validation_only is True


def test_advisor_requests_slowdown_before_predicted_collision():
    advice = advise_speed(
        snapshot(obstacle=(0.20, 0.0)), FOOTPRINT, straight(), freshness(),
        0.10, accepted_profile())
    assert advice.reason == 'PREDICTED_COLLISION'
    assert advice.recommended_speed_mps < 0.10
    assert advice.speed_reduction_required is True
    assert advice.motion_eligible is False


def test_advisor_reports_curvature_limit_separately():
    profile = accepted_profile(max_linear_speed_mps=0.5,
                               max_angular_speed_radps=0.2)
    path = tuple(Pose2D(0.5 * math.cos(angle), 0.5 * math.sin(angle),
                        angle + math.pi / 2)
                 for angle in (0.0, math.pi / 4, math.pi / 2))
    advice = advise_speed(snapshot(), FOOTPRINT, path, freshness(),
                          0.20, profile)
    assert advice.reason == 'CURVATURE_SPEED_UNSAFE'
    assert advice.curvature_speed_limit_mps == pytest.approx(0.10)


def test_stale_snapshot_fails_before_geometry_evaluation():
    advice = advise_speed(
        snapshot(), FOOTPRINT, straight(),
        freshness(now_ros_sec=11.0), 0.0, accepted_profile())
    assert advice.reason == 'STALE_SENSOR'
    assert advice.recommended_speed_mps is None


def test_profile_schema_is_strict():
    values = accepted_profile().__dict__.copy()
    assert BrakingProfile.from_mapping(values) == accepted_profile()
    values['unexpected'] = 1
    with pytest.raises(ValueError, match='schema'):
        BrakingProfile.from_mapping(values)
