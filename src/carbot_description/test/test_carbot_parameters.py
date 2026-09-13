import math
from pathlib import Path

import pytest
import yaml


PARAMETER_FILE = (
    Path(__file__).resolve().parents[1] / "config" / "carbot_parameters.yaml"
)


@pytest.fixture(scope="module")
def parameters():
    with PARAMETER_FILE.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def wheel_angular_velocities(linear_mps, angular_rad_s, parameters):
    kinematics = parameters["kinematics"]
    separation = kinematics["effective_track_separation_m"]
    radius = kinematics["effective_sprocket_radius_m"]
    left = (linear_mps - angular_rad_s * separation / 2.0) / radius
    right = (linear_mps + angular_rad_s * separation / 2.0) / radius
    return left, right


def saturate_wheel_velocities(left, right, maximum):
    scale = min(1.0, maximum / max(abs(left), abs(right)))
    return left * scale, right * scale, scale


def test_encoder_conversion_is_one_revolution(parameters):
    counts = parameters["kinematics"]["encoder_counts_per_revolution"]
    radians_per_tick = 2.0 * math.pi / counts
    assert counts * radians_per_tick == pytest.approx(2.0 * math.pi)
    assert radians_per_tick == pytest.approx(0.004027682889218966)


def test_rpm_and_radians_per_second_are_consistent(parameters):
    control = parameters["control"]
    converted = control["max_wheel_rpm"] * 2.0 * math.pi / 60.0
    assert control["max_wheel_velocity_rad_s"] == pytest.approx(
        converted, abs=0.001
    )


def test_effective_radius_sets_coupled_linear_limit(parameters):
    kinematics = parameters["kinematics"]
    control = parameters["control"]
    maximum_tangential_speed = (
        kinematics["effective_sprocket_radius_m"]
        * control["max_wheel_velocity_rad_s"]
    )
    assert maximum_tangential_speed == pytest.approx(0.569415)


@pytest.mark.parametrize(
    ("linear_mps", "angular_rad_s"),
    [(0.5, 3.5), (-0.5, 3.5), (0.5, -3.5), (-0.5, -3.5)],
)
def test_coupled_limit_scales_both_tracks_and_preserves_curvature(
    parameters, linear_mps, angular_rad_s
):
    maximum = parameters["control"]["max_wheel_velocity_rad_s"]
    original_left, original_right = wheel_angular_velocities(
        linear_mps, angular_rad_s, parameters
    )
    left, right, scale = saturate_wheel_velocities(
        original_left, original_right, maximum
    )

    assert scale < 1.0
    assert max(abs(left), abs(right)) == pytest.approx(maximum)
    assert left == pytest.approx(original_left * scale)
    assert right == pytest.approx(original_right * scale)

    separation = parameters["kinematics"]["effective_track_separation_m"]
    radius = parameters["kinematics"]["effective_sprocket_radius_m"]
    reconstructed_linear = radius * (left + right) / 2.0
    reconstructed_angular = radius * (right - left) / separation
    assert reconstructed_angular / reconstructed_linear == pytest.approx(
        angular_rad_s / linear_mps
    )


def test_physical_and_effective_geometry_are_not_conflated(parameters):
    physical = parameters["geometry"]["physical_track_separation_m"]
    effective = parameters["kinematics"]["effective_track_separation_m"]
    visual_radius = parameters["geometry"]["wheel_groups"]["rear_drive"][
        "radius_m"
    ]
    effective_radius = parameters["kinematics"]["effective_sprocket_radius_m"]
    assert physical == 0.225
    assert effective == 0.254
    assert visual_radius == 0.021
    assert effective_radius == 0.02175
    assert physical != effective
    assert visual_radius != effective_radius


def test_ideal_sim_does_not_apply_real_robot_trim(parameters):
    control = parameters["control"]
    assert control["ideal_sim_right_straight_trim"] == 1.0
    assert control["high_fidelity_right_straight_trim"] == 1.0005


def test_temporary_values_are_explicitly_unvalidated(parameters):
    marker = "TEMP_ESTIMATE_NOT_CALIBRATED"
    assert parameters["dynamics"]["calibration_status"] == marker
    assert parameters["domain_randomization"]["calibration_status"] == marker
    temporary_source = parameters["provenance"][marker]
    assert temporary_source["status"] == marker
    assert set(temporary_source["scope"]) == {
        "sensors.mid360.visual_proxy",
        "dynamics",
        "sensor_frame_assumptions",
        "domain_randomization",
    }


def test_critical_values_have_expected_provenance(parameters):
    provenance = parameters["provenance"]
    assert "geometry.body_collision" in provenance[
        "MEASURED_VALUE_CARBOT_2026_09_13"
    ]["scope"]
    assert "geometry.body_visual" in provenance[
        "DESIGN_VALUE_CARBOT_SIMULATION_2026_09_13"
    ]["scope"]
    assert set(
        provenance["GROUND_CALIBRATED_VALUE_CARBOT_2026_09_13"]["scope"]
    ) == {
        "kinematics.effective_track_separation_m",
        "kinematics.effective_sprocket_radius_m",
    }
    assert "control" in provenance[
        "SOURCE_CODE_VALUE_CARBOT_CONTROL_2026_09_13"
    ]["scope"]
    assert all(
        source["category"]
        in {
            "measured_value",
            "ground_calibrated_value",
            "source_code_value",
            "observed_value",
            "design_value",
            "temporary_estimate",
        }
        for source in provenance.values()
    )


def test_every_parameter_leaf_has_exactly_one_valid_source_scope(parameters):
    def leaf_paths(node, prefix=""):
        if isinstance(node, dict):
            for key, value in node.items():
                path = f"{prefix}.{key}" if prefix else key
                yield from leaf_paths(value, path)
        else:
            yield prefix

    parameter_tree = {
        key: value
        for key, value in parameters.items()
        if key not in {"schema_version", "provenance"}
    }
    leaves = set(leaf_paths(parameter_tree))
    scopes = [
        scope
        for source in parameters["provenance"].values()
        for scope in source["scope"]
    ]

    for scope in scopes:
        assert scope in leaves or any(path.startswith(f"{scope}.") for path in leaves)
    for path in leaves:
        matches = [
            scope for scope in scopes if path == scope or path.startswith(f"{scope}.")
        ]
        assert len(matches) == 1, (
            f"{path} must have exactly one provenance scope; found {matches}"
        )
