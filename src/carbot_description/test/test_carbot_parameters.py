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


def test_physical_response_envelope_matches_acceptance_evidence(parameters):
    response = parameters["hardware_response"]
    deadband = response["track_deadband"]
    assert response["calibration_status"] == "PARTIAL_PHYSICAL_EVIDENCE"
    assert response["surface"] == "wood_floor"
    assert response["added_payload_kg"] == 0.0
    assert deadband["forward_min_sustainable_mps"] == 0.05
    assert deadband["reverse_min_sustainable_mps"] == 0.02
    assert deadband["reliable_in_place_angular_command_rad_s"] == [
        0.40,
        0.50,
    ]
    assert response["timing"]["first_motion_latency_s"] == 0.073
    assert response["timing"]["ground_stop_tail_s"] == [0.57, 0.92]


def test_randomization_ranges_cover_observed_response(parameters):
    response = parameters["hardware_response"]
    randomization = parameters["domain_randomization"]

    def covered(value, bounds):
        return bounds[0] <= value <= bounds[1]

    assert covered(
        response["track_deadband"]["forward_min_sustainable_mps"],
        randomization["forward_track_deadband_mps"],
    )
    assert covered(
        response["track_deadband"]["reverse_min_sustainable_mps"],
        randomization["reverse_track_deadband_mps"],
    )
    assert covered(
        response["timing"]["first_motion_latency_s"],
        randomization["command_latency_s"],
    )
    assert randomization["stop_tail_s"][0] <= min(
        response["timing"]["ground_stop_tail_s"]
    )
    assert randomization["stop_tail_s"][1] >= max(
        response["timing"]["ground_stop_tail_s"]
    )
    assert randomization["longitudinal_gain"][0] <= 1.0 <= (
        randomization["longitudinal_gain"][1]
    )
    assert randomization["yaw_gain"][0] <= min(
        response["observed_motion"]["yaw_external_over_wheel_gain"]
    )
    assert randomization["yaw_gain"][1] >= max(
        response["observed_motion"]["yaw_external_over_wheel_gain"]
    )


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
    assert "sensors.mid360.simulation_profile_name" in provenance[
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
            "manufacturer_specification",
            "observed_value",
            "design_value",
            "temporary_estimate",
        }
        for source in provenance.values()
    )


def test_mid360_runtime_contract(parameters):
    lidar = parameters["sensors"]["mid360"]
    assert lidar["simulation_profile_name"] == "Livox_Mid360_Approx"
    assert lidar["pointcloud_topic"] == "/livox/lidar"
    assert lidar["vendor_imu_topic"] == "/livox/imu"
    assert lidar["imu_topic"] == "/mid360/imu/data_raw"
    assert lidar["acceleration_input_unit"] == "g"
    assert lidar["acceleration_output_unit"] == "m/s^2"
    assert lidar["simulation_compatibility_frame"] == "front_3d_lidar"
    assert lidar["simulation_rtx_origin_from_housing_bottom_m"] == [
        0.0,
        0.0,
        0.047,
    ]


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
        assert scope in leaves or any(
            path.startswith(f"{scope}.") for path in leaves
        )
    for path in leaves:
        matches = [
            scope
            for scope in scopes
            if path == scope or path.startswith(f"{scope}.")
        ]
        assert len(matches) == 1, (
            f"{path} must have exactly one provenance scope; found {matches}"
        )
