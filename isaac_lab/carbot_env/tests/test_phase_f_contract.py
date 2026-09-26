from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from isaac_lab.carbot_env.spec import (
    ENVIRONMENT_SPEC_PATH,
    PROJECT_ROOT,
    load_environment_spec,
    load_hardware_calibration_backlog,
    load_robot_parameters,
)
from isaac_sim.carbot_control import CarbotCommandLimiter, ControlLimits


TASK_PATH = PROJECT_ROOT / "isaac_lab/carbot_env/tasks/goal_navigation.yaml"


@pytest.fixture(scope="module")
def spec():
    return load_environment_spec()


def test_policy_action_is_deployable_high_level_twist(spec):
    action = spec["action"]
    assert action["interface"] == "high_level_twist"
    assert action["fields"] == ["linear_x_mps", "angular_z_rad_s"]
    assert action["units"] == "physical"
    assert action["normalized"] is False
    assert action["forbidden_policy_outputs"] == ["pwm", "motor_torque"]
    assert action["reserved_execution_modes"]["left_right_track_velocity"][
        "enabled"
    ] is False
    assert spec["simulation"]["control_mode"] == (
        "ideal_kinematic_tracked_differential"
    )


def test_limits_and_randomization_resolve_from_single_parameter_sources(spec):
    robot = load_robot_parameters()
    with (PROJECT_ROOT / "configs/carbot/real.yaml").open(
        encoding="utf-8"
    ) as stream:
        deployment = yaml.safe_load(stream)
    limits = spec["action"]["limits"]
    assert limits["linear_velocity_mps"] == deployment["velocity_limits"][
        "linear_mps"
    ]
    assert limits["angular_velocity_rad_s"] == deployment["velocity_limits"][
        "angular_rad_s"
    ]
    assert limits["linear_acceleration_mps2"] == deployment[
        "acceleration_limits"
    ]["linear_mps2"]
    assert limits["angular_acceleration_rad_s2"] == deployment[
        "acceleration_limits"
    ]["angular_rad_s2"]
    assert limits["watchdog_timeout_s"] == robot["control"][
        "cmd_vel_timeout_s"
    ]
    assert spec["domain_randomization"] == robot["domain_randomization"]
    assert spec["hardware_response"] == robot["hardware_response"]
    assert spec["action"]["hardware_response"]["track_deadband"] == {
        "forward_min_sustainable_mps": 0.05,
        "reverse_min_sustainable_mps": 0.02,
        "reliable_in_place_angular_command_rad_s": [0.40, 0.50],
    }
    assert spec["domain_randomization"]["calibration_status"] == (
        "PARTIAL_PHYSICAL_EVIDENCE_WITH_UNCALIBRATED_DYNAMICS"
    )
    assert spec["evidence_profile"]["status"].startswith(
        "EVIDENCE_CONSTRAINED"
    )
    assert robot["dynamics"]["per_side_effort_limit_nm"] / 6.0 == (
        pytest.approx(0.08333333333333333)
    )


def test_policy_observations_do_not_depend_on_isaac_ground_truth(spec):
    observations = spec["observations"]
    assert observations["policy_order"] == [
        "goal_pose_body",
        "planar_velocity",
        "projected_gravity",
        "lidar_ranges",
        "height_scan",
        "previous_action",
    ]
    assert observations["lidar"]["ray_count"] == 72
    assert observations["height_scan"]["grid_size"] == [5, 5]
    assert "isaac_ground_truth_pose" in observations[
        "forbidden_policy_observations"
    ]


def test_task_and_curriculum_are_asset_independent(spec):
    assert spec["task"]["coupled_to_robot_usd"] is False
    with TASK_PATH.open(encoding="utf-8") as stream:
        task = yaml.safe_load(stream)
    assert task["name"] == "goal_navigation"
    assert task["curriculum"]["enabled"] is False
    assert "straight and turning motion" in task["curriculum"][
        "enable_after"
    ]


def test_hardware_backlog_blocks_policy_release():
    backlog = load_hardware_calibration_backlog()
    assert backlog["status"] == "BLOCKING_SIM_TO_REAL_CALIBRATION"
    assert backlog["policy_release_blocked"] is True
    items = {item["id"]: item for item in backlog["items"]}
    blockers = {
        item_id
        for item_id, item in items.items()
        if item["blocking_calibration"]
    }
    assert items["effective_radius"]["state"] == "baseline_confirmed"
    assert items["effective_radius"]["blocking_calibration"] is False
    assert items["effective_radius"]["release_verification"] == (
        "wood_floor_no_payload_complete"
    )
    assert items["effective_track_separation"]["state"] == (
        "baseline_confirmed"
    )
    assert items["effective_track_separation"]["blocking_calibration"] is False
    assert items["effective_track_separation"]["release_verification"] == (
        "wood_floor_no_payload_complete"
    )
    assert {
        "track_slip_surface_variation",
        "actuator_latency_deadband_braking",
        "command_watchdog_and_estop",
        "observation_parity",
        "real_navigation_acceptance",
    }.issubset(blockers)
    assert items["mid360_origin_and_extrinsics"]["state"] == (
        "lidar_and_imu_extrinsics_calibrated"
    )
    assert (
        items["mid360_origin_and_extrinsics"]["blocking_calibration"]
        is False
    )
    assert items["jetson_wheel_ticks_decode"]["state"] == (
        "physical_verified"
    )
    assert items["jetson_wheel_ticks_decode"]["blocking_calibration"] is False
    assert items["jetson_odometry_owner"]["state"] == "physical_verified"
    assert items["jetson_odometry_owner"]["blocking_calibration"] is False
    assert items["real_navigation_acceptance"]["state"] == (
        "supervised_1p4m_route_verified_hardware_estop_pending"
    )


def test_safety_boundary_matches_existing_cmd_vel_limiter(spec):
    limits = ControlLimits.from_parameters(load_robot_parameters())
    phase_f = spec["action"]["limits"]
    limits = replace(
        limits,
        max_linear_velocity_mps=phase_f["linear_velocity_mps"],
        max_angular_velocity_rad_s=phase_f["angular_velocity_rad_s"],
    )
    limiter = CarbotCommandLimiter(limits)
    command = limiter.update(1.0, 1.0, command_age_s=0.0, dt_s=1.0)
    assert command.requested_linear_mps == phase_f["linear_velocity_mps"]
    assert command.requested_angular_rad_s == phase_f[
        "angular_velocity_rad_s"
    ]
    assert max(
        abs(command.left_wheel_rad_s), abs(command.right_wheel_rad_s)
    ) <= limits.max_wheel_velocity_rad_s


def test_watchdog_uses_canonical_half_second_timeout():
    limits = ControlLimits.from_parameters(load_robot_parameters())
    limiter = CarbotCommandLimiter(limits)
    limiter.update(0.2, 0.1, command_age_s=0.0, dt_s=0.1)
    command = limiter.update(0.2, 0.1, command_age_s=0.51, dt_s=0.1)
    assert limits.cmd_vel_timeout_s == 0.5
    assert command.watchdog_active is True
    assert command.requested_linear_mps == 0.0
    assert command.requested_angular_rad_s == 0.0


def test_spec_paths_exist_and_are_inside_repository():
    assert ENVIRONMENT_SPEC_PATH.is_file()
    assert TASK_PATH.is_file()
    assert Path(load_environment_spec()["simulation"]["robot_usd"]).suffix == (
        ".usd"
    )
