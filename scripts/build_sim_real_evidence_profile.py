#!/usr/bin/env python3
"""Build the offline evidence-constrained Carbot simulation profile."""

import argparse
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CANONICAL = ROOT / "src/carbot_description/config/carbot_parameters.yaml"
TURN_REPEAT = (
    ROOT
    / "calibration_data/2026-09-22_ekf_dynamic"
    / "turn_repeat_5pairs_03_analysis.json"
)
FINAL_ACCEPTANCE = (
    ROOT
    / "calibration_data/2026-09-23_cmd_comp_final"
    / "physical_final_04_analysis.json"
)
IMU_DYNAMIC = (
    ROOT
    / "calibration_data/2026-09-22_ekf_dynamic"
    / "controlled_mid360_wheel_cmd_01_analysis.json"
)
IMU_STATIC = (
    ROOT
    / "calibration_data/2026-09-21_static_mid360"
    / "mid360_static_analysis.json"
)
OUTPUT = ROOT / "configs/carbot/evidence_degraded.yaml"


def load(path):
    with path.open(encoding="utf-8") as stream:
        if path.suffix == ".json":
            return json.load(stream)
        return yaml.safe_load(stream)


def build_profile():
    canonical = load(CANONICAL)
    repeat = load(TURN_REPEAT)
    final = load(FINAL_ACCEPTANCE)
    dynamic = load(IMU_DYNAMIC)
    static = load(IMU_STATIC)
    control = canonical["control"]
    hardware = canonical["hardware_response"]
    kinematics = canonical["kinematics"]
    mid360 = canonical["sensors"]["mid360"]

    # The accepted Isaac test used the same three-second command and ramp.
    # Ratios therefore retain observed finite-duration response, not a claimed
    # motor steady-state transfer function.
    isaac_turn_deg = 51.566200949346886
    left_gain = final["left_angle_deg"] / isaac_turn_deg
    right_gain = abs(final["right_angle_deg"]) / isaac_turn_deg
    stop_bounds = hardware["timing"]["ground_stop_tail_s"]
    stop_tail = sum(stop_bounds) / len(stop_bounds)
    reference_speed = 0.05
    stationary_threshold = 0.001
    stop_tau = stop_tail / math.log(reference_speed / stationary_threshold)

    point_timing = static["pointcloud_timing_s"]["bag_receive_minus_header"]
    imu_timing = static["mid360_imu_si"]["bag_receive_minus_header_s"]
    gyro = dynamic["gyro_z_stationary"]
    return {
        "schema_version": 1,
        "status": "EVIDENCE_CONSTRAINED_WOOD_FLOOR_ZERO_PAYLOAD",
        "limitations": [
            "not a calibrated mass/friction/contact dynamics twin",
            "finite-duration gains are specific to wood floor and zero payload",
            "PWM, motor RPM, voltage sag and current were not recorded together",
        ],
        "sources": [
            str(path.relative_to(ROOT))
            for path in (
                CANONICAL,
                TURN_REPEAT,
                FINAL_ACCEPTANCE,
                IMU_DYNAMIC,
                IMU_STATIC,
            )
        ],
        "actuator": {
            "command_latency_s": hardware["timing"][
                "first_motion_latency_s"
            ],
            "rise_time_constant_s": 0.12,
            "rise_time_constant_status": "bounded_assumption_from_existing_steps",
            "stop_tail_s": stop_tail,
            "stop_tail_range_s": stop_bounds,
            "stop_time_constant_s": stop_tau,
            "stationary_threshold_mps": stationary_threshold,
            "linear_gain": {
                "forward": 0.882,
                "reverse": 0.875,
                "status": "surveyed_0p05_mps_10s_trials",
            },
            "yaw_gain": {
                "left": left_gain,
                "right_after_compensation": right_gain,
                "status": "2026_09_23_three_second_acceptance",
            },
            "right_turn_command_scale": control[
                "right_turn_command_scale"
            ],
            "uncompensated_right_over_left_response": repeat["summary"][
                "right_over_left_response"
            ],
            "track_deadband_mps": {
                "forward": hardware["track_deadband"][
                    "forward_min_sustainable_mps"
                ],
                "reverse": hardware["track_deadband"][
                    "reverse_min_sustainable_mps"
                ],
            },
        },
        "encoder": {
            "counts_per_revolution": kinematics[
                "encoder_counts_per_revolution"
            ],
            "publish_rate_hz": canonical["interfaces"]["wheel_ticks"][
                "rate_hz"
            ],
            "drop_fraction": 0.00022,
            "duplicate_fraction": 0.000067,
            "maximum_observed_receive_gap_s": 0.345,
            "seed": 20260923,
        },
        "imu": {
            "publish_rate_hz": mid360["imu_rate_hz"],
            "gyro_z_noise_stddev_rad_s": gyro["noise_stddev_rad_s"],
            "gyro_z_residual_bias_rad_s": 0.00039022790315293063,
            "gyro_z_covariance_rad2_s2": gyro[
                "configured_covariance_floor_rad2_s2"
            ],
            "receive_latency_mean_s": imu_timing["mean"],
            "receive_latency_stddev_s": imu_timing["std"],
            "timestamp_correction_s": canonical[
                "sensor_frame_assumptions"
            ]["imu_timestamp_correction_s"],
            "seed": 20260924,
        },
        "pointcloud": {
            "publish_rate_hz": mid360["pointcloud_rate_hz"],
            "range_noise_stddev_m": 0.03,
            "point_dropout_fraction": 0.02,
            "observed_points_per_frame": mid360[
                "observed_points_per_frame"
            ],
            "receive_latency_mean_s": point_timing["mean"],
            "receive_latency_stddev_s": point_timing["std"],
            "point_timestamp_span_mean_s": static["pointcloud_timing_s"][
                "point_timestamp_span"
            ]["mean"],
            "seed": 20260925,
        },
        "state_estimation": {
            "wheel_odom_topic": "/wheel/odom",
            "imu_topic": "/mid360/imu/data_raw",
            "filtered_odom_topic": "/odom",
            "ground_truth_topic": "/ground_truth/odom",
            "ekf_config": "src/carbot_hardware/config/mid360_wheel_ekf.yaml",
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    rendered = yaml.safe_dump(build_profile(), sort_keys=False)
    if args.check:
        if not args.output.is_file() or args.output.read_text() != rendered:
            raise SystemExit(f"stale evidence profile: {args.output}")
        print(f"evidence profile is current: {args.output}")
        return
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(rendered, encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
