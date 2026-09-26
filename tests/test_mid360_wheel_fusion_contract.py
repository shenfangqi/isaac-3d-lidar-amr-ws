from pathlib import Path

import yaml
import pytest

from scripts.audit_mid360_wheel_fusion_bag import motion_ticks, robust_rate_hz


ROOT = Path(__file__).resolve().parents[1]


def test_mid360_is_the_only_ekf_imu_source():
    config = yaml.safe_load(
        (ROOT / "src/carbot_hardware/config/mid360_wheel_ekf.yaml").read_text(
            encoding="utf-8"
        )
    )["ekf_filter_node"]["ros__parameters"]
    assert config["imu0"] == "/mid360/imu/data_raw"
    assert all(value != "/imu/data_raw" for value in config.values())


def test_ekf_owns_standard_odom_without_duplicate_wheel_tf():
    ekf = yaml.safe_load(
        (ROOT / "src/carbot_hardware/config/mid360_wheel_ekf.yaml").read_text(
            encoding="utf-8"
        )
    )["ekf_filter_node"]["ros__parameters"]
    wheel = yaml.safe_load(
        (
            ROOT
            / "src/carbot_hardware/config/wheel_odometry_fused.yaml"
        ).read_text(encoding="utf-8")
    )["carbot_wheel_odometry"]["ros__parameters"]
    assert ekf["odom0"] == "/wheel/odom"
    assert wheel["odom_topic"] == "/wheel/odom"
    assert wheel["publish_tf"] is False
    assert ekf["publish_tf"] is True


def test_ekf_fuses_wheel_translation_and_mid360_yaw_rate_only():
    config = yaml.safe_load(
        (ROOT / "src/carbot_hardware/config/mid360_wheel_ekf.yaml").read_text(
            encoding="utf-8"
        )
    )["ekf_filter_node"]["ros__parameters"]
    enabled_wheel = [index for index, value in enumerate(config["odom0_config"]) if value]
    enabled_imu = [index for index, value in enumerate(config["imu0_config"]) if value]
    assert enabled_wheel == [5, 6, 7]
    assert enabled_imu == [11]


def test_rate_uses_median_positive_period():
    assert robust_rate_hz([1.0, 1.005, 1.010, 1.015]) == pytest.approx(200.0)


def test_motion_tick_delta():
    assert motion_ticks((10, -2), (25, 7)) == {"left": 15, "right": 9}
