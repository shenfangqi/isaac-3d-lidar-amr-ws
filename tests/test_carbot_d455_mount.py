"""Control limits for the visual D455 gimbal."""

import math

import pytest
import yaml

from isaac_sim.carbot_d455_mount import clamp_gimbal_angles


def test_gimbal_limits_and_invalid_command():
    with open("src/carbot_description/config/carbot_parameters.yaml") as stream:
        parameters = yaml.safe_load(stream)
    config = parameters["sensors"]["d455_sim_mount"]
    plate = parameters["geometry"]["body_visual_base_plate"]
    plate_top = plate["position_from_base_link_m"][2] + plate["size_m"][2] / 2
    camera_center = (
        config["yaw_pivot_xyz_m"][2] + config["mast_height_m"]
        + config["camera_center_from_pitch_xyz_m"][2]
    )
    assert config["yaw_pivot_xyz_m"][0] == pytest.approx(
        plate["position_from_base_link_m"][0] + plate["size_m"][0] / 2
    )
    assert config["yaw_pivot_xyz_m"][1] == pytest.approx(0.0)
    assert camera_center - plate_top == pytest.approx(0.05)
    assert config["initial_pitch_rad"] == pytest.approx(math.pi / 12)
    yaw, pitch = clamp_gimbal_angles(config, math.pi, -math.pi / 2)
    assert yaw == pytest.approx(math.pi / 2)
    assert pitch == pytest.approx(-math.pi / 12)
    with pytest.raises(ValueError):
        clamp_gimbal_angles(config, float("nan"), 0.0)
