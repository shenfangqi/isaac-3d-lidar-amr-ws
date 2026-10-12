"""Check eight ToF modules fit on and face away from the green plate."""

import math

import pytest
import yaml

from isaac_sim.carbot_vl53l5cx_mount import vl53l5cx_placements


def test_eight_edge_mounts_have_outward_axes_and_touch_plate():
    with open("src/carbot_description/config/carbot_parameters.yaml") as stream:
        parameters = yaml.safe_load(stream)
    plate = parameters["geometry"]["body_visual_base_plate"]
    config = parameters["sensors"]["vl53l5cx_sim_mount"]
    poses = vl53l5cx_placements(plate, config)
    assert len(poses) == len({pose[0] for pose in poses}) == 8
    assert {name.split("_")[0] for name, *_ in poses} == {
        "front", "rear", "left", "right"
    }
    expected_yaw = {
        "front_left": 12.5, "front_right": -12.5,
        "rear_left": 157.5, "rear_right": -157.5,
        "left_front": 67.5, "left_rear": 112.5,
        "right_front": -67.5, "right_rear": -112.5,
    }
    plate_top = plate["position_from_base_link_m"][2] + plate["size_m"][2] / 2
    for name, x, y, z, yaw in poses:
        assert yaw == expected_yaw[name]
        assert abs(x) < plate["size_m"][0] / 2
        assert abs(y) < plate["size_m"][1] / 2
        assert z - config["board_size_m"][2] / 2 == pytest.approx(plate_top)
        assert math.isfinite(x + y + z + yaw)
