"""Check eight ToF modules fit on and face away from the green plate."""

import math

import pytest
import yaml

from isaac_sim.carbot_vl53l5cx_mount import vl53l5cx_placements


def test_eight_low_perimeter_mounts_have_outward_axes_and_clear_tracks():
    with open("src/carbot_description/config/carbot_parameters.yaml") as stream:
        parameters = yaml.safe_load(stream)
    geometry = parameters["geometry"]
    config = parameters["sensors"]["vl53l5cx_sim_mount"]
    poses = vl53l5cx_placements(geometry, config)
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
    footprint = geometry["footprint_m"]
    front = max(point[0] for point in footprint)
    rear = min(point[0] for point in footprint)
    left = max(point[1] for point in footprint)
    right = min(point[1] for point in footprint)
    optical_centres = {}
    for name, x, y, z, yaw in poses:
        assert yaw == expected_yaw[name]
        yaw_rad = math.radians(yaw)
        optical_centres[name] = (
            x + math.cos(yaw_rad) * config["board_size_m"][0] / 2,
            y + math.sin(yaw_rad) * config["board_size_m"][0] / 2,
            z,
        )
        assert z + geometry["base_link_height_m"] == pytest.approx(
            config["optical_center_height_from_ground_m"]
        )
        assert math.isfinite(x + y + z + yaw)
    inset = config["optical_center_footprint_inset_m"]
    assert optical_centres["front_left"][0] == pytest.approx(front - inset)
    assert optical_centres["front_right"][0] == pytest.approx(front - inset)
    assert optical_centres["rear_left"][0] == pytest.approx(rear + inset)
    assert optical_centres["rear_right"][0] == pytest.approx(rear + inset)
    assert optical_centres["left_front"][1] == pytest.approx(left - inset)
    assert optical_centres["left_rear"][1] == pytest.approx(left - inset)
    assert optical_centres["right_front"][1] == pytest.approx(right + inset)
    assert optical_centres["right_rear"][1] == pytest.approx(right + inset)
