import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
PARAMETER_FILE = ROOT / "config" / "carbot_parameters.yaml"
GENERATED_URDF = Path(os.environ.get("CARBOT_GENERATED_URDF", ""))


@pytest.fixture(scope="module")
def description():
    if not os.environ.get("CARBOT_GENERATED_URDF") or not GENERATED_URDF.exists():
        pytest.skip("CARBOT_GENERATED_URDF is provided by the CMake test fixture")
    return ET.parse(GENERATED_URDF).getroot()


@pytest.fixture(scope="module")
def parameters():
    return yaml.safe_load(PARAMETER_FILE.read_text(encoding="utf-8"))


def test_required_frame_tree_exists(description):
    links = {link.attrib["name"] for link in description.findall("link")}
    assert {
        "base_footprint",
        "base_link",
        "lidar_link",
        "livox_frame",
        "front_3d_lidar",
        "imu_link",
    } <= links

    parents = {
        joint.find("child").attrib["link"]: joint.find("parent").attrib["link"]
        for joint in description.findall("joint")
    }
    assert parents["base_link"] == "base_footprint"
    assert parents["lidar_link"] == "base_link"
    assert parents["livox_frame"] == "lidar_link"
    assert parents["front_3d_lidar"] == "lidar_link"
    assert parents["imu_link"] == "lidar_link"


def test_twelve_wheel_joints_are_continuous_and_never_steering(description):
    joints = description.findall("joint")
    wheel_joints = [joint for joint in joints if joint.attrib["name"].endswith("_wheel_joint")]
    assert len(wheel_joints) == 12
    assert all(joint.attrib["type"] == "continuous" for joint in wheel_joints)
    assert not any("steer" in joint.attrib["name"].lower() for joint in joints)
    assert not any("ackermann" in joint.attrib["name"].lower() for joint in joints)
    assert all(joint.find("axis").attrib["xyz"] == "0 -1 0" for joint in wheel_joints)


def test_wheel_joint_limits_match_common_parameters(description, parameters):
    wheel_joints = [
        joint
        for joint in description.findall("joint")
        if joint.attrib["name"].endswith("_wheel_joint")
    ]
    expected_effort = parameters["dynamics"]["per_side_effort_limit_nm"] / 6.0
    expected_velocity = parameters["control"]["max_wheel_velocity_rad_s"]
    for joint in wheel_joints:
        limit = joint.find("limit")
        assert float(limit.attrib["effort"]) == pytest.approx(expected_effort)
        assert float(limit.attrib["velocity"]) == pytest.approx(expected_velocity)


def test_body_visual_does_not_cover_tracks(description, parameters):
    base_link = next(
        link for link in description.findall("link")
        if link.attrib["name"] == "base_link"
    )
    visual_size = [
        float(value)
        for value in base_link.find("visual/geometry/box").attrib["size"].split()
    ]
    geometry = parameters["geometry"]
    inner_track_edge = (
        geometry["physical_track_separation_m"] - geometry["track_width_m"]
    ) / 2.0
    assert visual_size[1] / 2.0 < inner_track_edge


def test_lidar_mount_proxy_fills_gap_without_changing_sensor_origin(
    description, parameters
):
    geometry = parameters["geometry"]
    lidar = parameters["sensors"]["mid360"]
    body_visual = geometry["body_visual"]
    expected_height = lidar["translation_from_base_link_m"][2] - (
        body_visual["position_from_base_link_m"][2]
        + body_visual["size_m"][2] / 2.0
    )
    assert expected_height > 0.0

    lidar_joint = next(
        joint for joint in description.findall("joint")
        if joint.attrib["name"] == "base_link_to_lidar_link"
    )
    joint_z = float(lidar_joint.find("origin").attrib["xyz"].split()[2])
    assert joint_z == pytest.approx(lidar["translation_from_base_link_m"][2])

    lidar_link = next(
        link for link in description.findall("link")
        if link.attrib["name"] == "lidar_link"
    )
    mount = next(
        visual for visual in lidar_link.findall("visual")
        if visual.attrib["name"] == "mid360_mount_proxy_visual"
    )
    assert float(mount.find("geometry/cylinder").attrib["length"]) == (
        pytest.approx(expected_height)
    )
    assert float(mount.find("origin").attrib["xyz"].split()[2]) == (
        pytest.approx(-expected_height / 2.0)
    )


def test_wheel_centers_use_physical_geometry(description, parameters):
    expected_y = parameters["geometry"]["physical_track_separation_m"] / 2.0
    joints = {
        joint.attrib["name"]: joint
        for joint in description.findall("joint")
        if joint.attrib["name"].endswith("_wheel_joint")
    }
    for name, joint in joints.items():
        xyz = [float(value) for value in joint.find("origin").attrib["xyz"].split()]
        assert abs(xyz[1]) == pytest.approx(expected_y)
        assert math.copysign(1.0, xyz[1]) == (1.0 if name.startswith("left_") else -1.0)


def test_each_wheel_has_collision_geometry(description):
    wheel_links = [
        link for link in description.findall("link")
        if link.attrib["name"].endswith("_wheel_link")
    ]
    assert len(wheel_links) == 12
    assert all(link.find("collision/geometry/cylinder") is not None for link in wheel_links)


def test_total_declared_mass_matches_working_mass(description, parameters):
    masses = [
        float(mass.attrib["value"])
        for mass in description.findall("link/inertial/mass")
    ]
    assert sum(masses) == pytest.approx(parameters["robot"]["total_working_mass_kg"])
