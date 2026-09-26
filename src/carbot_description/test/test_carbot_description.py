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
    assert parents["imu_link"] == "livox_frame"


def test_imu_link_uses_calibrated_mid360_chip_transform(
    description, parameters
):
    imu_joint = next(
        joint for joint in description.findall("joint")
        if joint.attrib["name"] == "lidar_link_to_imu_link"
    )
    origin = imu_joint.find("origin")
    xyz = [float(value) for value in origin.attrib["xyz"].split()]
    rpy = [float(value) for value in origin.attrib["rpy"].split()]
    calibration = parameters["sensor_frame_assumptions"]
    assert xyz == pytest.approx(calibration["imu_translation_m"])
    assert rpy == pytest.approx(calibration["imu_rotation_rpy_rad"])


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


def test_lidar_housing_top_matches_measured_height(description, parameters):
    geometry = parameters["geometry"]
    lidar = parameters["sensors"]["mid360"]
    assert lidar["mount_translation_xy_from_base_link_m"] == pytest.approx(
        [0.01656608, -0.00014467]
    )
    assert lidar["rotation_rpy_rad"] == pytest.approx(
        [-0.006135404, -0.004857536, 0.019559477]
    )
    expected_mount_z = (
        lidar["housing_top_height_from_ground_m"]
        - lidar["housing_height_m"]
        - geometry["base_link_height_m"]
    )
    assert expected_mount_z == pytest.approx(0.072)

    lidar_joint = next(
        joint for joint in description.findall("joint")
        if joint.attrib["name"] == "base_link_to_lidar_link"
    )
    joint_xyz = [
        float(value) for value in lidar_joint.find("origin").attrib["xyz"].split()
    ]
    assert joint_xyz[:2] == pytest.approx(
        lidar["mount_translation_xy_from_base_link_m"]
    )
    assert joint_xyz[2] == pytest.approx(expected_mount_z)
    joint_rpy = [
        float(value)
        for value in lidar_joint.find("origin").attrib["rpy"].split()
    ]
    assert joint_rpy == pytest.approx(lidar["rotation_rpy_rad"])

    lidar_link = next(
        link for link in description.findall("link")
        if link.attrib["name"] == "lidar_link"
    )
    housing = next(
        visual for visual in lidar_link.findall("visual")
        if visual.attrib["name"] == "mid360_proxy_visual"
    )
    housing_height = float(
        housing.find("geometry/cylinder").attrib["length"]
    )
    housing_center_z = float(housing.find("origin").attrib["xyz"].split()[2])
    assert housing_height == pytest.approx(0.060)
    assert housing_center_z == pytest.approx(housing_height / 2.0)
    modeled_top_height = (
        geometry["base_link_height_m"]
        + joint_xyz[2]
        + housing_center_z
        + housing_height / 2.0
    )
    assert modeled_top_height == pytest.approx(0.222)

    fixed_joints = {
        joint.attrib["name"]: joint for joint in description.findall("joint")
    }
    expected_origin_z = lidar[
        "coordinate_origin_height_from_housing_bottom_m"
    ]
    for name in (
        "lidar_link_to_livox_frame",
        "lidar_link_to_front_3d_lidar",
    ):
        xyz = [
            float(value)
            for value in fixed_joints[name].find("origin").attrib["xyz"].split()
        ]
        assert xyz == pytest.approx([0.0, 0.0, expected_origin_z])

    livox_height_from_ground = (
        geometry["base_link_height_m"] + expected_mount_z + expected_origin_z
    )
    assert livox_height_from_ground == pytest.approx(0.209)


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
