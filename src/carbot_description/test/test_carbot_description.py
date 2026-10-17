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


def test_body_visual_and_base_plate_are_visual_only(description, parameters):
    base_link = next(
        link for link in description.findall("link")
        if link.attrib["name"] == "base_link"
    )
    visuals = {visual.attrib["name"]: visual for visual in base_link.findall("visual")}
    blue = visuals["body_visual"]
    plate = visuals["body_visual_base_plate"]
    blue_size = [float(v) for v in blue.find("geometry/box").attrib["size"].split()]
    plate_size = [float(v) for v in plate.find("geometry/box").attrib["size"].split()]
    blue_z = float(blue.find("origin").attrib["xyz"].split()[2])
    plate_z = float(plate.find("origin").attrib["xyz"].split()[2])
    blue_x = float(blue.find("origin").attrib["xyz"].split()[0])
    plate_x = float(plate.find("origin").attrib["xyz"].split()[0])
    assert blue_size == pytest.approx([0.11, 0.15, 0.0589])
    assert plate_size == pytest.approx([0.225, 0.213, 0.01])
    assert plate_x + plate_size[0] / 2 - (
        blue_x + blue_size[0] / 2
    ) == pytest.approx(0.059)
    assert plate_z + plate_size[2] / 2 == pytest.approx(blue_z - blue_size[2] / 2)
    geometry = parameters["geometry"]
    front_idler = geometry["wheel_groups"]["front_idler"]
    track_top_z = (
        front_idler["z_m"] + front_idler["radius_m"]
        + geometry["track_thickness_m"] / 2
    )
    assert plate_z - plate_size[2] / 2 - track_top_z == pytest.approx(0.02)
    assert blue_z + blue_size[2] / 2 == pytest.approx(0.07)
    assert plate.find("material").attrib["name"] == "base_plate_green"
    assert len(base_link.findall("collision")) == 1
    collision = base_link.find("collision")
    assert collision.attrib["name"] == "body_collision"
    assert [float(v) for v in collision.find("geometry/box").attrib["size"].split()] == pytest.approx(
        parameters["geometry"]["body_collision"]["size_m"]
    )


def test_track_visual_names_are_unique_for_usd_import(description):
    visual_names = [
        visual.attrib["name"]
        for link in description.findall("link")
        for visual in link.findall("visual")
        if "belt" in visual.attrib.get("name", "")
    ]
    assert set(visual_names) == {
        "left_lower_belt", "left_upper_belt",
        "right_lower_belt", "right_upper_belt",
    }


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
    visuals = {
        visual.attrib["name"]: visual for visual in lidar_link.findall("visual")
    }
    assert set(visuals) == {
        "mid360_mount_visual",
        "mid360_body_visual",
        "mid360_scan_window_visual",
    }
    proxy = lidar["visual_proxy"]
    mount_size = [
        float(value) for value in
        visuals["mid360_mount_visual"].find("geometry/box").attrib["size"].split()
    ]
    body_size = [
        float(value) for value in
        visuals["mid360_body_visual"].find("geometry/box").attrib["size"].split()
    ]
    scan = visuals["mid360_scan_window_visual"].find("geometry/cylinder")
    scan_height = float(scan.attrib["length"])
    scan_radius = float(scan.attrib["radius"])
    assert mount_size == pytest.approx(proxy["mount_size_m"])
    assert body_size == pytest.approx(proxy["body_size_m"])
    assert scan_height == pytest.approx(proxy["scan_window_height_m"])
    assert scan_radius == pytest.approx(proxy["scan_window_radius_m"])
    assert mount_size[2] + body_size[2] + scan_height == pytest.approx(
        lidar["housing_height_m"]
    )

    housing_center_z = float(
        visuals["mid360_scan_window_visual"]
        .find("origin").attrib["xyz"].split()[2]
    )
    modeled_top_height = (
        geometry["base_link_height_m"]
        + joint_xyz[2]
        + housing_center_z
        + scan_height / 2.0
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
    plate = parameters["geometry"]["body_visual_base_plate"]
    plate_front_x = plate["position_from_base_link_m"][0] + plate["size_m"][0] / 2
    assert plate_front_x - lidar["mount_translation_xy_from_base_link_m"][0] == pytest.approx(
        0.110
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
