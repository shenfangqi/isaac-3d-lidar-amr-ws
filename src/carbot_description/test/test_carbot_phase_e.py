from pathlib import Path

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = DESCRIPTION_ROOT.parents[1]
CONFIG_ROOT = WORKSPACE / "configs"
LAUNCH_ROOT = WORKSPACE / "launch"


def load_yaml(relative_path):
    return yaml.safe_load((WORKSPACE / relative_path).read_text(encoding="utf-8"))


def nav_parameters(runtime):
    return load_yaml(f"configs/nav2_params_{runtime}.yaml")


def test_runtime_profiles_separate_simulation_from_real_hardware():
    simulation = load_yaml("configs/carbot/sim.yaml")
    real = load_yaml("configs/carbot/real.yaml")

    assert simulation["use_sim_time"] is True
    assert simulation["odom_source"] == "isaac_direct"
    assert simulation["allow_chassis_odom_relay"] is False
    assert simulation["allow_ground_truth_map_to_odom"] is True

    assert real["use_sim_time"] is False
    assert real["odom_source"] == "jetson_wheel_ticks"
    assert real["allow_chassis_odom_relay"] is False
    assert real["allow_ground_truth_map_to_odom"] is False
    assert real["amcl_initial_pose_default"] == "manual"


def test_nav2_footprints_match_the_canonical_carbot_polygon():
    canonical = load_yaml(
        "src/carbot_description/config/carbot_parameters.yaml"
    )["geometry"]["footprint_m"]

    for runtime in ("sim", "real"):
        parameters = nav_parameters(runtime)
        for costmap in ("local_costmap", "global_costmap"):
            node = parameters[costmap][costmap]["ros__parameters"]
            assert node["robot_base_frame"] == "base_footprint"
            assert "robot_radius" not in node
            assert yaml.safe_load(node["footprint"]) == canonical
            assert node["inflation_layer"]["inflation_radius"] == 0.45


def test_nav2_time_and_velocity_limits_are_runtime_specific():
    expected = {
        "sim": (True, 0.30, 0.35),
        "real": (False, 0.10, 0.30),
    }
    for runtime, (use_sim_time, linear, angular) in expected.items():
        parameters = nav_parameters(runtime)
        assert parameters["controller_server"]["ros__parameters"][
            "use_sim_time"
        ] is use_sim_time
        controller = parameters["controller_server"]["ros__parameters"][
            "FollowPath"
        ]
        assert controller["desired_linear_vel"] == linear
        assert controller["rotate_to_heading_angular_vel"] == angular
        smoother = parameters["velocity_smoother"]["ros__parameters"]
        assert smoother["max_velocity"] == [linear, 0.0, angular]


def test_amcl_uses_base_footprint_and_runtime_time_source():
    for runtime, use_sim_time in (("sim", True), ("real", False)):
        parameters = load_yaml(f"configs/amcl_params_{runtime}.yaml")
        amcl = parameters["amcl"]["ros__parameters"]
        assert amcl["use_sim_time"] is use_sim_time
        assert amcl["base_frame_id"] == "base_footprint"
        assert amcl["odom_frame_id"] == "odom"


def test_launch_contract_has_no_legacy_odom_relay():
    shared = (LAUNCH_ROOT / "carbot_navigation.py").read_text(encoding="utf-8")
    simulation = (LAUNCH_ROOT / "carbot_sim.launch.py").read_text(
        encoding="utf-8"
    )
    real = (LAUNCH_ROOT / "carbot_real.launch.py").read_text(encoding="utf-8")

    assert "/chassis/odom" not in shared
    assert "topic_tools" not in shared
    assert '"sim"' in simulation
    assert '"real"' in real
    assert '"amcl"' in real
    assert "ground_truth" not in real


def test_common_runtime_config_points_to_canonical_geometry():
    common = load_yaml("configs/carbot/common.yaml")
    assert common["frames"]["robot_base"] == "base_footprint"
    assert common["topics"]["odom"] == "/odom"
    assert common["topics"]["pointcloud"] == "/livox/lidar"
    assert common["maps"]["nav2_yaml"].endswith("warehouse_v3.yaml")
    assert "geometry" not in common
