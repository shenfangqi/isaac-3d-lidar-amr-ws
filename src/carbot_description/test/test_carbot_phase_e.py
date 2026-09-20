from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = DESCRIPTION_ROOT.parents[1]
CONFIG_ROOT = WORKSPACE / "configs"
LAUNCH_ROOT = WORKSPACE / "launch"


def load_yaml(relative_path):
    return yaml.safe_load(
        (WORKSPACE / relative_path).read_text(encoding="utf-8")
    )


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


def test_sim_global_planning_uses_stable_scan_and_conditional_replanning():
    common = load_yaml("configs/carbot/common.yaml")
    simulation = nav_parameters("sim")
    real = nav_parameters("real")

    local_scan = simulation["local_costmap"]["local_costmap"][
        "ros__parameters"
    ]["obstacle_layer"]["scan"]["topic"]
    global_scan = simulation["global_costmap"]["global_costmap"][
        "ros__parameters"
    ]["obstacle_layer"]["scan"]["topic"]
    assert local_scan == common["topics"]["scan"] == "/scan"
    assert global_scan == common["topics"]["global_scan"]
    assert common["topics"]["global_scan_static_filtered"] == (
        "/scan_global_static_filtered"
    )

    behavior_tree = simulation["bt_navigator"]["ros__parameters"][
        "default_nav_to_pose_bt_xml"
    ]
    assert behavior_tree.endswith(
        "configs/behavior_trees/"
        "navigate_to_pose_replan_after_controller_failure.xml"
    )
    assert simulation["planner_server"]["ros__parameters"]["GridBased"][
        "tolerance"
    ] == 0.0

    tree = ET.parse(behavior_tree).getroot()
    assert tree.find(".//RecoveryNode[@name='NavigateRecovery']") is None
    assert tree.find(".//ReactiveFallback[@name='RecoveryFallback']") is None
    assert len(tree.findall(".//ComputePathToPose")) == 2
    assert len(tree.findall(".//FollowPath")) == 1
    waits = tree.findall(".//Wait")
    assert len(waits) == 2
    assert all(wait.attrib["wait_duration"] == "2" for wait in waits)

    # This turn deliberately implements simulation first.  Real hardware must
    # remain on its reviewed configuration until the matching issue is done.
    assert real["global_costmap"]["global_costmap"]["ros__parameters"][
        "obstacle_layer"
    ]["scan"]["topic"] == "/scan"

    filter_parameters = load_yaml("configs/laser_filters_global_sim.yaml")
    median = filter_parameters["global_scan_filter"]["ros__parameters"][
        "filter1"
    ]["params"]["range_filter_chain"]["filter1"]["params"]
    assert median["number_of_observations"] == 5

    launch_source = (LAUNCH_ROOT / "carbot_navigation.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="static_map_scan_filter"' in launch_source
    assert '"static_margin_m": 0.12' in launch_source

    tree_source = (WORKSPACE / behavior_tree.split(
        "/isaac_3d_lidar_amr_ws/", 1
    )[1]).read_text(encoding="utf-8")
    assert '<Sequence name="NavigateWithFailureTriggeredReplanning">' in (
        tree_source
    )
    assert "<PipelineSequence" not in tree_source
    assert "<GoalUpdatedController>" not in tree_source
    assert 'name="PersistentObstacleReplan"' in tree_source
    assert "<IsPathValid" not in tree_source
    assert "PathExpiringTimer" not in tree_source


def test_common_runtime_config_points_to_canonical_geometry():
    common = load_yaml("configs/carbot/common.yaml")
    assert common["frames"]["robot_base"] == "base_footprint"
    assert common["topics"]["odom"] == "/odom"
    assert common["topics"]["pointcloud"] == "/livox/lidar"
    assert common["maps"]["nav2_yaml"].endswith("warehouse_v3.yaml")
    assert "geometry" not in common
