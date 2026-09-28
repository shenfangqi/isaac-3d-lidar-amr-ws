from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET

import yaml


DESCRIPTION_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = DESCRIPTION_ROOT.parents[1]
CONFIG_ROOT = WORKSPACE / "configs"
LAUNCH_ROOT = WORKSPACE / "launch"


def load_overhead_obstacle_module():
    path = WORKSPACE / "isaac_sim/overhead_clearance_obstacles.py"
    spec = importlib.util.spec_from_file_location(
        "overhead_clearance_obstacles", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    assert real["odom_source"] == "fast_lio_base_adapter"
    assert real["allow_chassis_odom_relay"] is False
    assert real["allow_ground_truth_map_to_odom"] is False
    assert real["localization_default"] == "manual_map_localizer"
    assert real["initial_pose_default"] == "rviz_manual"
    assert real["map_to_odom_owner"] == "manual_map_localizer"
    assert "/livox/lidar" in real["required_external_interfaces"]
    assert "/mid360/imu/data_raw" in real["required_external_interfaces"]


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
            if runtime == "sim":
                assert node["inflation_layer"]["inflation_radius"] == 0.45

    real = nav_parameters("real")
    assert real["local_costmap"]["local_costmap"]["ros__parameters"][
        "footprint_padding"
    ] == 0.01
    assert real["global_costmap"]["global_costmap"]["ros__parameters"][
        "footprint_padding"
    ] == 0.03
    assert real["local_costmap"]["local_costmap"]["ros__parameters"][
        "inflation_layer"
    ]["inflation_radius"] == 0.25
    assert real["global_costmap"]["global_costmap"]["ros__parameters"][
        "inflation_layer"
    ]["inflation_radius"] == 0.30


def test_nav2_time_and_velocity_limits_are_runtime_specific():
    expected = {
        "sim": (True, 0.30, 0.35),
        "real": (False, 0.10, 0.50),
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
        if runtime == "real":
            goal_checker = parameters["controller_server"]["ros__parameters"][
                "general_goal_checker"
            ]
            assert goal_checker["yaw_goal_tolerance"] == 0.03
            behavior = parameters["behavior_server"]["ros__parameters"]
            assert behavior["min_rotational_vel"] == 0.40
            assert behavior["max_rotational_vel"] == 0.50


def test_sim_real_parity_profile_matches_physical_navigation_policy():
    simulation = nav_parameters("sim")
    parity = load_yaml("configs/nav2_params_sim_real_parity.yaml")
    real = nav_parameters("real")

    # The established fast simulation profile stays the default and retains
    # its independent warehouse regression envelope.
    sim_profile = load_yaml("configs/carbot/sim.yaml")
    assert sim_profile["nav2_params_file"].endswith(
        "configs/nav2_params_sim.yaml"
    )
    assert simulation["controller_server"]["ros__parameters"][
        "FollowPath"
    ]["desired_linear_vel"] == 0.30

    for node_name in (
        "controller_server",
        "planner_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
        "velocity_smoother",
        "lifecycle_manager_navigation",
    ):
        assert parity[node_name]["ros__parameters"]["use_sim_time"] is True

    parity_controller = parity["controller_server"]["ros__parameters"]
    real_controller = real["controller_server"]["ros__parameters"]
    assert parity_controller["controller_frequency"] == (
        real_controller["controller_frequency"]
    )
    for key in (
        "desired_linear_vel",
        "lookahead_dist",
        "min_lookahead_dist",
        "max_lookahead_dist",
        "max_allowed_time_to_collision_up_to_carrot",
        "rotate_to_heading_min_angle",
        "rotate_to_heading_angular_vel",
        "max_angular_accel",
    ):
        assert parity_controller["FollowPath"][key] == (
            real_controller["FollowPath"][key]
        )

    for costmap in ("local_costmap", "global_costmap"):
        parity_costmap = parity[costmap][costmap]["ros__parameters"]
        real_costmap = real[costmap][costmap]["ros__parameters"]
        assert parity_costmap["use_sim_time"] is True
        assert parity_costmap["global_frame"] == real_costmap["global_frame"]
        assert parity_costmap["footprint"] == real_costmap["footprint"]
        assert parity_costmap["footprint_padding"] == (
            real_costmap["footprint_padding"]
        )
        assert parity_costmap["inflation_layer"]["inflation_radius"] == (
            real_costmap["inflation_layer"]["inflation_radius"]
        )
    assert parity["global_costmap"]["global_costmap"]["ros__parameters"][
        "obstacle_layer"
    ]["enabled"] is False

    assert parity["planner_server"]["ros__parameters"]["GridBased"] == (
        real["planner_server"]["ros__parameters"]["GridBased"]
    )
    assert parity["bt_navigator"]["ros__parameters"][
        "default_nav_to_pose_bt_xml"
    ] == real["bt_navigator"]["ros__parameters"][
        "default_nav_to_pose_bt_xml"
    ]
    for key in ("max_velocity", "min_velocity", "max_accel", "max_decel"):
        assert parity["velocity_smoother"]["ros__parameters"][key] == (
            real["velocity_smoother"]["ros__parameters"][key]
        )

    parity_source = (
        WORKSPACE / "configs/nav2_params_sim_real_parity.yaml"
    ).read_text(encoding="utf-8")
    assert "fast_lio" not in parity_source.lower()
    assert "manual_map_localizer" not in parity_source


def test_sim_launch_can_select_real_navigation_parity_profile():
    launch_source = (LAUNCH_ROOT / "carbot_sim.launch.py").read_text(
        encoding="utf-8"
    )
    shared_source = (LAUNCH_ROOT / "carbot_navigation.py").read_text(
        encoding="utf-8"
    )
    assert 'LaunchConfiguration("nav2_params_file")' in launch_source
    assert '"nav2_params_file",' in launch_source
    assert "nav2_params_sim_real_parity.yaml" in launch_source
    assert '"params_file": nav2_params_file or profile[' in shared_source


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
    assert "carbot_navigation_real.launch.py" in real
    assert "cmd_vel_output" in real
    assert "amcl_initial_pose_mode" not in real
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

    tree_path = WORKSPACE / "configs/behavior_trees" / Path(behavior_tree).name
    tree = ET.parse(tree_path).getroot()
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
    canonical = load_yaml(
        "src/carbot_description/config/carbot_parameters.yaml"
    )["geometry"]
    assert common["frames"]["robot_base"] == "base_footprint"
    assert common["topics"]["odom"] == "/odom"
    assert common["topics"]["pointcloud"] == "/livox/lidar"
    assert common["maps"]["nav2_yaml"].endswith("warehouse_v3.yaml")
    assert "geometry" not in common
    clearance = common["scan_projection"]["max_height_m"]
    assert clearance == canonical["minimum_overhead_clearance_m"] == 0.35
    assert clearance - canonical["overall_size_m"][2] >= 0.10


def test_all_scan_and_esdf_paths_share_overhead_clearance():
    clearance = 0.35
    launch_paths = (
        "src/isaac_3d_lidar_bringup/launch/"
        "carbot_navigation_real.launch.py",
        "src/isaac_3d_lidar_bringup/launch/"
        "mid360_nvblox_real.launch.py",
        "src/isaac_3d_lidar_exploration/launch/explore_nvblox.launch.py",
    )
    for path in launch_paths:
        source = (WORKSPACE / path).read_text(encoding="utf-8")
        assert f"'max_height': {clearance}" in source

    nvblox_paths = {
        "src/isaac_3d_lidar_bringup/config/nvblox/"
        "mid360_nvblox_real.yaml": 0.185209546,
        "src/isaac_3d_lidar_bringup/config/nvblox/"
        "mid360_nvblox_sim.yaml": clearance,
    }
    for path, expected_max_height in nvblox_paths.items():
        parameters = load_yaml(path)["/**"]["ros__parameters"]
        assert parameters["static_mapper"][
            "esdf_slice_max_height"
        ] == expected_max_height
        if "dynamic_mapper" in parameters:
            assert parameters["dynamic_mapper"][
                "esdf_slice_max_height"
            ] == clearance


def test_persistent_overhead_obstacles_bracket_safe_clearance():
    module = load_overhead_obstacle_module()
    common = load_yaml("configs/carbot/common.yaml")
    geometry = load_yaml(
        "src/carbot_description/config/carbot_parameters.yaml"
    )["geometry"]
    clearance = common["scan_projection"]["max_height_m"]
    obstacles = module.OVERHEAD_OBSTACLES
    configured = common["simulation_overhead_obstacles"]

    assert len(obstacles) == 2
    assert len(configured) == len(obstacles)
    assert {item.clearance_m for item in obstacles} == {0.28, 0.40}
    assert [item.name for item in obstacles] == [
        item["name"] for item in configured
    ]
    assert any(item.clearance_m < clearance for item in obstacles)
    assert any(item.clearance_m > clearance for item in obstacles)
    assert all(
        item.clearance_m > geometry["overall_size_m"][2]
        for item in obstacles
    )
    assert len({item.prim_path for item in obstacles}) == len(obstacles)
    assert all(item.size_y_m >= 1.0 for item in obstacles)

    for path in (
        WORKSPACE / "isaac_sim/auto_play_carbot.py",
        WORKSPACE / "isaac_sim/streaming_carbot.py",
    ):
        source = path.read_text(encoding="utf-8")
        assert "create_overhead_clearance_obstacles" in source


def test_rviz_displays_simulation_overhead_obstacle_markers():
    common = load_yaml("configs/carbot/common.yaml")
    topic = common["topics"]["overhead_clearance_markers"]
    launch_source = (LAUNCH_ROOT / "carbot_navigation.py").read_text(
        encoding="utf-8"
    )
    setup_source = (
        WORKSPACE / "src/isaac_3d_lidar_bringup/setup.py"
    ).read_text(encoding="utf-8")
    marker_source = (
        WORKSPACE
        / "src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/"
        "overhead_clearance_marker_publisher.py"
    ).read_text(encoding="utf-8")
    rviz_source = (
        WORKSPACE / "configs/rviz/carbot_navigation.rviz"
    ).read_text(encoding="utf-8")

    assert topic == "/overhead_clearance_markers"
    assert 'executable="overhead_clearance_marker_publisher"' in (
        launch_source
    )
    assert "overhead_clearance_marker_publisher:main" in setup_source
    assert "Marker.CUBE" in marker_source
    assert "Marker.TEXT_VIEW_FACING" in marker_source
    assert "rviz_default_plugins/MarkerArray" in rviz_source
    assert f"Value: {topic}" in rviz_source
