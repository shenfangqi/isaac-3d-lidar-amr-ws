"""Run Carbot control inside the existing Isaac Sim WebRTC Kit process."""

import asyncio
import math
import sys
from pathlib import Path

import omni.kit.app
import omni.usd
import yaml
from isaacsim.core.utils.extensions import (
    enable_extension,
    get_extension_path_from_name,
)


WORKSPACE = Path("/workspace/ros-humble/isaac_3d_lidar_amr_ws")
SCENE_PATH = WORKSPACE / "isaac_sim/usd/warehouse_3d_nav_origin_carbot.usd"
PARAMETER_PATH = (
    WORKSPACE / "src/carbot_description/config/carbot_parameters.yaml"
)
PROFILE_PATH = WORKSPACE / "isaac_sim/lidar_configs/Livox_Mid360_Approx.json"
LEGACY_LIDAR_GRAPH_PATH = "/World/ROS2_LidarRTX"
LEGACY_ROBOT_ROOT_PATH = "/World/Robot"
DYNAMIC_OBSTACLE_PATH = "/World/CarbotDynamicValidationObstacle"
DYNAMIC_OBSTACLE_COUNT = 4
sys.path.insert(0, str(WORKSPACE))


def is_legacy_carter_path(path):
    lowered = str(path).lower()
    return (
        "carter" in lowered
        or "nova_" in lowered
        or str(path) in (LEGACY_LIDAR_GRAPH_PATH, LEGACY_ROBOT_ROOT_PATH)
    )


def deactivate_legacy_carter_roots(stage):
    paths = [prim.GetPath() for prim in stage.TraverseAll()]
    root_paths = [
        path
        for path in paths
        if is_legacy_carter_path(path)
        and not is_legacy_carter_path(path.GetParentPath())
    ]
    for path in root_paths:
        stage.GetPrimAtPath(path).SetActive(False)
    return [str(path) for path in root_paths]


async def update_app(app, count):
    for _ in range(count):
        await app.next_update_async()


def create_dynamic_validation_obstacle(stage):
    """Create hidden, static cubes controlled by a ROS validation topic."""
    from pxr import Gf, UsdGeom, UsdPhysics

    transforms = []
    for index in range(DYNAMIC_OBSTACLE_COUNT):
        path = f"{DYNAMIC_OBSTACLE_PATH}_{index}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.15, 0.05)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        transform = UsdGeom.XformCommonAPI(cube.GetPrim())
        transform.SetTranslate(Gf.Vec3d(0.0, 0.0, -100.0))
        transform.SetScale(Gf.Vec3f(0.01, 0.01, 0.01))
        transforms.append(transform)
    return transforms


def apply_dynamic_obstacle_command(transforms, command):
    """Apply one or more 7-value cube commands; hide unused cubes."""
    from pxr import Gf

    if len(command) == 6:
        command = [*command, 0.0]
    if not command or len(command) % 7 != 0:
        raise ValueError(
            "dynamic obstacle command needs groups of 7 values: "
            "x y z size_x size_y size_z yaw"
        )
    groups = [
        command[index:index + 7] for index in range(0, len(command), 7)
    ]
    if len(groups) > len(transforms):
        raise ValueError(
            f"at most {len(transforms)} obstacle cubes are supported"
        )
    enabled_count = 0
    for index, transform in enumerate(transforms):
        values = groups[index] if index < len(groups) else None
        if values is None or min(map(float, values[3:6])) <= 0.0:
            transform.SetTranslate(Gf.Vec3d(0.0, 0.0, -100.0))
            transform.SetScale(Gf.Vec3f(0.01, 0.01, 0.01))
            transform.SetRotate(Gf.Vec3f(0.0, 0.0, 0.0))
            continue
        x, y, z, size_x, size_y, size_z, yaw = map(float, values)
        transform.SetTranslate(Gf.Vec3d(x, y, z))
        transform.SetScale(Gf.Vec3f(size_x, size_y, size_z))
        transform.SetRotate(Gf.Vec3f(0.0, 0.0, math.degrees(yaw)))
        enabled_count += 1
    return enabled_count


async def load_and_control():
    app = omni.kit.app.get_app()
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    await update_app(app, 100)

    import numpy as np
    import rclpy
    from std_msgs.msg import Float64MultiArray
    from isaac_sim.auto_play_carbot import (
        CarbotCommandLimiter,
        CarbotRosNode,
        ControlLimits,
        publish_state,
        quaternion_from_yaw,
        quaternion_yaw_angle,
        rotate_vector,
    )
    from isaac_sim.carbot_mid360 import (
        create_mid360_pipeline,
        install_profile,
        mid360_runtime_config,
    )
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.viewports import set_camera_view

    parameters = yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))
    installed_profile = install_profile(
        PROFILE_PATH, get_extension_path_from_name
    )
    print(f"Installed Mid-360 proxy profile: {installed_profile}", flush=True)
    limits = ControlLimits.from_parameters(parameters)
    control_period_s = parameters["control"]["differential_period_s"]
    robot_prim_path = parameters["simulation"]["articulation_root_prim"]
    wheel_joint_sign = parameters["simulation"][
        "wheel_joint_coordinate_sign"
    ]

    print(f"Opening streaming Carbot warehouse: {SCENE_PATH}", flush=True)
    await omni.usd.get_context().open_stage_async(str(SCENE_PATH))
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        disabled_legacy_roots = deactivate_legacy_carter_roots(stage)
        print(
            f"Disabled legacy Carter prims: {disabled_legacy_roots}",
            flush=True,
        )
    await update_app(app, 300)
    if stage is None or not stage.GetPrimAtPath(robot_prim_path).IsValid():
        raise RuntimeError(f"Carbot prim is missing: {robot_prim_path}")

    mid360_handles = create_mid360_pipeline(stage, parameters)
    mid360_config = mid360_runtime_config(parameters)
    print(
        f"Carbot Mid-360 ready: {mid360_handles[0].GetPath()}; "
        f"{mid360_config['pointcloud_topic']} "
        f"[frame_id={mid360_config['frame_id']}]; "
        f"rate={mid360_config['pointcloud_rate_hz']} Hz; "
        "RTX origin uses the verified Livox O offset",
        flush=True,
    )
    # Author the test-only cubes before World/RTX initialization. Keeping the
    # prims resident and moving them from a hidden pose is more reliable than
    # adding new geometry to the RTX scene while the timeline is running.
    obstacle_transform = create_dynamic_validation_obstacle(stage)

    world = World(
        physics_dt=control_period_s,
        rendering_dt=control_period_s,
        stage_units_in_meters=1.0,
    )
    await world.initialize_simulation_context_async()
    robot = world.scene.add(
        SingleArticulation(prim_path=robot_prim_path, name="carbot")
    )
    await app.next_update_async()
    await world.reset_async()
    zero_joint_velocities = np.zeros(12, dtype=float)
    for _ in range(50):
        robot.set_joint_velocities(zero_joint_velocities)
        await app.next_update_async()

    dof_names = list(robot.dof_names)
    if len(dof_names) != 12 or not all(
        name.endswith("_wheel_joint") for name in dof_names
    ):
        raise RuntimeError(f"Unexpected Carbot DOFs: {dof_names}")

    initial_position, initial_orientation = robot.get_world_pose()
    planar_position = initial_position.copy()
    planar_yaw = quaternion_yaw_angle(initial_orientation)
    initial_orientation = quaternion_from_yaw(planar_yaw)
    robot.set_world_pose(initial_position, initial_orientation)
    set_camera_view(
        eye=[1.7, 2.6, 1.25],
        target=[0.0, 0.984415, 0.08],
        camera_prim_path="/OmniverseKit_Persp",
    )

    if not rclpy.ok():
        rclpy.init(args=None)
    node = CarbotRosNode(limits)
    obstacle_command = {"sequence": 0, "values": []}

    def receive_obstacle_command(message):
        obstacle_command["values"] = list(message.data)
        obstacle_command["sequence"] += 1

    node.create_subscription(
        Float64MultiArray,
        "/isaac_sim/dynamic_obstacle",
        receive_obstacle_command,
        10,
    )
    applied_obstacle_sequence = 0
    limiter = CarbotCommandLimiter(limits)
    last_publish_time = float("-inf")
    previous_simulation_time = world.current_time
    print(
        "Streaming Carbot ready: connect WebRTC client to 127.0.0.1; "
        "/cmd_vel active; publishing /odom /tf /joint_states /clock "
        "/livox/lidar",
        flush=True,
    )

    try:
        while app.is_running():
            await app.next_update_async()
            simulation_time = world.current_time
            dt_s = simulation_time - previous_simulation_time
            if dt_s <= 0.0:
                continue
            previous_simulation_time = simulation_time
            rclpy.spin_once(node, timeout_sec=0.0)
            if obstacle_command["sequence"] != applied_obstacle_sequence:
                stage_obstacle_command = list(obstacle_command["values"])
                for index in range(0, len(stage_obstacle_command), 7):
                    if len(stage_obstacle_command) - index < 6:
                        break
                    # Commands use the map/odom frame whose origin is the
                    # robot's initial pose; USD uses absolute world space.
                    stage_obstacle_command[index] += float(initial_position[0])
                    stage_obstacle_command[index + 1] += float(
                        initial_position[1]
                    )
                enabled_count = apply_dynamic_obstacle_command(
                    obstacle_transform, stage_obstacle_command
                )
                applied_obstacle_sequence = obstacle_command["sequence"]
                print(
                    "Dynamic validation obstacle "
                    f"{'enabled' if enabled_count else 'hidden'} "
                    f"({enabled_count} cubes): "
                    f"{obstacle_command['values']}",
                    flush=True,
                )
            command = limiter.update(
                node.requested_linear_mps,
                node.requested_angular_rad_s,
                command_age_s=node.command_age(),
                dt_s=dt_s,
            )
            node.report_watchdog(command.watchdog_active)

            joint_velocities = np.zeros(len(dof_names), dtype=float)
            for index, name in enumerate(dof_names):
                logical_velocity = (
                    command.left_wheel_rad_s
                    if name.startswith("left_")
                    else command.right_wheel_rad_s
                )
                joint_velocities[index] = wheel_joint_sign * logical_velocity

            midpoint_yaw = (
                planar_yaw + command.applied_angular_rad_s * dt_s / 2.0
            )
            planar_position[0] += (
                command.applied_linear_mps * np.cos(midpoint_yaw) * dt_s
            )
            planar_position[1] += (
                command.applied_linear_mps * np.sin(midpoint_yaw) * dt_s
            )
            planar_yaw += command.applied_angular_rad_s * dt_s
            physics_position, _ = robot.get_world_pose()
            planar_position[2] = physics_position[2]
            planar_orientation = quaternion_from_yaw(planar_yaw)
            robot.set_world_pose(planar_position, planar_orientation)
            robot.set_linear_velocity(
                rotate_vector(
                    [command.applied_linear_mps, 0.0, 0.0],
                    planar_orientation,
                )
            )
            robot.set_angular_velocity(
                np.array([0.0, 0.0, command.applied_angular_rad_s])
            )
            robot.set_joint_velocities(joint_velocities)

            if simulation_time - last_publish_time >= control_period_s - 1e-9:
                publish_state(
                    node,
                    robot,
                    simulation_time,
                    initial_position,
                    initial_orientation,
                )
                last_publish_time = simulation_time
    finally:
        robot.set_joint_velocities(zero_joint_velocities)
        robot.set_linear_velocity(np.zeros(3, dtype=float))
        robot.set_angular_velocity(np.zeros(3, dtype=float))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def report_task_result(task):
    try:
        task.result()
    except asyncio.CancelledError:
        pass
    except BaseException as error:
        print(f"Streaming Carbot failed: {error!r}", flush=True)
        raise


task = asyncio.ensure_future(load_and_control())
task.add_done_callback(report_task_result)
