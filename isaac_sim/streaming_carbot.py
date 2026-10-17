"""Run Carbot control inside the existing Isaac Sim WebRTC Kit process."""

import asyncio
import math
import os
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
    from isaac_sim.evidence_models import (
        EvidenceActuatorModel,
        load_evidence_profile,
    )
    from isaac_sim.carbot_mid360 import (
        create_mid360_pipeline,
        install_profile,
        mid360_runtime_config,
    )
    from isaac_sim.carbot_d455_mount import set_d455_mount_angles
    from isaac_sim.overhead_clearance_obstacles import (
        create_overhead_clearance_obstacles,
    )
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Usd, UsdGeom

    parameters = yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))
    d455_config = parameters["sensors"]["d455_sim_mount"]
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
    response_mode = os.environ.get(
        "CARBOT_SIM_RESPONSE_MODE",
        parameters["simulation"]["response_mode_default"],
    )
    if response_mode not in parameters["simulation"]["response_modes"]:
        raise ValueError(f"unsupported CARBOT_SIM_RESPONSE_MODE={response_mode}")
    response_model = None
    if response_mode == "evidence_degraded":
        evidence = load_evidence_profile(
            WORKSPACE / parameters["simulation"]["evidence_profile"]
        )
        response_model = EvidenceActuatorModel(evidence["actuator"])

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

    overhead_obstacles = create_overhead_clearance_obstacles(
        stage, robot_prim_path
    )
    print(
        "Persistent overhead-clearance obstacles: "
        + ", ".join(
            f"{item.clearance_m:.2f} m at "
            f"map=({item.map_x_m:.2f}, {item.map_y_m:.2f})"
            for item in overhead_obstacles
        ),
        flush=True,
    )

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
        eye=[0.85, 1.65, 1.90],
        target=[0.0, 0.984415, 0.08],
        camera_prim_path="/OmniverseKit_Persp",
    )

    d455_yaw_path = (
        f"{robot_prim_path}/base_link/d455_mount/arm1_yaw"
    )
    d455_pitch_path = f"{d455_yaw_path}/arm2_pitch"
    d455_yaw_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(d455_yaw_path))
    d455_pitch_api = UsdGeom.XformCommonAPI(stage.GetPrimAtPath(d455_pitch_path))
    if not d455_yaw_api or not d455_pitch_api:
        raise RuntimeError("D455 two-axis mount is missing from Carbot USD")
    d455_command = {"sequence": 0, "angles": (0.0, 0.0)}

    def receive_d455_command(message):
        if len(message.data) != 2:
            node.get_logger().warning("D455 gimbal command needs [yaw, pitch]")
            return
        try:
            from isaac_sim.carbot_d455_mount import clamp_gimbal_angles
            d455_command["angles"] = clamp_gimbal_angles(
                d455_config, float(message.data[0]), float(message.data[1])
            )
            d455_command["sequence"] += 1
        except ValueError as error:
            node.get_logger().warning(str(error))

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
    node.create_subscription(
        Float64MultiArray,
        "/isaac_sim/d455_gimbal_cmd",
        receive_d455_command,
        10,
    )

    def receive_prim_query(message):
        """Print every prim whose world bounds lie near robot-frame points.

        Debug aid for LiDAR returns without a known source.  The message is
        [radius_m, bearing_deg, range_m, z_m, ...] relative to the robot's
        base footprint; bounds include render, proxy and guide geometry,
        which the RTX LiDAR can hit even without a collider.
        """
        values = list(message.data)
        if values == [-1.0]:
            # List every boundable prim under the robot with its local size;
            # robot transforms may live in Fabric, so world bounds can be
            # stale while local extents stay valid.
            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                 UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
            )
            root = stage.GetPrimAtPath(robot_prim_path)
            for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                if not prim.IsA(UsdGeom.Boundable):
                    continue
                box = cache.ComputeLocalBound(prim).ComputeAlignedRange()
                if box.IsEmpty():
                    continue
                size = box.GetSize()
                imageable = UsdGeom.Imageable(prim)
                print(f"[PRIM QUERY] robot part {prim.GetPath()} "
                      f"{prim.GetTypeName()} purpose="
                      f"{imageable.ComputePurpose()} size="
                      f"{tuple(round(v, 3) for v in size)}", flush=True)
            print("[PRIM QUERY] robot parts done", flush=True)
            return
        if len(values) < 4 or (len(values) - 1) % 3:
            node.get_logger().warning(
                "prim query needs [radius, bearing_deg, range_m, z_m, ...]")
            return
        radius = float(values[0])
        position, orientation = robot.get_world_pose()
        yaw = quaternion_yaw_angle(orientation)
        targets = []
        for index in range(1, len(values), 3):
            bearing = math.radians(values[index])
            distance = values[index + 1]
            targets.append((
                float(position[0]) + distance * math.cos(yaw + bearing),
                float(position[1]) + distance * math.sin(yaw + bearing),
                float(values[index + 2]),
            ))
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
             UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide],
        )
        print(f"[PRIM QUERY] robot world=({float(position[0]):.3f}, "
              f"{float(position[1]):.3f}) yaw={math.degrees(yaw):.1f} deg "
              f"targets={[tuple(round(v, 2) for v in t) for t in targets]}",
              flush=True)
        found = 0
        for prim in Usd.PrimRange(
                stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            # Boundable covers Gprims and PointInstancers.
            if not prim.IsA(UsdGeom.Boundable):
                continue
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if box.IsEmpty():
                continue
            low, high = box.GetMin(), box.GetMax()
            for target in targets:
                gap = math.sqrt(sum(
                    max(low[axis] - target[axis], 0.0,
                        target[axis] - high[axis]) ** 2
                    for axis in range(3)))
                if gap <= radius:
                    imageable = UsdGeom.Imageable(prim)
                    print(f"[PRIM QUERY] {prim.GetPath()} "
                          f"{prim.GetTypeName()} purpose="
                          f"{imageable.ComputePurpose()} visibility="
                          f"{imageable.ComputeVisibility()} min="
                          f"{tuple(round(v, 2) for v in low)} max="
                          f"{tuple(round(v, 2) for v in high)} "
                          f"target={tuple(round(v, 2) for v in target)}",
                          flush=True)
                    found += 1
                    break
        print(f"[PRIM QUERY] done: {found} prims", flush=True)

    node.create_subscription(
        Float64MultiArray,
        "/isaac_sim/debug_prim_query",
        receive_prim_query,
        10,
    )
    applied_obstacle_sequence = 0
    applied_d455_sequence = 0
    limiter = CarbotCommandLimiter(limits, response_model=response_model)
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
            if d455_command["sequence"] != applied_d455_sequence:
                set_d455_mount_angles(
                    d455_yaw_api, d455_pitch_api, d455_config,
                    *d455_command["angles"],
                )
                applied_d455_sequence = d455_command["sequence"]
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
