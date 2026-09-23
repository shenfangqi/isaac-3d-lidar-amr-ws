#!/usr/bin/env python3
"""Run the Carbot articulation with bounded ROS 2 differential control."""

import os
import sys
import time
from pathlib import Path


os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
bridge_library = "/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"
message_library = (
    "/workspace/ros-humble/isaac_3d_lidar_amr_ws/install/carbot_msgs/lib"
)
library_path = os.environ.get("LD_LIBRARY_PATH", "")
os.environ["LD_LIBRARY_PATH"] = ":".join(
    dict.fromkeys(
        value for value in (library_path, bridge_library, message_library)
        if value
    )
)

WORKSPACE = Path("/workspace/ros-humble/isaac_3d_lidar_amr_ws")
SCENE_PATH = WORKSPACE / "isaac_sim/usd/warehouse_3d_nav_origin_carbot.usd"
PARAMETER_PATH = (
    WORKSPACE / "src/carbot_description/config/carbot_parameters.yaml"
)
sys.path.insert(0, str(WORKSPACE))

simulation_app = None
if __name__ == "__main__":
    from isaacsim import SimulationApp  # noqa: E402

    simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import yaml  # noqa: E402
from isaacsim.core.utils.extensions import (  # noqa: E402
    enable_extension,
    get_extension_path_from_name,
)


if simulation_app is not None:
    enable_extension("isaacsim.ros2.bridge")
    enable_extension("isaacsim.sensors.rtx")
    for _ in range(100):
        simulation_app.update()

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as TimeMessage  # noqa: E402
from geometry_msgs.msg import TransformStamped, Twist  # noqa: E402
from isaac_sim.carbot_control import (  # noqa: E402
    CarbotCommandLimiter,
    ControlLimits,
)
from isaac_sim.evidence_models import (  # noqa: E402
    EvidenceActuatorModel,
    load_evidence_profile,
)
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


def update_app(count):
    for _ in range(count):
        simulation_app.update()


def quaternion_conjugate(quaternion):
    w, x, y, z = quaternion
    return np.array([w, -x, -y, -z], dtype=float)


def quaternion_multiply(first, second):
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def rotate_vector(vector, quaternion):
    pure_vector = np.array([0.0, *vector], dtype=float)
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, pure_vector),
        quaternion_conjugate(quaternion),
    )
    return rotated[1:]


def yaw_quaternion(quaternion):
    yaw = quaternion_yaw_angle(quaternion)
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def quaternion_yaw_angle(quaternion):
    w, x, y, z = quaternion
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def time_message(seconds):
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    return TimeMessage(sec=whole_seconds, nanosec=nanoseconds)


class CarbotRosNode(Node):
    def __init__(self, limits, odom_topic="/odom", publish_odom_tf=True):
        super().__init__("carbot_isaac_sim")
        self.requested_linear_mps = 0.0
        self.requested_angular_rad_s = 0.0
        self.last_command_monotonic = float("-inf")
        self.watchdog_was_active = True
        self.limits = limits
        self.publish_odom_tf = publish_odom_tf

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Twist, "/cmd_vel", self.command_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.joint_publisher = self.create_publisher(
            JointState, "/joint_states", sensor_qos
        )
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", tf_qos)
        self.clock_publisher = self.create_publisher(
            Clock, "/clock", clock_qos
        )

    def command_callback(self, message):
        self.requested_linear_mps = float(message.linear.x)
        self.requested_angular_rad_s = float(message.angular.z)
        self.last_command_monotonic = time.monotonic()

    def command_age(self):
        return time.monotonic() - self.last_command_monotonic

    def report_watchdog(self, active):
        if active != self.watchdog_was_active:
            state = (
                "active; ramping command to zero"
                if active
                else "receiving commands"
            )
            self.get_logger().info(f"cmd_vel watchdog: {state}")
            self.watchdog_was_active = active


def publish_state(
    node, robot, simulation_time, initial_position, initial_orientation
):
    stamp = time_message(simulation_time)
    position, orientation = robot.get_world_pose()
    inverse_initial = quaternion_conjugate(initial_orientation)
    relative_position = rotate_vector(
        position - initial_position, inverse_initial
    )
    relative_orientation = quaternion_multiply(inverse_initial, orientation)
    planar_orientation = yaw_quaternion(relative_orientation)

    world_linear_velocity = robot.get_linear_velocity()
    world_angular_velocity = robot.get_angular_velocity()
    body_linear_velocity = rotate_vector(
        world_linear_velocity, quaternion_conjugate(orientation)
    )
    body_angular_velocity = rotate_vector(
        world_angular_velocity, quaternion_conjugate(orientation)
    )

    odometry = Odometry()
    odometry.header.stamp = stamp
    odometry.header.frame_id = "odom"
    odometry.child_frame_id = "base_footprint"
    odometry.pose.pose.position.x = float(relative_position[0])
    odometry.pose.pose.position.y = float(relative_position[1])
    odometry.pose.pose.orientation.w = float(planar_orientation[0])
    odometry.pose.pose.orientation.x = float(planar_orientation[1])
    odometry.pose.pose.orientation.y = float(planar_orientation[2])
    odometry.pose.pose.orientation.z = float(planar_orientation[3])
    odometry.twist.twist.linear.x = float(body_linear_velocity[0])
    odometry.twist.twist.linear.y = float(body_linear_velocity[1])
    odometry.twist.twist.angular.z = float(body_angular_velocity[2])
    node.odom_publisher.publish(odometry)

    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x = float(relative_position[0])
    transform.transform.translation.y = float(relative_position[1])
    transform.transform.rotation.w = float(planar_orientation[0])
    transform.transform.rotation.x = float(planar_orientation[1])
    transform.transform.rotation.y = float(planar_orientation[2])
    transform.transform.rotation.z = float(planar_orientation[3])
    if node.publish_odom_tf:
        node.tf_publisher.publish(TFMessage(transforms=[transform]))

    joint_state = JointState()
    joint_state.header.stamp = stamp
    joint_state.header.frame_id = "base_link"
    joint_state.name = list(robot.dof_names)
    joint_state.position = [
        float(value) for value in robot.get_joint_positions()
    ]
    joint_state.velocity = [
        float(value) for value in robot.get_joint_velocities()
    ]
    node.joint_publisher.publish(joint_state)
    node.clock_publisher.publish(Clock(clock=stamp))


def main():
    parameters = yaml.safe_load(PARAMETER_PATH.read_text(encoding="utf-8"))
    from isaac_sim.carbot_mid360 import (
        create_mid360_pipeline,
        install_profile,
        mid360_runtime_config,
    )
    from isaac_sim.overhead_clearance_obstacles import (
        create_overhead_clearance_obstacles,
    )

    profile_path = (
        WORKSPACE / "isaac_sim/lidar_configs/Livox_Mid360_Approx.json"
    )
    installed_profile = install_profile(
        profile_path, get_extension_path_from_name
    )
    print(f"Installed Mid-360 proxy profile: {installed_profile}", flush=True)
    limits = ControlLimits.from_parameters(parameters)
    control_period_s = parameters["control"]["differential_period_s"]
    robot_prim_path = parameters["simulation"]["articulation_root_prim"]
    wheel_joint_sign = parameters["simulation"]["wheel_joint_coordinate_sign"]
    response_mode = os.environ.get(
        "CARBOT_SIM_RESPONSE_MODE",
        parameters["simulation"]["response_mode_default"],
    )
    if response_mode not in parameters["simulation"]["response_modes"]:
        raise ValueError(f"unsupported CARBOT_SIM_RESPONSE_MODE={response_mode}")
    response_model = None
    evidence = None
    if response_mode == "evidence_degraded":
        evidence_path = WORKSPACE / parameters["simulation"][
            "evidence_profile"
        ]
        evidence = load_evidence_profile(evidence_path)
        response_model = EvidenceActuatorModel(evidence["actuator"])
    estimator_mode = os.environ.get("CARBOT_SIM_ESTIMATOR_MODE", "direct")
    if estimator_mode not in ("direct", "evidence_ekf"):
        raise ValueError(
            f"unsupported CARBOT_SIM_ESTIMATOR_MODE={estimator_mode}"
        )
    if estimator_mode == "evidence_ekf" and evidence is None:
        raise ValueError("evidence_ekf requires evidence_degraded response mode")
    raw_pointcloud_topic = (
        "/livox/lidar_ground_truth"
        if estimator_mode == "evidence_ekf"
        else None
    )

    print(f"Opening Carbot warehouse: {SCENE_PATH}", flush=True)
    omni.usd.get_context().open_stage(str(SCENE_PATH))
    update_app(300)
    stage = omni.usd.get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath(robot_prim_path).IsValid():
        raise RuntimeError(
            f"Carbot warehouse failed to load {robot_prim_path}"
        )

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

    mid360_handles = create_mid360_pipeline(
        stage, parameters, raw_pointcloud_topic
    )
    mid360_config = mid360_runtime_config(parameters, raw_pointcloud_topic)
    print(
        f"Carbot Mid-360 ready: {mid360_handles[0].GetPath()}; "
        f"{mid360_config['pointcloud_topic']} "
        f"[frame_id={mid360_config['frame_id']}]; "
        f"rate={mid360_config['pointcloud_rate_hz']} Hz; "
        "RTX origin uses the verified Livox O offset",
        flush=True,
    )

    world = World(
        physics_dt=control_period_s,
        rendering_dt=control_period_s,
        stage_units_in_meters=1.0,
    )
    robot = world.scene.add(
        SingleArticulation(prim_path=robot_prim_path, name="carbot")
    )
    world.reset()
    dof_names = list(robot.dof_names)
    if len(dof_names) != 12 or not all(
        name.endswith("_wheel_joint") for name in dof_names
    ):
        raise RuntimeError(f"Unexpected Carbot DOFs: {dof_names}")
    zero_joint_velocities = np.zeros(len(dof_names), dtype=float)
    for _ in range(50):
        robot.set_joint_velocities(zero_joint_velocities)
        world.step(render=True)
    initial_position, initial_orientation = robot.get_world_pose()
    planar_position = initial_position.copy()
    planar_yaw = quaternion_yaw_angle(initial_orientation)
    initial_orientation = quaternion_from_yaw(planar_yaw)
    robot.set_world_pose(initial_position, initial_orientation)

    rclpy.init(args=None)
    node = CarbotRosNode(
        limits,
        odom_topic=(
            evidence["state_estimation"]["ground_truth_topic"]
            if estimator_mode == "evidence_ekf"
            else "/odom"
        ),
        publish_odom_tf=estimator_mode == "direct",
    )
    evidence_observations = None
    if estimator_mode == "evidence_ekf":
        from isaac_sim.evidence_observations import (
            EvidenceObservationPublisher,
        )

        evidence_observations = EvidenceObservationPublisher(
            node, evidence, time_message
        )
    limiter = CarbotCommandLimiter(limits, response_model=response_model)
    last_publish_time = float("-inf")
    print(
        f"Carbot control ready [{response_mode}, {estimator_mode}]: "
        "/cmd_vel -> 12 wheel joints; publishing state and sensors",
        flush=True,
    )

    try:
        while simulation_app.is_running():
            loop_started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            command = limiter.update(
                node.requested_linear_mps,
                node.requested_angular_rad_s,
                command_age_s=node.command_age(),
                dt_s=control_period_s,
            )
            node.report_watchdog(command.watchdog_active)
            joint_velocities = np.zeros(len(dof_names), dtype=float)
            for index, name in enumerate(dof_names):
                logical_wheel_velocity = (
                    command.left_wheel_rad_s
                    if name.startswith("left_")
                    else command.right_wheel_rad_s
                )
                # The imported joint axis is -Y, so raw joint coordinates have
                # the opposite sign from forward-positive encoder semantics.
                joint_velocities[index] = (
                    wheel_joint_sign * logical_wheel_velocity
                )
            robot.set_joint_velocities(joint_velocities)
            world.step(render=True)

            midpoint_yaw = (
                planar_yaw
                + command.applied_angular_rad_s * control_period_s / 2.0
            )
            planar_position[0] += (
                command.applied_linear_mps
                * np.cos(midpoint_yaw)
                * control_period_s
            )
            planar_position[1] += (
                command.applied_linear_mps
                * np.sin(midpoint_yaw)
                * control_period_s
            )
            planar_yaw += command.applied_angular_rad_s * control_period_s
            physics_position, _ = robot.get_world_pose()
            planar_position[2] = physics_position[2]
            planar_orientation = quaternion_from_yaw(planar_yaw)
            robot.set_world_pose(planar_position, planar_orientation)
            forward_velocity = rotate_vector(
                [command.applied_linear_mps, 0.0, 0.0], planar_orientation
            )
            robot.set_linear_velocity(forward_velocity)
            robot.set_angular_velocity(
                np.array([0.0, 0.0, command.applied_angular_rad_s])
            )
            robot.set_joint_velocities(joint_velocities)

            if (
                world.current_time - last_publish_time
                >= control_period_s - 1e-9
            ):
                publish_state(
                    node,
                    robot,
                    world.current_time,
                    initial_position,
                    initial_orientation,
                )
                if evidence_observations is not None:
                    evidence_observations.update(
                        world.current_time, command, control_period_s
                    )
                last_publish_time = world.current_time
            remaining_wall_time = control_period_s - (
                time.monotonic() - loop_started
            )
            if remaining_wall_time > 0.0:
                time.sleep(remaining_wall_time)
    finally:
        robot.set_joint_velocities(zero_joint_velocities)
        robot.set_linear_velocity(np.zeros(3, dtype=float))
        robot.set_angular_velocity(np.zeros(3, dtype=float))
        world.step(render=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
