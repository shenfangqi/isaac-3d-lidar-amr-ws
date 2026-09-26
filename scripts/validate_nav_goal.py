#!/usr/bin/env python3
"""Pre-plan and execute one measurable Carbot Nav2 regression goal."""

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", choices=("straight", "turning"), required=True)
parser.add_argument("--distance", type=float, default=0.70)
parser.add_argument("--startup-delay", type=float, default=45.0)
parser.add_argument("--timeout", type=float, default=120.0)
args, ros_args = parser.parse_known_args()


MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class NavRegression(Node):
    def __init__(self):
        super().__init__("carbot_nav_regression")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.map_message = None
        self.global_costmap = None
        self.latest_odom = None
        self.trajectory = []
        self.record_trajectory = False
        self.feedback_recoveries = 0
        self.minimum_distance_remaining = float("inf")
        self.create_subscription(OccupancyGrid, "/map", self._map, MAP_QOS)
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._costmap,
            MAP_QOS,
        )
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.compute_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.navigate_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )

    def _map(self, message):
        self.map_message = message

    def _costmap(self, message):
        self.global_costmap = message

    def _odom(self, message):
        self.latest_odom = message
        if self.record_trajectory:
            self.trajectory.append(
                (message.pose.pose.position.x, message.pose.pose.position.y)
            )

    def feedback(self, message):
        feedback = message.feedback
        self.feedback_recoveries = max(
            self.feedback_recoveries, int(feedback.number_of_recoveries)
        )
        self.minimum_distance_remaining = min(
            self.minimum_distance_remaining, float(feedback.distance_remaining)
        )

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False


def grid_value(message, x, y):
    info = message.info
    column = math.floor((x - info.origin.position.x) / info.resolution)
    row = math.floor((y - info.origin.position.y) / info.resolution)
    if not (0 <= column < info.width and 0 <= row < info.height):
        raise RuntimeError(f"Goal ({x:.3f}, {y:.3f}) is outside the grid")
    return int(message.data[row * info.width + column])


def path_length(path):
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def trajectory_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def main():
    if not 0.4 <= args.distance <= 1.0:
        raise ValueError("--distance must be between 0.4 and 1.0 m")
    rclpy.init(args=ros_args)
    node = NavRegression()
    try:
        ready = node.wait_until(
            lambda: (
                node.map_message is not None
                and node.global_costmap is not None
                and node.latest_odom is not None
                and node.tf_buffer.can_transform("map", "base_footprint", Time())
            ),
            15.0,
        )
        if not ready:
            raise RuntimeError("Timed out waiting for map, costmap, odom, or TF")

        map_data = node.map_message.data
        counts = {
            "unknown": sum(value == -1 for value in map_data),
            "free": sum(value == 0 for value in map_data),
            "occupied": sum(value == 100 for value in map_data),
        }
        expected_counts = {"unknown": 117570, "free": 52618, "occupied": 6620}
        info = node.map_message.info
        print(
            f"[NAV][MAP] {info.width}x{info.height}, resolution={info.resolution:.3f} m, "
            f"origin=({info.origin.position.x:.1f}, {info.origin.position.y:.1f}), "
            f"counts={counts}",
            flush=True,
        )
        if (
            info.width != 417
            or info.height != 424
            or not math.isclose(info.resolution, 0.05, abs_tol=1e-6)
            or counts != expected_counts
        ):
            raise RuntimeError("warehouse_v3 map statistics changed")

        start_tf = node.tf_buffer.lookup_transform(
            "map", "base_footprint", Time(), timeout=Duration(seconds=2.0)
        )
        start_x = start_tf.transform.translation.x
        start_y = start_tf.transform.translation.y
        start_yaw = yaw_from_quaternion(start_tf.transform.rotation)
        if args.mode == "straight":
            local_x, local_y, yaw_delta = args.distance, 0.0, 0.0
        else:
            leg = args.distance / math.sqrt(2.0)
            local_x, local_y, yaw_delta = leg, leg, math.pi / 2.0
        goal_x = start_x + local_x * math.cos(start_yaw) - local_y * math.sin(start_yaw)
        goal_y = start_y + local_x * math.sin(start_yaw) + local_y * math.cos(start_yaw)
        goal_yaw = start_yaw + yaw_delta

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation = quaternion_from_yaw(goal_yaw)
        static_value = grid_value(node.map_message, goal_x, goal_y)
        cost_value = grid_value(node.global_costmap, goal_x, goal_y)
        print(
            f"[NAV][GOAL] mode={args.mode}, start=({start_x:.3f}, {start_y:.3f}, "
            f"{math.degrees(start_yaw):.1f} deg), goal=({goal_x:.3f}, {goal_y:.3f}, "
            f"{math.degrees(goal_yaw):.1f} deg), map/cost=({static_value}, {cost_value})",
            flush=True,
        )
        if static_value != 0 or not 0 <= cost_value < 253:
            raise RuntimeError("Goal cell is not safe known free space")

        if not node.compute_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/compute_path_to_pose action is unavailable")
        compute_goal = ComputePathToPose.Goal(goal=goal, use_start=False)
        future = node.compute_client.send_goal_async(compute_goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        compute_handle = future.result()
        if compute_handle is None or not compute_handle.accepted:
            raise RuntimeError("ComputePathToPose rejected the candidate")
        result_future = compute_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
        planned = result_future.result()
        if planned is None or planned.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("ComputePathToPose did not succeed")
        planned_length = path_length(planned.result.path)
        print(
            f"[NAV][PLAN] SUCCEEDED: poses={len(planned.result.path.poses)}, "
            f"length={planned_length:.3f} m; robot has not moved",
            flush=True,
        )

        print(
            f"[NAV] Starting {args.mode} NavigateToPose in {args.startup_delay:.0f} s",
            flush=True,
        )
        deadline = time.monotonic() + args.startup_delay
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.navigate_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/navigate_to_pose action is unavailable")
        nav_goal = NavigateToPose.Goal(pose=goal)
        send_future = node.navigate_client.send_goal_async(
            nav_goal, feedback_callback=node.feedback
        )
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            raise RuntimeError("NavigateToPose rejected the goal")

        node.trajectory = [(start_x, start_y)]
        node.record_trajectory = True
        nav_result_future = nav_handle.get_result_async()
        completed = node.wait_until(lambda: nav_result_future.done(), args.timeout)
        node.record_trajectory = False
        if not completed:
            nav_handle.cancel_goal_async()
            raise RuntimeError("NavigateToPose timed out and was cancelled")
        nav_result = nav_result_future.result()

        settle_deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        final_tf = node.tf_buffer.lookup_transform(
            "map", "base_footprint", Time(), timeout=Duration(seconds=2.0)
        )
        final_x = final_tf.transform.translation.x
        final_y = final_tf.transform.translation.y
        final_yaw = yaw_from_quaternion(final_tf.transform.rotation)
        position_error = math.hypot(final_x - goal_x, final_y - goal_y)
        yaw_error = abs(angle_error(final_yaw, goal_yaw))
        actual_length = trajectory_length(node.trajectory)
        linear_speed = math.hypot(
            node.latest_odom.twist.twist.linear.x,
            node.latest_odom.twist.twist.linear.y,
        )
        angular_speed = abs(node.latest_odom.twist.twist.angular.z)
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(nav_result.status, str(nav_result.status))
        print(
            f"[NAV][RESULT] status={status_name}, recoveries={node.feedback_recoveries}, "
            f"planned/actual=({planned_length:.3f}, {actual_length:.3f}) m, "
            f"final=({final_x:.3f}, {final_y:.3f}, {math.degrees(final_yaw):.1f} deg), "
            f"errors=({position_error:.3f} m, {math.degrees(yaw_error):.2f} deg), "
            f"stopped=({linear_speed:.4f} m/s, {angular_speed:.4f} rad/s)",
            flush=True,
        )
        if (
            nav_result.status != GoalStatus.STATUS_SUCCEEDED
            or node.feedback_recoveries != 0
            or position_error > 0.12
            or yaw_error > 0.20
            or linear_speed > 0.01
            or angular_speed > 0.02
        ):
            raise RuntimeError("Navigation goal failed its acceptance limits")
        print(f"[NAV] PASS: {args.mode} goal completed safely", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
