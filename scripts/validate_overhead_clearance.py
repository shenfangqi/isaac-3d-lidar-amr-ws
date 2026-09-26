#!/usr/bin/env python3
"""Validate permanent 0.40 m pass and 0.28 m blocked overhead beams."""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


PASS_GATE = (2.0, 0.0, 0.40, 1.00)
BLOCKED_GATE = (-2.0, 0.0, 0.40, 1.00)
PASS_GOAL = (4.0, 0.0, 0.0)
HOME_GOAL = (0.0, 0.0, math.pi)
BLOCKED_GOAL = (-4.0, 0.0, math.pi)
ROBOT_HALF_WIDTH_M = 0.133


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def status_name(status):
    return {
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
    }.get(status, str(status))


def wait_until(node, predicate, timeout_s):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


class OverheadClearanceValidation(Node):
    def __init__(self):
        super().__init__("overhead_clearance_validation")
        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.latest_odom = None
        self.latest_cmd = None
        self.active_paths = []
        self.goal_active = False
        self.pass_gate_hits_max = 0
        self.blocked_gate_hits_max = 0
        self.scan_samples = 0

    def odom_callback(self, message):
        self.latest_odom = message

    def path_callback(self, message):
        if self.goal_active:
            self.active_paths.append(message)

    def cmd_callback(self, message):
        self.latest_cmd = message

    def scan_callback(self, message):
        if self.latest_odom is None:
            return
        pose = self.latest_odom.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        hit_counts = [0, 0]
        angle = message.angle_min
        for scan_range in message.ranges:
            if math.isfinite(scan_range):
                local_x = scan_range * math.cos(angle)
                local_y = scan_range * math.sin(angle)
                map_x = pose.position.x + cosine * local_x - sine * local_y
                map_y = pose.position.y + sine * local_x + cosine * local_y
                for index, gate in enumerate((PASS_GATE, BLOCKED_GATE)):
                    gate_x, gate_y, size_x, size_y = gate
                    if (
                        abs(map_x - gate_x) <= size_x / 2.0 + 0.08
                        and abs(map_y - gate_y) <= size_y / 2.0 + 0.08
                    ):
                        hit_counts[index] += 1
            angle += message.angle_increment
        self.pass_gate_hits_max = max(
            self.pass_gate_hits_max, hit_counts[0]
        )
        self.blocked_gate_hits_max = max(
            self.blocked_gate_hits_max, hit_counts[1]
        )
        self.scan_samples += 1

    def reset_paths(self):
        self.active_paths = []

    def send_goal(self, name, target, timeout_s=180.0):
        self.reset_paths()
        self.goal_active = True
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = target[0]
        goal.pose.position.y = target[1]
        goal.pose.orientation = quaternion_from_yaw(target[2])
        future = self.action_client.send_goal_async(
            NavigateToPose.Goal(pose=goal)
        )
        if not wait_until(self, future.done, 10.0):
            raise RuntimeError(f"{name}: timed out sending goal")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{name}: goal was rejected")
        result_future = handle.get_result_async()
        if not wait_until(self, result_future.done, timeout_s):
            handle.cancel_goal_async()
            raise RuntimeError(f"{name}: navigation timed out")
        self.goal_active = False
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"{name}: expected SUCCEEDED, got "
                f"{status_name(result.status)}"
            )
        if not self.active_paths:
            raise RuntimeError(f"{name}: no /plan was observed")
        print(
            f"[OVERHEAD][{name}] SUCCEEDED plans={len(self.active_paths)}",
            flush=True,
        )
        return self.active_paths[-1]


def minimum_lateral_offset_at_gate(path, gate):
    gate_x, gate_y, size_x, _ = gate
    near_gate = [
        pose.pose.position
        for pose in path.poses
        if abs(pose.pose.position.x - gate_x) <= size_x / 2.0 + 0.10
    ]
    if not near_gate:
        return math.inf
    return min(abs(point.y - gate_y) for point in near_gate)


def main():
    rclpy.init()
    node = OverheadClearanceValidation()
    summary = {}
    try:
        ready = wait_until(
            node,
            lambda: (
                node.latest_odom is not None
                and node.scan_samples >= 20
                and node.action_client.server_is_ready()
            ),
            30.0,
        )
        if not ready:
            raise RuntimeError("timed out waiting for odom, scan, or Nav2")

        summary["pass_gate_scan_hits"] = node.pass_gate_hits_max
        summary["blocked_gate_scan_hits"] = node.blocked_gate_hits_max
        if node.pass_gate_hits_max != 0:
            raise RuntimeError(
                "0.40 m gate unexpectedly entered the 0.35 m /scan slice"
            )
        if node.blocked_gate_hits_max == 0:
            raise RuntimeError("0.28 m gate was not observed in /scan")

        pass_path = node.send_goal("PASS_40CM", PASS_GOAL)
        pass_offset = minimum_lateral_offset_at_gate(pass_path, PASS_GATE)
        summary["pass_40cm_min_lateral_offset_m"] = round(pass_offset, 3)
        if pass_offset > 0.35:
            raise RuntimeError("0.40 m route did not pass under its beam")

        node.send_goal("RETURN_HOME", HOME_GOAL)
        blocked_path = node.send_goal("BLOCK_28CM", BLOCKED_GOAL)
        blocked_offset = minimum_lateral_offset_at_gate(
            blocked_path, BLOCKED_GATE
        )
        summary["blocked_28cm_min_lateral_offset_m"] = round(
            blocked_offset, 3
        )
        blocked_collision_limit = BLOCKED_GATE[3] / 2.0 + (
            ROBOT_HALF_WIDTH_M
        )
        if blocked_offset <= blocked_collision_limit:
            raise RuntimeError("0.28 m route incorrectly passed under beam")

        wait_until(node, lambda: False, 1.0)
        cmd_speed = 0.0
        if node.latest_cmd is not None:
            cmd_speed = abs(node.latest_cmd.linear.x) + abs(
                node.latest_cmd.angular.z
            )
        summary.update(
            {
                "pass_40cm": "SUCCEEDED_UNDER_BEAM",
                "blocked_28cm": "SUCCEEDED_BY_DETOUR",
                "final_cmd_speed": round(cmd_speed, 4),
            }
        )
        if cmd_speed > 0.01:
            raise RuntimeError("non-zero command remained after validation")
        print(
            f"[OVERHEAD][RESULT] {json.dumps(summary, sort_keys=True)}",
            flush=True,
        )
        print("[OVERHEAD] PASS", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
