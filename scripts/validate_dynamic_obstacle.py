#!/usr/bin/env python3
"""Validate Nav2 behavior around programmatically controlled Isaac cubes."""

import argparse
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument(
    "--scenario",
    # persistent: the detour placement for a tree without a detour (issue #1
    # obstacle_wait_retry); the goal must abort safely after one path.
    choices=("temporary", "detour", "persistent", "blocked"),
    required=True,
)
parser.add_argument("--goal-x", type=float, default=4.34064)
parser.add_argument("--goal-y", type=float, default=2.39164)
parser.add_argument("--goal-yaw", type=float, default=0.0)
parser.add_argument("--obstacle-distance", type=float, default=1.20)
parser.add_argument("--timeout", type=float, default=210.0)
parser.add_argument(
    "--ghost-margin",
    type=float,
    default=0.30,
    help="lethal cells farther than this from the cube count as ghosts; "
    "use the keep-off radius plus a cell with novel_obstacle_keepoff",
)
parser.add_argument(
    "--stale-check-s",
    type=float,
    default=0.0,
    help="after hiding the cubes, wait this long and count every lethal "
    "cell on free static map cells (evidence that was not cleared)",
)
parser.add_argument(
    "--trace-output",
    help="write plans, obstacle and the odometry trace to this JSON file",
)
args, ros_args = parser.parse_known_args()

# Carbot measured footprint in base_footprint (x forward, y left).
FOOTPRINT = ((0.155, 0.133), (0.155, -0.133), (-0.130, -0.133), (-0.130, 0.133))
# OccupancyGrid value of Nav2's LETHAL_OBSTACLE (cost 254).
LETHAL_OCCUPANCY = 100


def footprint_perimeter(step=0.01):
    """Return points along the footprint edges in base_footprint."""
    points = []
    for first, second in zip(FOOTPRINT, FOOTPRINT[1:] + FOOTPRINT[:1]):
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        count = max(1, int(math.ceil(length / step)))
        for index in range(count):
            ratio = index / count
            points.append((
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            ))
    return points


def obstacle_boxes(command):
    """Yield (x, y, size_x, size_y, yaw) for each enabled command group."""
    for index in range(0, len(command) - 6, 7):
        x, y, _, size_x, size_y, _, yaw = command[index:index + 7]
        if size_x > 0.0 and size_y > 0.0:
            yield x, y, size_x, size_y, yaw


def distance_to_box(point, box):
    """Return the planar distance from a point to a box (0 inside)."""
    x, y, size_x, size_y, yaw = box
    relative_x = point[0] - x
    relative_y = point[1] - y
    local_x = math.cos(-yaw) * relative_x - math.sin(-yaw) * relative_y
    local_y = math.sin(-yaw) * relative_x + math.cos(-yaw) * relative_y
    return math.hypot(
        max(abs(local_x) - size_x / 2.0, 0.0),
        max(abs(local_y) - size_y / 2.0, 0.0),
    )


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion

    return Quaternion(z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def status_name(status):
    return {
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
    }.get(status, str(status))


def point_and_tangent(path, distance):
    points = [
        (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
    ]
    if len(points) < 2:
        raise RuntimeError("initial path has fewer than two poses")
    remaining = distance
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        segment = math.hypot(dx, dy)
        if segment <= 1e-6:
            continue
        if remaining <= segment:
            ratio = remaining / segment
            return (
                first[0] + ratio * dx,
                first[1] + ratio * dy,
                math.atan2(dy, dx),
            )
        remaining -= segment
    first, second = points[-2:]
    return second[0], second[1], math.atan2(
        second[1] - first[1], second[0] - first[0]
    )


class DynamicObstacleValidation(Node):
    def __init__(self):
        super().__init__(f"dynamic_obstacle_validation_{args.scenario}")
        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.obstacle_publisher = self.create_publisher(
            Float64MultiArray, "/isaac_sim/dynamic_obstacle", 10
        )
        self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.global_costmap_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.static_map_callback,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # Nav2 sends the full grid only on start or resize; later changes
        # arrive as updates and must be applied to the latched grid.
        self.create_subscription(
            OccupancyGridUpdate,
            "/global_costmap/costmap_updates",
            self.global_costmap_update_callback,
            10,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.goal_started = False
        self.plan_count = 0
        self.initial_path = None
        self.latest_odom = None
        self.latest_cmd = None
        self.cmd_history = []
        self.obstacle_command = None
        self.obstacle_command_placed = None
        self.global_costmap_data = []
        self.static_map = None
        self.obstacle_enabled = False
        self.obstacle_placed_at = None
        self.obstacle_removed_at = None
        self.obstacle_seen_in_scan = False
        self.obstacle_scan_hits_max = 0
        self.was_moving = False
        self.stopped_at = None
        self.stop_observed = False
        self.maximum_stop_duration = 0.0
        self.feedback_recoveries = 0
        self.latest_global_costmap = None
        self.global_obstacle_cells_at_replan = None
        self.replan_path_clearance = None
        self.minimum_clearance = math.inf
        self.perimeter = footprint_perimeter()
        self.plans = []
        self.odom_trace = []

    def path_callback(self, message):
        if not self.goal_started:
            return
        self.plan_count += 1
        self.plans.append([
            [round(pose.pose.position.x, 3), round(pose.pose.position.y, 3)]
            for pose in message.poses
        ])
        if self.initial_path is None:
            self.initial_path = message
        elif (
            self.global_obstacle_cells_at_replan is None
            and self.obstacle_enabled
        ):
            # Issue #1: a detour is only meaningful if the global costmap
            # still holds the obstacle the robot stopped in front of.
            self.global_obstacle_cells_at_replan = (
                self.lethal_cells_in_obstacle()
            )
            self.replan_path_clearance = min(
                distance_to_box(
                    (pose.pose.position.x, pose.pose.position.y), box)
                for pose in message.poses
                for box in obstacle_boxes(self.obstacle_command)
            )
            print(
                "[DYNAMIC][GLOBAL] lethal cells inside obstacle at replan: "
                f"{self.global_obstacle_cells_at_replan}, detour centre-line "
                f"clearance {self.replan_path_clearance:.3f} m",
                flush=True,
            )
        print(
            f"[DYNAMIC][PLAN] count={self.plan_count}, "
            f"poses={len(message.poses)}",
            flush=True,
        )

    def global_costmap_callback(self, message):
        self.latest_global_costmap = message
        # rclpy converts message.data back to an array on assignment; keep
        # the patched cells in a separate buffer.
        self.global_costmap_data = list(message.data)

    def static_map_callback(self, message):
        self.static_map = message

    def ghost_cells(self, margin=0.30):
        """Count lethal global cells on free static cells away from cubes."""
        grid = self.latest_global_costmap
        static = self.static_map
        if grid is None or static is None:
            return None
        info = grid.info
        # The current cube is hidden by the time the result is reported.
        boxes = list(obstacle_boxes(self.obstacle_command_placed or []))
        count = 0
        for index, value in enumerate(self.global_costmap_data):
            if value < LETHAL_OCCUPANCY:
                continue
            x = info.origin.position.x + (
                index % info.width + 0.5) * info.resolution
            y = info.origin.position.y + (
                index // info.width + 0.5) * info.resolution
            column = int((x - static.info.origin.position.x)
                         / static.info.resolution)
            row = int((y - static.info.origin.position.y)
                      / static.info.resolution)
            if not (0 <= column < static.info.width
                    and 0 <= row < static.info.height):
                continue
            if static.data[row * static.info.width + column] != 0:
                continue
            if any(distance_to_box((x, y), box) <= margin for box in boxes):
                continue
            count += 1
        return count

    def global_costmap_update_callback(self, message):
        grid = self.latest_global_costmap
        if grid is None:
            return
        for row in range(message.height):
            start = (message.y + row) * grid.info.width + message.x
            self.global_costmap_data[start:start + message.width] = (
                message.data[row * message.width:(row + 1) * message.width])

    def lethal_cells_in_obstacle(self):
        grid = self.latest_global_costmap
        if grid is None or self.obstacle_command is None:
            return 0
        info = grid.info
        count = 0
        for box in obstacle_boxes(self.obstacle_command):
            radius = math.hypot(box[2], box[3]) / 2.0
            first_column = int((box[0] - radius - info.origin.position.x)
                               / info.resolution)
            last_column = int((box[0] + radius - info.origin.position.x)
                              / info.resolution)
            first_row = int((box[1] - radius - info.origin.position.y)
                            / info.resolution)
            last_row = int((box[1] + radius - info.origin.position.y)
                           / info.resolution)
            for row in range(max(first_row, 0),
                             min(last_row, info.height - 1) + 1):
                for column in range(max(first_column, 0),
                                    min(last_column, info.width - 1) + 1):
                    centre = (
                        info.origin.position.x
                        + (column + 0.5) * info.resolution,
                        info.origin.position.y
                        + (row + 0.5) * info.resolution,
                    )
                    if (
                        distance_to_box(centre, box) == 0.0
                        and self.global_costmap_data[row * info.width + column]
                        >= LETHAL_OCCUPANCY
                    ):
                        count += 1
        return count

    def update_clearance(self):
        if not self.obstacle_enabled or self.latest_odom is None:
            return
        pose = self.latest_odom.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for box in obstacle_boxes(self.obstacle_command):
            for local_x, local_y in self.perimeter:
                point = (
                    pose.position.x + cosine * local_x - sine * local_y,
                    pose.position.y + sine * local_x + cosine * local_y,
                )
                self.minimum_clearance = min(
                    self.minimum_clearance, distance_to_box(point, box)
                )

    def odom_callback(self, message):
        self.latest_odom = message
        self.update_clearance()
        if self.goal_started:
            pose = message.pose.pose
            self.odom_trace.append([
                round(time.monotonic(), 2),
                round(pose.position.x, 3),
                round(pose.position.y, 3),
                round(2.0 * math.atan2(pose.orientation.z,
                                       pose.orientation.w), 3),
            ])

    def cmd_callback(self, message):
        self.latest_cmd = message
        speed = abs(message.linear.x) + 0.25 * abs(message.angular.z)
        self.cmd_history.append((time.monotonic(), speed))
        self.cmd_history = self.cmd_history[-200:]

    def scan_callback(self, message):
        if not self.obstacle_enabled or self.latest_odom is None:
            return
        obstacle_x, obstacle_y = self.obstacle_command[:2]
        size_x, size_y = self.obstacle_command[3:5]
        obstacle_yaw = self.obstacle_command[6]
        pose = self.latest_odom.pose.pose
        orientation = pose.orientation
        robot_yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        cosine = math.cos(-obstacle_yaw)
        sine = math.sin(-obstacle_yaw)
        hits = 0
        for index, scan_range in enumerate(message.ranges):
            if not math.isfinite(scan_range):
                continue
            angle = (
                robot_yaw
                + message.angle_min
                + index * message.angle_increment
            )
            relative_x = (
                pose.position.x + scan_range * math.cos(angle) - obstacle_x
            )
            relative_y = (
                pose.position.y + scan_range * math.sin(angle) - obstacle_y
            )
            local_x = cosine * relative_x - sine * relative_y
            local_y = sine * relative_x + cosine * relative_y
            if (
                abs(local_x) <= size_x / 2.0 + 0.12
                and abs(local_y) <= size_y / 2.0 + 0.12
            ):
                hits += 1
        self.obstacle_scan_hits_max = max(self.obstacle_scan_hits_max, hits)
        if hits and not self.obstacle_seen_in_scan:
            self.obstacle_seen_in_scan = True
            print(
                f"[DYNAMIC][SCAN] obstacle detected with {hits} rays",
                flush=True,
            )

    def feedback_callback(self, message):
        self.feedback_recoveries = max(
            self.feedback_recoveries,
            int(message.feedback.number_of_recoveries),
        )

    def publish_obstacle(self, values):
        message = Float64MultiArray()
        message.data = [float(value) for value in values]
        self.obstacle_publisher.publish(message)

    def hide_obstacle(self):
        self.obstacle_command = [0.0, 0.0, 0.0, -1.0, -1.0, -1.0, 0.0]
        self.publish_obstacle(self.obstacle_command)
        self.obstacle_enabled = False
        self.obstacle_removed_at = time.monotonic()
        print("[DYNAMIC][OBSTACLE] hidden", flush=True)

    def place_obstacle_from_initial_path(self):
        x, y, tangent = point_and_tangent(
            self.initial_path, args.obstacle_distance
        )
        if args.scenario == "blocked":
            # Four physical cubes form a closed ring around the robot's pose
            # when the obstacle is placed (cube commands share the map/odom
            # frame of the plan). All walls are inside LiDAR range, so neither
            # local control nor global planning can exploit an unseen end of
            # a finite wall.
            centre = self.latest_odom.pose.pose.position
            wall_distance = 1.0
            wall_length = 4.0
            wall_thickness = 0.8
            walls = (
                (wall_distance, 0.0, wall_thickness, wall_length),
                (-wall_distance, 0.0, wall_thickness, wall_length),
                (0.0, wall_distance, wall_length, wall_thickness),
                (0.0, -wall_distance, wall_length, wall_thickness),
            )
            self.obstacle_command = []
            for wall_x, wall_y, size_x, size_y in walls:
                self.obstacle_command.extend([
                    centre.x + wall_x, centre.y + wall_y, 0.5,
                    size_x, size_y, 1.0, 0.0,
                ])
            dimensions = (wall_thickness, wall_length, 1.0)
            x, y, yaw = centre.x + walls[0][0], centre.y + walls[0][1], 0.0
        else:
            dimensions = (0.65, 0.65, 1.0)
            yaw = 0.0
            self.obstacle_command = [x, y, 0.5, *dimensions, yaw]
        self.publish_obstacle(self.obstacle_command)
        self.obstacle_command_placed = list(self.obstacle_command)
        self.obstacle_enabled = True
        self.obstacle_seen_in_scan = False
        self.obstacle_scan_hits_max = 0
        self.obstacle_placed_at = time.monotonic()
        print(
            "[DYNAMIC][OBSTACLE] enabled "
            f"at=({x:.3f}, {y:.3f}), size={dimensions}, "
            f"yaw={math.degrees(yaw):.1f} deg",
            flush=True,
        )

    def speed_measure(self):
        if self.latest_cmd is None:
            return 0.0
        return abs(self.latest_cmd.linear.x) + 0.25 * abs(
            self.latest_cmd.angular.z
        )

    def update_stop_detection(self):
        speed = self.speed_measure()
        if speed >= 0.04:
            self.was_moving = True
            if self.stopped_at is not None:
                self.maximum_stop_duration = max(
                    self.maximum_stop_duration,
                    time.monotonic() - self.stopped_at,
                )
            self.stopped_at = None
            return
        if self.was_moving and self.obstacle_enabled:
            if self.stopped_at is None:
                self.stopped_at = time.monotonic()
            duration = time.monotonic() - self.stopped_at
            self.maximum_stop_duration = max(
                self.maximum_stop_duration, duration
            )
            if duration >= 0.75 and not self.stop_observed:
                self.stop_observed = True
                print(
                    f"[DYNAMIC][STOP] observed after {duration:.2f}s "
                    "below threshold",
                    flush=True,
                )


def wait_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    rclpy.init(args=ros_args)
    node = DynamicObstacleValidation()
    result_summary = {"scenario": args.scenario}
    try:
        ready = wait_until(
            node,
            lambda: (
                node.latest_odom is not None
                and node.obstacle_publisher.get_subscription_count() >= 1
                and node.action_client.server_is_ready()
            ),
            20.0,
        )
        if not ready:
            raise RuntimeError(
                "timed out waiting for odom, obstacle control, or Nav2"
            )

        node.hide_obstacle()
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = args.goal_x
        goal.pose.position.y = args.goal_y
        goal.pose.orientation = quaternion_from_yaw(args.goal_yaw)

        node.goal_started = True
        send_future = node.action_client.send_goal_async(
            NavigateToPose.Goal(pose=goal),
            feedback_callback=node.feedback_callback,
        )
        if not wait_until(node, send_future.done, 10.0):
            raise RuntimeError("timed out sending navigation goal")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("navigation goal was rejected")
        result_future = goal_handle.get_result_async()
        if not wait_until(node, lambda: node.initial_path is not None, 15.0):
            raise RuntimeError("initial /plan was not observed")

        node.place_obstacle_from_initial_path()
        # Repeat briefly so late discovery cannot lose the validation command.
        repeat_until = time.monotonic() + 1.0
        while time.monotonic() < repeat_until:
            node.publish_obstacle(node.obstacle_command)
            rclpy.spin_once(node, timeout_sec=0.05)

        deadline = time.monotonic() + args.timeout
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not result_future.done()
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
            node.update_stop_detection()
            if (
                args.scenario == "temporary"
                and node.stop_observed
                and node.obstacle_enabled
            ):
                node.hide_obstacle()

        if not result_future.done():
            goal_handle.cancel_goal_async()
            raise RuntimeError("navigation did not finish before timeout")
        result = result_future.result()

        settle_until = time.monotonic() + 2.0
        while time.monotonic() < settle_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.hide_obstacle()

        final_status = status_name(result.status)
        # Issue #1: the global costmap is never cleared after the robot has
        # moved, so lethal cells left by earlier obstacles must not persist.
        ghost_cells = node.ghost_cells(args.ghost_margin)
        stale_cells = None
        if args.stale_check_s > 0.0:
            stale_until = time.monotonic() + args.stale_check_s
            while time.monotonic() < stale_until:
                rclpy.spin_once(node, timeout_sec=0.05)
            placed = node.obstacle_command_placed
            node.obstacle_command_placed = None
            stale_cells = node.ghost_cells(0.0)
            node.obstacle_command_placed = placed
        recent_cmd_speeds = [
            speed
            for received_at, speed in node.cmd_history
            if received_at >= settle_until - 0.75
        ]
        recent_cmd_speed = max(recent_cmd_speeds, default=0.0)
        odom_twist = node.latest_odom.twist.twist
        final_odom_speed = abs(odom_twist.linear.x) + 0.25 * abs(
            odom_twist.angular.z
        )
        result_summary.update(
            {
                "status": final_status,
                "plan_count": node.plan_count,
                "stop_observed": node.stop_observed,
                "obstacle_seen_in_scan": node.obstacle_seen_in_scan,
                "obstacle_scan_hits_max": node.obstacle_scan_hits_max,
                "maximum_stop_duration_s": round(
                    node.maximum_stop_duration, 3
                ),
                "recoveries": node.feedback_recoveries,
                "post_result_recent_cmd_speed_max": round(
                    recent_cmd_speed, 4
                ),
                "post_result_odom_speed": round(final_odom_speed, 4),
                "global_obstacle_cells_at_replan": (
                    node.global_obstacle_cells_at_replan
                ),
                "global_ghost_cells": ghost_cells,
                "stale_keepoff_cells": stale_cells,
                "replan_path_clearance_m": (
                    None if node.replan_path_clearance is None
                    else round(node.replan_path_clearance, 3)
                ),
                "minimum_footprint_clearance_m": round(
                    node.minimum_clearance, 3
                ),
            }
        )
        if args.trace_output:
            with open(args.trace_output, "w", encoding="utf-8") as stream:
                json.dump({
                    "summary": result_summary,
                    "obstacle_command": node.obstacle_command_placed,
                    "plans": node.plans,
                    "odom": node.odom_trace,
                }, stream)
        print(
            f"[DYNAMIC][RESULT] "
            f"{json.dumps(result_summary, sort_keys=True)}"
        )

        if not node.obstacle_seen_in_scan:
            raise RuntimeError("dynamic obstacle was not observed in /scan")
        if not node.stop_observed:
            raise RuntimeError("local obstacle stop was not observed")
        if node.minimum_clearance <= 0.0:
            raise RuntimeError("footprint touched the obstacle")
        if recent_cmd_speed > 0.01 or final_odom_speed > 0.01:
            raise RuntimeError(
                "non-zero velocity persisted after action result"
            )
        if args.scenario == "temporary":
            passed = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and node.plan_count == 1
            )
        elif args.scenario == "persistent":
            passed = (
                result.status == GoalStatus.STATUS_ABORTED
                and node.plan_count == 1
            )
        elif args.scenario == "detour":
            passed = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and node.plan_count == 2
                and (node.global_obstacle_cells_at_replan or 0) > 0
            )
        else:
            passed = (
                result.status == GoalStatus.STATUS_ABORTED
                and 1 <= node.plan_count <= 3
            )
        if not passed:
            raise RuntimeError(
                "scenario result did not meet its acceptance criteria"
            )
        print(f"[DYNAMIC] PASS: {args.scenario}", flush=True)
    finally:
        if node.obstacle_publisher.get_subscription_count() >= 1:
            node.hide_obstacle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
