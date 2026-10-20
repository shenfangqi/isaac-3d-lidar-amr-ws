"""
Back the robot away from an obstacle it stopped against.

RPP refuses to turn when the footprint's sweep reaches an obstacle, and the
real behavior tree had no escape: on 2026-10-18 the robot stopped about
6 cm from a sofa corner and every later goal failed at once.  Nav2's BackUp
cannot help there.  Its first collision check is the current pose, and with
5 cm cells an obstacle 5-7 cm away shares a cell with the footprint edge, so
the robot already "collides" and BackUp refuses to move although the space
behind it is free (reproduced in Isaac at a wall end).

This node serves the same nav2_msgs/action/BackUp interface under its own
name (the behavior tree's BackUp node selects it with ``server_name``).
It backs straight up and judges collision by cell centres:

* a local-costmap cell is inside the footprint when its centre lies inside
  the footprint polygon;
* the cells inside the footprint at the start pose are ignored, since that
  is where the robot already stands;
* before every command, the footprint is swept back over the remaining
  distance plus ``lookahead_m``; any other lethal cell inside it within
  ``lookahead_m`` plus the distance the robot can cover while the costmap
  ages stops the robot.

If a cell behind stops it, or ``time_allowance`` runs out, before
``min_progress_m``, it drives forward the requested distance under the same
check instead (``forward_fallback``): on
2026-10-18 the robot stood with its rear 2-4 cm from a wall, could neither
turn nor back up, and the space ahead was free.  RPP's refusal was about
the turn, so a short straight move ahead is checked like any other.

Each move starts with ``settle_s`` of zero commands.  The ESP32 wheel PID
(2wheel-carbot-esp32 motor_pid_controller.c) clears its integral only when
the target is exactly zero; the velocity smoother ramps from forward to
reverse without an exact zero, so the forward integral held the motors off
and the robot backed up 0.03-0.05 m in 6 s (robot, 2026-10-18).  Holding
zero lets the smoother output an exact zero first (0.5 s from 0.10 m/s at
0.20 m/s^2).

The result is SUCCEEDED when the robot moved at least ``min_progress_m``
(the requested distance, or less when a cell stopped it) and FAILED
otherwise.
"""

import math

import numpy as np

LETHAL = 254


def footprint_from_string(text):
    """Parse a Nav2 footprint string into a list of (x, y) vertices."""
    import json
    return [tuple(float(v) for v in point) for point in json.loads(text)]


def _inside_polygon(x, y, polygon):
    """Vectorised even-odd test of points against a closed polygon."""
    inside = np.zeros(x.shape, dtype=bool)
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        inside ^= crosses & (x < x_cross)
    return inside


def footprint_cells(grid, pose, footprint):
    """Return the set of (row, col) cells whose centres lie inside the footprint at pose."""
    x0, y0, yaw = pose
    reach = max(math.hypot(px, py) for px, py in footprint)
    res = grid['resolution']
    col_min = max(int(math.floor((x0 - reach - grid['origin_x']) / res)), 0)
    col_max = min(int(math.ceil((x0 + reach - grid['origin_x']) / res)),
                  grid['width'] - 1)
    row_min = max(int(math.floor((y0 - reach - grid['origin_y']) / res)), 0)
    row_max = min(int(math.ceil((y0 + reach - grid['origin_y']) / res)),
                  grid['height'] - 1)
    if col_min > col_max or row_min > row_max:
        return set()
    cols, rows = np.meshgrid(np.arange(col_min, col_max + 1),
                             np.arange(row_min, row_max + 1))
    cx = grid['origin_x'] + (cols + 0.5) * res - x0
    cy = grid['origin_y'] + (rows + 0.5) * res - y0
    c, s = math.cos(yaw), math.sin(yaw)
    bx = c * cx + s * cy
    by = -s * cx + c * cy
    inside = _inside_polygon(bx, by, footprint)
    return set(zip(rows[inside].tolist(), cols[inside].tolist()))


def first_blocked_distance(grid, costs, start_pose, pose, footprint,
                           distance, step=0.01, direction=-1.0):
    """
    Sweep the footprint straight back (direction -1) or ahead (+1) by distance.

    Returns the distance at which a lethal cell that was not under the
    footprint at start_pose enters it, or None when the sweep is clear.
    """
    ignored = footprint_cells(grid, start_pose, footprint)
    x0, y0, yaw = pose
    steps = max(int(math.ceil(distance / step)), 1)
    for index in range(0, steps + 1):
        back = min(index * step, distance)
        moved = (x0 + direction * back * math.cos(yaw),
                 y0 + direction * back * math.sin(yaw), yaw)
        for row, col in footprint_cells(grid, moved, footprint) - ignored:
            if costs[row, col] >= LETHAL:
                return back
    return None


def main(args=None):
    import threading
    import time

    import rclpy
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import BackUp
    from nav2_msgs.msg import Costmap
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.duration import Duration
    from rclpy.executors import ExternalShutdownException
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class ReverseEscape(Node):
        """Serve nav2_msgs/action/BackUp with a start-pose-aware collision check."""

        def __init__(self):
            super().__init__('reverse_escape')
            self.declare_parameter('action_name', 'reverse_escape')
            self.declare_parameter('costmap_topic',
                                   '/local_costmap/costmap_raw')
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('base_frame', 'base_footprint')
            self.declare_parameter(
                'footprint',
                '[[0.155, 0.133], [0.155, -0.133], [-0.130, -0.133], '
                '[-0.130, 0.133]]')
            self.declare_parameter('max_speed_mps', 0.05)
            self.declare_parameter('lookahead_m', 0.03)
            self.declare_parameter('min_progress_m', 0.05)
            self.declare_parameter('forward_fallback', True)
            self.declare_parameter('settle_s', 0.7)
            self.declare_parameter('rate_hz', 20.0)
            # The local costmap publishes costmap_raw at 5 Hz.
            self.declare_parameter('max_costmap_age_s', 1.5)
            # The costmap is taken only while a goal runs; allow this long
            # (beyond settle_s) for the first one.
            self.declare_parameter('input_wait_s', 2.0)
            self._footprint = footprint_from_string(
                self.get_parameter('footprint').value)
            self._frame = self.get_parameter('global_frame').value
            self._base = self.get_parameter('base_frame').value
            self._costmap = None
            self._costmap_time = None
            self._lock = threading.Lock()
            self._active = False
            group = ReentrantCallbackGroup()
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            # The costmap is decoded only while a goal runs (~0.1 core on
            # the Jetson otherwise).  Subscribing per goal and destroying
            # the subscription afterwards races the multi-threaded
            # executor (2026-10-20: it killed the motion guard).
            self.create_subscription(
                Costmap, self.get_parameter('costmap_topic').value,
                self._on_costmap, 1, callback_group=group)
            # Remapped to cmd_vel_nav in the launch file, like the
            # controller and behavior_server.
            self._cmd = self.create_publisher(Twist, 'cmd_vel', 1)
            self._server = ActionServer(
                self, BackUp, self.get_parameter('action_name').value,
                execute_callback=self._execute,
                goal_callback=self._on_goal,
                cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=group)

        def _on_costmap(self, message):
            if not self._active:
                return
            meta = message.metadata
            grid = {
                'resolution': meta.resolution,
                'origin_x': meta.origin.position.x,
                'origin_y': meta.origin.position.y,
                'width': meta.size_x, 'height': meta.size_y,
                'frame': message.header.frame_id,
            }
            costs = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(
                meta.size_y, meta.size_x)
            with self._lock:
                self._costmap = (grid, costs)
                self._costmap_time = Time.from_msg(message.header.stamp)

        def _forget_costmap(self):
            with self._lock:
                self._costmap = None
                self._costmap_time = None

        def _inputs_ready(self):
            with self._lock:
                stamp = self._costmap_time
            if stamp is None:
                return False
            age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
            return (age <= self.get_parameter('max_costmap_age_s').value
                    and self._tf_buffer.can_transform(
                        self._frame, self._base, Time()))

        def _on_goal(self, _request):
            return GoalResponse.REJECT if self._active else GoalResponse.ACCEPT

        def _pose(self, stats=None):
            stamped = self._tf_buffer.lookup_transform(
                self._frame, self._base, Time(),
                timeout=Duration(seconds=0.2))
            # A static transform carries a zero stamp: no age to report.
            if stats is not None and stamped.header.stamp.sec > 0:
                age = (self.get_clock().now() - Time.from_msg(
                    stamped.header.stamp)).nanoseconds * 1e-9
                stats['tf_age_max'] = max(stats['tf_age_max'], age)
            transform = stamped.transform
            q = transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return transform.translation.x, transform.translation.y, yaw

        def _stop(self):
            self._cmd.publish(Twist())

        def _execute(self, handle):
            # A costmap kept from an earlier goal is never used.
            self._forget_costmap()
            self._active = True
            try:
                return self._run(handle)
            finally:
                self._stop()
                self._active = False

        def _run(self, handle):
            goal = handle.request
            result = BackUp.Result()
            started = self.get_clock().now()
            target = abs(goal.target.x)
            speed = min(abs(goal.speed),
                        self.get_parameter('max_speed_mps').value)
            allowance = (goal.time_allowance.sec
                         + goal.time_allowance.nanosec * 1e-9) or 10.0
            min_progress = self.get_parameter('min_progress_m').value

            def finish(succeeded, summary):
                elapsed = self.get_clock().now() - started
                result.total_elapsed_time = elapsed.to_msg()
                message = f'reverse escape {summary} (of {target:.2f} m)'
                # rclpy fixes one severity per logging call site.
                if succeeded:
                    self.get_logger().info(message)
                else:
                    self.get_logger().warning(message)
                (handle.succeed if succeeded else handle.abort)()
                return result

            if speed <= 0.0 or target <= 0.0:
                return finish(False, 'rejected an empty request')
            outcome, back, reason = self._drive(
                handle, -1.0, target, speed, allowance)
            summary = f'{reason}: backed up {back:.3f} m'
            if outcome == 'canceled':
                handle.canceled()
                self.get_logger().info(f'reverse escape canceled, {summary}')
                return result
            if back >= min_progress:
                return finish(True, summary)
            # A backward move that stalled (2026-10-18: 0.032 m in 6 s on the
            # robot, cause unknown) is treated like one a cell blocked.
            if (outcome not in ('blocked', 'timeout')
                    or not self.get_parameter('forward_fallback').value):
                return finish(False, summary)
            outcome, ahead, reason = self._drive(
                handle, 1.0, target, speed, allowance)
            summary += f'; {reason}: forward {ahead:.3f} m'
            if outcome == 'canceled':
                handle.canceled()
                self.get_logger().info(f'reverse escape canceled, {summary}')
                return result
            return finish(ahead >= min_progress, summary)

        def _drive(self, handle, direction, target, speed, allowance):
            """Drive straight (direction -1 back, +1 ahead); return (outcome, distance, reason)."""
            outcome, traveled, reason, stats = self._drive_loop(
                handle, direction, target, speed, allowance)
            # Loop timing and data ages, to explain a stalled move.
            if stats['cycles']:
                reason += (f' [{stats["cycles"]} cycles, period mean '
                           f'{stats["period_sum"] / stats["cycles"]:.3f} max '
                           f'{stats["period_max"]:.3f} s, tf age max '
                           f'{stats["tf_age_max"]:.2f} s, costmap age max '
                           f'{stats["costmap_age_max"]:.2f} s]')
            return outcome, traveled, reason

        def _drive_loop(self, handle, direction, target, speed, allowance):
            stats = {'cycles': 0, 'period_sum': 0.0, 'period_max': 0.0,
                     'tf_age_max': 0.0, 'costmap_age_max': 0.0}
            lookahead = self.get_parameter('lookahead_m').value
            max_age = self.get_parameter('max_costmap_age_s').value
            period = 1.0 / self.get_parameter('rate_hz').value
            side = 'behind' if direction < 0 else 'ahead'
            settle_until = time.monotonic() + self.get_parameter(
                'settle_s').value
            inputs_until = settle_until + self.get_parameter(
                'input_wait_s').value
            while rclpy.ok() and (
                    time.monotonic() < settle_until
                    or (time.monotonic() < inputs_until
                        and not self._inputs_ready())):
                if handle.is_cancel_requested:
                    return 'canceled', 0.0, 'canceled', stats
                self._stop()
                time.sleep(period)
            deadline = time.monotonic() + allowance
            try:
                start = self._pose()
            except TransformException as error:
                return 'error', 0.0, f'no pose ({error})', stats
            traveled = 0.0
            feedback = BackUp.Feedback()
            last = None
            try:
                while rclpy.ok():
                    now = time.monotonic()
                    if last is not None:
                        stats['period_sum'] += now - last
                        stats['period_max'] = max(stats['period_max'],
                                                  now - last)
                        stats['cycles'] += 1
                    last = now
                    if handle.is_cancel_requested:
                        return 'canceled', traveled, 'canceled', stats
                    try:
                        pose = self._pose(stats)
                    except TransformException as error:
                        return 'error', traveled, f'lost its pose ({error})', stats
                    traveled = math.hypot(pose[0] - start[0],
                                          pose[1] - start[1])
                    feedback.distance_traveled = float(traveled)
                    handle.publish_feedback(feedback)
                    if traveled >= target:
                        return 'done', traveled, 'done', stats
                    if time.monotonic() > deadline:
                        return 'timeout', traveled, 'timed out', stats
                    with self._lock:
                        costmap, stamp = self._costmap, self._costmap_time
                    age = (None if stamp is None else
                           (self.get_clock().now() - stamp).nanoseconds * 1e-9)
                    if age is not None:
                        stats['costmap_age_max'] = max(
                            stats['costmap_age_max'], age)
                    if (costmap is None or costmap[0]['frame'] != self._frame
                            or age > max_age):
                        return 'error', traveled, 'no fresh local costmap', stats
                    # Cells may have appeared since the costmap was taken.
                    margin = lookahead + speed * max(age, 0.0)
                    blocked = first_blocked_distance(
                        costmap[0], costmap[1], start, pose, self._footprint,
                        target - traveled + margin, direction=direction)
                    if blocked is not None and blocked <= margin:
                        return 'blocked', traveled, \
                            f'blocked {blocked:.2f} m {side}', stats
                    command = Twist()
                    command.linear.x = direction * speed
                    self._cmd.publish(command)
                    time.sleep(period)
                return 'error', traveled, 'interrupted', stats
            finally:
                self._stop()

    rclpy.init(args=args)
    node = ReverseEscape()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
