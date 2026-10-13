#!/usr/bin/env python3
"""
Supervised short straight runs on the Issue #53 guard control chain.

Publishes only to /cmd_vel_command (-> compensator -> /cmd_vel -> ESP32),
the chain the translation guard will use; Nav2 must be inactive.  An
operator must be at the robot, able to cut the ESP32 power at once (the
car has no standard e-stop): pass --operator-present to run at all.

Modes:
  stops     forward then backward runs of --distance at --speed, each
            followed by 3 s of zero commands (linear-profile data); every
            run starts forward, so a backward run only retraces ground the
            robot has just crossed;
  watchdog  drive for --hold seconds, then stop publishing entirely (no
            zero) and measure how long the chassis watchdog takes to stop
            the robot; trials alternate forward/backward.  If it still
            moves after --safety-net seconds, zero is published and the
            trial is reported as FAIL.

Every 50 ms the script aborts (3 s of zero commands, nonzero exit) on a
localization emergency, a blocked/disconnected/stale chassis, stale
odometry, a lateral offset beyond --max-lateral, a heading change beyond
--max-yaw-deg, an overrun 0.20 m past the target, a position more than
--max-excursion from where the script started, or more than --max-path of
total travel.  It sees no obstacles: keep the straight line clear.
Record /cmd_vel_command and /odom (and /fast_lio/cloud_registered_body for
the submap work) with ``ros2 bag record`` while it runs, then analyse with
scripts/analyze_linear_profile.py.
"""

import argparse
import json
import math
import sys
import threading
import time

PERIOD = 0.05
OVERRUN_M = 0.20


class Abort(RuntimeError):
    pass


def along_and_lateral(origin, x, y):
    """Signed travel along the origin heading and lateral offset from that line."""
    dx, dy = x - origin[1], y - origin[2]
    c, s = math.cos(origin[3]), math.sin(origin[3])
    return dx * c + dy * s, -dx * s + dy * c


def turned(origin_yaw, yaw):
    return abs(math.atan2(math.sin(yaw - origin_yaw), math.cos(yaw - origin_yaw)))


def check_envelope(args):
    """Raise SystemExit if the arguments leave the supervised envelope."""
    if not args.operator_present:
        raise SystemExit('refusing: pass --operator-present only with an operator at the '
                         'robot, able to cut the ESP32 power at once')
    if not (0.0 < args.speed <= 0.10 and 0.05 <= args.distance <= 0.60
            and 1 <= args.pairs <= 6 and 1 <= args.trials <= 6
            and 0.0 < args.hold <= 3.0 and args.hold * args.speed <= 0.30
            and 0.5 <= args.safety_net <= 2.0
            and 0.0 < args.max_lateral <= 0.15 and 0.0 < args.max_yaw_deg <= 15.0
            and args.distance + OVERRUN_M <= args.max_excursion <= 1.0
            and 0.0 < args.max_path <= 6.0):
        raise SystemExit('argument outside the supervised envelope')


class Capture:
    def __init__(self, node, args):
        from carbot_msgs.msg import CarbotStatus, WheelTicks
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                               ReliabilityPolicy)
        from std_msgs.msg import Bool
        self.Twist = Twist
        self.node = node
        self.args = args
        self.lock = threading.Lock()
        self.odom = None            # (t, x, y, yaw)
        self.chassis = None         # (t, connected, blocked, battery_low)
        self.emergency = None
        self.ticks = None
        self.ticks_changed = None
        self.path = 0.0
        self.home = None            # first odometry pose: the excursion centre
        self._last = None
        self.history = []           # (t, x, y) for position-derived speed
        self.publisher = node.create_publisher(Twist, '/cmd_vel_command', 10)
        node.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        node.create_subscription(CarbotStatus, '/carbot/status', self._on_chassis,
                                 qos_profile_sensor_data)
        node.create_subscription(WheelTicks, '/wheel_ticks', self._on_ticks,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/localization/emergency_stop', self._on_emergency,
                                 latched)

    def _on_odom(self, message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        x, y = message.pose.pose.position.x, message.pose.pose.position.y
        now = time.monotonic()
        with self.lock:
            if self._last is not None:
                self.path += math.hypot(x - self._last[0], y - self._last[1])
            self._last = (x, y)
            self.odom = (now, x, y, yaw)
            if self.home is None:
                self.home = self.odom
            self.history.append((now, x, y))
            self.history = [h for h in self.history if now - h[0] <= 1.0]

    def _on_chassis(self, message):
        with self.lock:
            self.chassis = (time.monotonic(), message.agent_connected,
                            message.motion_blocked, message.battery_low)

    def _on_ticks(self, message):
        now = time.monotonic()
        with self.lock:
            value = (message.left_ticks, message.right_ticks)
            if self.ticks is None or value != self.ticks:
                self.ticks_changed = now
            self.ticks = value

    def _on_emergency(self, message):
        with self.lock:
            self.emergency = message.data

    def snapshot(self):
        with self.lock:
            return (self.odom, self.chassis, self.emergency, self.ticks_changed, self.path,
                    self.home, list(self.history))

    def speed(self):
        """Position-derived speed over the last 0.3 s (the twist can be biased)."""
        history = self.snapshot()[6]
        if len(history) < 2:
            return math.inf
        recent = [h for h in history if history[-1][0] - h[0] <= 0.3]
        a, b = recent[0], recent[-1]
        if b[0] - a[0] < 0.15:
            return math.inf
        return math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0])

    def publish(self, linear):
        message = self.Twist()
        message.linear.x = float(linear)
        self.publisher.publish(message)

    def zero_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(0.0)
            time.sleep(PERIOD)

    def check(self, origin=None, target=None):
        """Raise Abort on any unsafe condition; return the odometry sample."""
        now = time.monotonic()
        odom, chassis, emergency, _, path, home, _ = self.snapshot()
        if emergency is not False:
            raise Abort('localization emergency stop active or unknown')
        if chassis is None or now - chassis[0] > 1.5:
            raise Abort('chassis status missing or stale')
        if not chassis[1] or chassis[2]:
            raise Abort('chassis disconnected or motion blocked')
        if odom is None or now - odom[0] > 0.3:
            raise Abort('odometry stale')
        if path > self.args.max_path:
            raise Abort(f'travel budget exceeded ({path:.2f} m)')
        if math.hypot(odom[1] - home[1], odom[2] - home[2]) > self.args.max_excursion:
            raise Abort('robot left the excursion limit around its start')
        if origin is not None:
            along, lateral = along_and_lateral(origin, odom[1], odom[2])
            if abs(lateral) > self.args.max_lateral:
                raise Abort(f'lateral offset {lateral:.3f} m beyond limit')
            if turned(origin[3], odom[3]) > math.radians(self.args.max_yaw_deg):
                raise Abort('heading changed beyond limit')
            if target is not None and abs(along) > abs(target) + OVERRUN_M:
                raise Abort(f'overrun: {along:.3f} m')
        return odom

    def wait_stationary(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            odom = self.check()
            self.publish(0.0)
            if self.speed() < 0.01:
                return odom
            time.sleep(PERIOD)
        raise Abort('robot did not become stationary')


def preflight(capture, node):
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        odom, chassis, emergency = capture.snapshot()[:3]
        if odom and chassis and emergency is not None:
            break
        time.sleep(0.1)
    publishers = node.count_publishers('/cmd_vel_command')
    subscribers = {info.node_name for info in
                   node.get_subscriptions_info_by_topic('/cmd_vel_command')}
    if publishers != 1 or 'cmd_vel_compensator' not in subscribers:
        raise Abort(f'/cmd_vel_command has {publishers} publishers and subscribers '
                    f'{sorted(subscribers)}; expected only this script publishing and '
                    'cmd_vel_compensator listening')
    chassis = capture.snapshot()[1]
    if chassis and chassis[3]:
        raise Abort('battery low')
    odom = capture.wait_stationary(5.0)
    print(json.dumps({'event': 'preflight_ok'}), flush=True)
    return odom


def drive_until(capture, direction, distance, timeout):
    """Command straight motion until ``distance`` along the start heading."""
    origin = capture.check()
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        odom = capture.check(origin, distance)
        if along_and_lateral(origin, odom[1], odom[2])[0] * direction >= distance:
            break
        capture.publish(direction * capture.args.speed)
        time.sleep(PERIOD)
    return origin


def run_stops(capture, args):
    for index in range(args.pairs * 2):
        direction = 1 if index % 2 == 0 else -1
        origin = drive_until(capture, direction, args.distance, args.run_timeout)
        odom_off = capture.check(origin)
        capture.zero_for(3.0)
        odom_end = capture.wait_stationary(5.0)
        along_off = along_and_lateral(origin, odom_off[1], odom_off[2])[0]
        along_end, lateral_end = along_and_lateral(origin, odom_end[1], odom_end[2])
        print(json.dumps({
            'event': 'stop', 'index': index, 'direction': direction,
            'travel_at_off_m': along_off, 'final_travel_m': along_end,
            'tail_m': along_end - along_off, 'lateral_end_m': lateral_end,
            'yaw_change_deg': math.degrees(turned(origin[3], odom_end[3])),
        }), flush=True)


def run_watchdog(capture, args):
    results = []
    for index in range(args.trials):
        direction = 1 if index % 2 == 0 else -1
        origin = capture.check()
        start = time.monotonic()
        while time.monotonic() - start < args.hold:
            capture.check(origin, args.speed * args.hold)
            capture.publish(direction * args.speed)
            time.sleep(PERIOD)
        last_command = time.monotonic()
        odom_at_cut = capture.check(origin)
        # From here on nothing is published: the chassis must stop itself.
        stopped_at = None
        while time.monotonic() - last_command < args.safety_net:
            capture.check(origin, args.speed * args.hold)
            ticks_changed = capture.snapshot()[3]
            ticks_still = (ticks_changed is not None
                           and time.monotonic() - ticks_changed >= 0.2)
            if ticks_still and capture.speed() < 0.01:
                stopped_at = ticks_changed
                break
            time.sleep(0.01)
        passed = stopped_at is not None
        if not passed:
            capture.zero_for(3.0)             # safety net
        odom_end = capture.wait_stationary(5.0)
        result = {
            'event': 'watchdog', 'index': index, 'direction': direction, 'passed': passed,
            'stop_after_last_command_s': (None if stopped_at is None
                                          else stopped_at - last_command),
            'travel_after_cut_m': along_and_lateral(origin, odom_end[1], odom_end[2])[0]
            - along_and_lateral(origin, odom_at_cut[1], odom_at_cut[2])[0],
        }
        results.append(result)
        print(json.dumps(result), flush=True)
        capture.zero_for(2.0)
    return all(item['passed'] for item in results)


def parse(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--mode', choices=['stops', 'watchdog'], required=True)
    parser.add_argument('--operator-present', action='store_true')
    parser.add_argument('--speed', type=float, default=0.08,
                        help='m/s; above the 0.05 m/s forward deadband, at most 0.10')
    parser.add_argument('--distance', type=float, default=0.40)
    parser.add_argument('--pairs', type=int, default=4)
    parser.add_argument('--run-timeout', type=float, default=12.0)
    parser.add_argument('--trials', type=int, default=4)
    parser.add_argument('--hold', type=float, default=2.0)
    parser.add_argument('--safety-net', type=float, default=1.5)
    parser.add_argument('--max-lateral', type=float, default=0.10)
    parser.add_argument('--max-yaw-deg', type=float, default=10.0)
    parser.add_argument('--max-excursion', type=float, default=0.80)
    parser.add_argument('--max-path', type=float, default=5.0)
    args = parser.parse_args(argv)
    check_envelope(args)
    return args


def main():
    args = parse()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init()
    node = rclpy.create_node('carbot_linear_profile_capture')
    capture = Capture(node, args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    status = 0
    try:
        preflight(capture, node)
        if args.mode == 'stops':
            run_stops(capture, args)
        elif not run_watchdog(capture, args):
            status = 3
    except Abort as error:
        print(json.dumps({'event': 'abort', 'reason': str(error)}), flush=True)
        status = 2
    except KeyboardInterrupt:
        print(json.dumps({'event': 'abort', 'reason': 'interrupted'}), flush=True)
        status = 2
    finally:
        capture.zero_for(3.0 if status else 1.0)
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({'event': 'done', 'status': status}), flush=True)
    return status


if __name__ == '__main__':
    sys.exit(main())
