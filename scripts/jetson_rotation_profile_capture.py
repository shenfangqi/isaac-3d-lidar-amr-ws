#!/usr/bin/env python3
"""
Supervised in-place rotations on the Issue #13 guard control chain.

Publishes only to /cmd_vel_command (-> compensator -> /cmd_vel -> ESP32),
the chain the motion guard uses; Nav2 must be inactive.  An operator must be
on site with the e-stop: pass --operator-present to run at all.

Modes:
  stops     alternating left/right turns of --angle-deg at --speed, each
            followed by 3 s of zero commands (motion-profile data);
  watchdog  turn for --hold seconds, then stop publishing entirely (no zero)
            and measure how long the chassis watchdog takes to stop the
            robot.  If it still moves after --safety-net seconds, zero is
            published and the trial is reported as FAIL.

Every 50 ms the script aborts (3 s of zero commands, nonzero exit) on a
localization emergency, a blocked/disconnected/stale chassis, stale
odometry, translation beyond --max-drift, overshoot beyond 30 deg past the
target, or more than 4*pi total rotation.
"""

import argparse
import json
import math
import sys
import threading
import time

from carbot_msgs.msg import CarbotStatus, WheelTicks
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool


PERIOD = 0.05


class Abort(RuntimeError):
    pass


class Capture:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.lock = threading.Lock()
        self.odom = None            # (t, x, y, yaw, linear, angular)
        self.chassis = None         # (t, connected, blocked, battery_low)
        self.emergency = None
        self.ticks = None           # (t, left, right)
        self.ticks_changed = None
        self.total_travel = 0.0
        self._last_yaw = None
        self.publisher = node.create_publisher(Twist, '/cmd_vel_command', 10)
        node.create_subscription(Odometry, '/odom', self._on_odom,
                                 qos_profile_sensor_data)
        node.create_subscription(CarbotStatus, '/carbot/status',
                                 self._on_chassis, qos_profile_sensor_data)
        node.create_subscription(WheelTicks, '/wheel_ticks', self._on_ticks,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/localization/emergency_stop',
                                 self._on_emergency, latched)

    # -- inputs -----------------------------------------------------------

    def _on_odom(self, message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        twist = message.twist.twist
        with self.lock:
            if self._last_yaw is not None:
                self.total_travel += abs(math.atan2(
                    math.sin(yaw - self._last_yaw),
                    math.cos(yaw - self._last_yaw)))
            self._last_yaw = yaw
            self.odom = (time.monotonic(), message.pose.pose.position.x,
                         message.pose.pose.position.y, yaw,
                         math.hypot(twist.linear.x, twist.linear.y),
                         twist.angular.z)

    def _on_chassis(self, message):
        with self.lock:
            self.chassis = (time.monotonic(), message.agent_connected,
                            message.motion_blocked, message.battery_low)

    def _on_ticks(self, message):
        now = time.monotonic()
        with self.lock:
            value = (message.left_ticks, message.right_ticks)
            if self.ticks is None or value != self.ticks[1:]:
                self.ticks_changed = now
            self.ticks = (now,) + value

    def _on_emergency(self, message):
        with self.lock:
            self.emergency = message.data

    # -- helpers ----------------------------------------------------------

    def snapshot(self):
        with self.lock:
            return (self.odom, self.chassis, self.emergency,
                    self.ticks_changed, self.total_travel)

    def publish(self, angular):
        message = Twist()
        message.angular.z = float(angular)
        self.publisher.publish(message)

    def zero_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.publish(0.0)
            time.sleep(PERIOD)

    def check(self, origin=None, target=None):
        """Raise Abort on any unsafe condition; return the odom sample."""
        now = time.monotonic()
        odom, chassis, emergency, _, travel = self.snapshot()
        if emergency is not False:
            raise Abort('localization emergency stop active or unknown')
        if chassis is None or now - chassis[0] > 1.5:
            raise Abort('chassis status missing or stale')
        if not chassis[1] or chassis[2]:
            raise Abort('chassis disconnected or motion blocked')
        if odom is None or now - odom[0] > 0.3:
            raise Abort('odometry stale')
        if travel > 4.0 * math.pi:
            raise Abort('total rotation budget exceeded')
        if origin is not None:
            drift = math.hypot(odom[1] - origin[1], odom[2] - origin[2])
            if drift > self.args.max_drift:
                raise Abort(f'translation {drift:.3f} m beyond limit')
            if target is not None:
                progress = signed_progress(origin[3], odom[3])
                if abs(progress) > abs(target) + math.radians(30):
                    raise Abort(f'overshoot: {math.degrees(progress):.1f} deg')
        return odom

    def stationary(self, odom):
        return abs(odom[4]) < 0.02 and abs(odom[5]) < 0.03

    def wait_stationary(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            odom = self.check()
            self.publish(0.0)
            if self.stationary(odom):
                return odom
            time.sleep(PERIOD)
        raise Abort('robot did not become stationary')


def signed_progress(start_yaw, yaw):
    return math.atan2(math.sin(yaw - start_yaw), math.cos(yaw - start_yaw))


def preflight(capture, node):
    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        odom, chassis, emergency, _, _ = capture.snapshot()
        if odom and chassis and emergency is not None:
            break
        time.sleep(0.1)
    # Only this script may publish.  Read-only subscribers (a bag
    # recorder) cannot actuate; the compensator must be listening.
    publishers = node.count_publishers('/cmd_vel_command')
    subscribers = {info.node_name for info in
                   node.get_subscriptions_info_by_topic('/cmd_vel_command')}
    if publishers != 1 or 'cmd_vel_compensator' not in subscribers:
        raise Abort(f'/cmd_vel_command has {publishers} publishers and '
                    f'subscribers {sorted(subscribers)}; expected only this '
                    'script publishing and cmd_vel_compensator listening')
    if capture.snapshot()[1] and capture.snapshot()[1][3]:
        raise Abort('battery low')
    odom = capture.wait_stationary(5.0)
    print(json.dumps({'event': 'preflight_ok'}), flush=True)
    return odom


def rotate_until(capture, direction, target, timeout, stop_publishing=False):
    """Command rotation until the target or timeout; return start, end."""
    origin = capture.check()
    start = time.monotonic()
    rate = direction * capture.args.speed
    while time.monotonic() - start < timeout:
        odom = capture.check(origin, target)
        if abs(signed_progress(origin[3], odom[3])) >= abs(target):
            break
        capture.publish(rate)
        time.sleep(PERIOD)
    return origin, time.monotonic()


def run_stops(capture, args):
    target = math.radians(args.angle_deg)
    for index in range(args.pairs * 2):
        direction = 1 if index % 2 == 0 else -1
        origin, t_off = rotate_until(capture, direction, direction * target,
                                     args.turn_timeout)
        odom_off = capture.check(origin)
        capture.zero_for(3.0)
        odom_end = capture.wait_stationary(5.0)
        print(json.dumps({
            'event': 'stop', 'index': index, 'direction': direction,
            'progress_at_off_deg': math.degrees(
                signed_progress(origin[3], odom_off[3])),
            'final_progress_deg': math.degrees(
                signed_progress(origin[3], odom_end[3])),
            'drift_m': math.hypot(odom_end[1] - origin[1],
                                  odom_end[2] - origin[2]),
        }), flush=True)


def run_watchdog(capture, args):
    results = []
    for index in range(args.trials):
        direction = 1 if index % 2 == 0 else -1
        origin = capture.check()
        start = time.monotonic()
        while time.monotonic() - start < args.hold:
            capture.check(origin, direction * math.radians(120))
            capture.publish(direction * args.speed)
            time.sleep(PERIOD)
        last_command = time.monotonic()
        odom_at_cut = capture.check(origin)
        # From here on nothing is published: the chassis must stop itself.
        stopped_at = None
        while time.monotonic() - last_command < args.safety_net:
            odom, _, _, ticks_changed, _ = capture.snapshot()
            capture.check(origin)
            ticks_still = (ticks_changed is not None
                           and time.monotonic() - ticks_changed >= 0.2)
            if capture.stationary(odom) and ticks_still:
                stopped_at = ticks_changed
                break
            time.sleep(0.01)
        passed = stopped_at is not None
        if not passed:
            capture.zero_for(3.0)             # safety net
        odom_end = capture.wait_stationary(5.0)
        result = {
            'event': 'watchdog', 'index': index, 'direction': direction,
            'passed': passed,
            'stop_after_last_command_s': (
                None if stopped_at is None else stopped_at - last_command),
            'yaw_after_cut_deg': math.degrees(
                signed_progress(odom_at_cut[3], odom_end[3])),
            'rate_at_cut': odom_at_cut[5],
        }
        results.append(result)
        print(json.dumps(result), flush=True)
        capture.zero_for(2.0)
    return all(item['passed'] for item in results)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--mode', choices=['stops', 'watchdog'],
                        required=True)
    parser.add_argument('--operator-present', action='store_true')
    parser.add_argument('--speed', type=float, default=0.40)
    parser.add_argument('--angle-deg', type=float, default=60.0)
    parser.add_argument('--pairs', type=int, default=4)
    parser.add_argument('--turn-timeout', type=float, default=8.0)
    parser.add_argument('--trials', type=int, default=4)
    parser.add_argument('--hold', type=float, default=1.5)
    parser.add_argument('--safety-net', type=float, default=1.5)
    parser.add_argument('--max-drift', type=float, default=0.10)
    args = parser.parse_args()
    if not args.operator_present:
        raise SystemExit('refusing: pass --operator-present only with an '
                         'operator at the robot and the e-stop in reach')
    if not (0.0 < args.speed <= 0.40 and 0.0 < args.angle_deg <= 90.0
            and 1 <= args.pairs <= 6 and 1 <= args.trials <= 6
            and 0.0 < args.hold <= 3.0 and 0.5 <= args.safety_net <= 2.0):
        raise SystemExit('argument outside the supervised envelope')

    rclpy.init()
    node = rclpy.create_node('carbot_rotation_profile_capture')
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
        print(json.dumps({'event': 'abort', 'reason': str(error)}),
              flush=True)
        status = 2
    except KeyboardInterrupt:
        print(json.dumps({'event': 'abort', 'reason': 'interrupted'}),
              flush=True)
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
