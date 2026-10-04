#!/usr/bin/env python3
"""One supervised, bounded pulse through the complete navigation velocity chain.

Run on the robot, not across a workstation timer. Requires explicit operator
authorization. JSONL is telemetry evidence, not an independent physical truth
measurement or an automatic recovery acceptance certificate.
"""
import argparse
from collections import deque
import json
import math
import signal
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from carbot_msgs.msg import CarbotStatus, WheelTicks
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def emit(kind, **data):
    print(json.dumps(dict(kind=kind, monotonic=time.monotonic(), **data)), flush=True)


def pump(node, seconds=.05):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=max(0., deadline-time.monotonic()))


def interrupted(*_):
    raise KeyboardInterrupt()


class Pulse(Node):
    def __init__(self):
        super().__init__('carbot_bounded_ground_pulse')
        self.latest = {}
        self.scans = deque(maxlen=20)
        self.active = set()
        self.estop_seen = False
        self.estop = True
        self.fault = ''
        self.anchor = None
        self.pose = None
        self.last_pose = None
        self.travel = 0.0
        self.turn = 0.0
        self.phase = 'preflight'
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, typ in (
            ('/odom', Odometry), ('/scan', LaserScan),
            ('/carbot/status', CarbotStatus), ('/wheel_ticks', WheelTicks),
            ('/automatic_localization/status', String),
            ('/cmd_vel_command', Twist), ('/cmd_vel', Twist),
            ('/local_costmap/costmap_raw', Costmap),
        ):
            self.create_subscription(typ, topic, lambda msg, t=topic: self.capture(t, msg), qos)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/localization/emergency_stop', self.stop, retained)
        for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path',
                       'spin', 'backup', 'wait', 'follow_waypoints', 'bounded_recovery'):
            self.create_subscription(GoalStatusArray, '/' + action + '/_action/status',
                                     lambda msg, a=action: self.goals(a, msg), retained)

    def stop(self, msg):
        self.estop_seen = True
        self.estop = msg.data
        if msg.data:
            self.fault = 'localization emergency stop'

    def goals(self, action, msg):
        if any(s.status in (1, 2, 3) for s in msg.status_list):
            self.active.add(action)
            self.fault = 'active action: ' + action
        else:
            self.active.discard(action)

    def capture(self, topic, msg):
        self.latest[topic] = (time.monotonic(), msg)
        if topic == '/local_costmap/costmap_raw':
            emit('costmap', stamp_sec=msg.header.stamp.sec,
                 stamp_nanosec=msg.header.stamp.nanosec, frame=msg.header.frame_id)
        elif topic != '/scan':
            emit('sample', phase=self.phase, topic=topic, message=message_to_ordereddict(msg))
        else:
            self.scans.append((time.monotonic(), msg))
            valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            emit('scan', count=len(valid), minimum=min(valid, default=None),
                 stamp_sec=msg.header.stamp.sec,
                 stamp_nanosec=msg.header.stamp.nanosec)
        if topic == '/odom':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.pose = (p.x, p.y, yaw)
            if not all(math.isfinite(v) for v in self.pose):
                self.fault = 'nonfinite odometry'
                return
            if self.anchor is not None and self.last_pose is not None:
                step = math.hypot(p.x-self.last_pose[0], p.y-self.last_pose[1])
                angle = abs(math.remainder(yaw-self.last_pose[2], 2*math.pi))
                self.travel += step
                self.turn += angle
                if step > .03 or angle > .10:
                    self.fault = 'odometry discontinuity'
            self.last_pose = self.pose

    def fresh(self, topic, age=.5):
        stamp, msg = self.latest.get(topic, (0, None))
        if time.monotonic()-stamp > age:
            raise RuntimeError('missing/stale ' + topic)
        if hasattr(msg, 'header') and topic in ('/scan', '/odom'):
            source = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            source_age = self.get_clock().now().nanoseconds/1e9-source
            if not -.1 <= source_age <= .5:
                raise RuntimeError(f'stale source stamp {topic}: age={source_age:.3f}s')
        return msg

    def guard(self):
        if self.fault or self.active or not self.estop_seen or self.estop:
            raise RuntimeError(self.fault or 'safety state unavailable')
        # Firmware status is 2 Hz (observed 0.484..0.512 s intervals).
        # Keep scan/odometry at 0.5 s; permit one status period plus jitter.
        status = self.fresh('/carbot/status', age=.75)
        if not status.agent_connected or not status.time_synchronized or status.motion_blocked or status.battery_low:
            raise RuntimeError('base not ready')
        self.fresh('/wheel_ticks')
        loc = json.loads(self.fresh('/automatic_localization/status').data)
        if not loc.get('ready') or not loc.get('navigation_activated'):
            raise RuntimeError('localization not READY')
        self.fresh('/scan')
        # READY alone does not prove the costmap can transform a scan.
        # TF and scan callbacks can be delivered a few milliseconds apart.
        # Select the newest still-fresh scan whose exact source-time transform
        # has reached this process; never substitute a current-time transform.
        grid = self.fresh('/local_costmap/costmap_raw', age=1.5)
        scan = None
        tf_deadline = time.monotonic() + .04
        while scan is None:
            now_ros = self.get_clock().now().nanoseconds / 1e9
            for received, candidate in reversed(self.scans):
                source = candidate.header.stamp.sec + candidate.header.stamp.nanosec / 1e9
                if not (0 <= time.monotonic() - received <= .5
                        and -.1 <= now_ros - source <= .5):
                    continue
                if self.tf_buffer.can_transform(
                        grid.header.frame_id, candidate.header.frame_id,
                        Time.from_msg(candidate.header.stamp)):
                    scan = candidate
                    break
            if scan is not None or time.monotonic() >= tf_deadline:
                break
            rclpy.spin_once(self, timeout_sec=min(.005, tf_deadline-time.monotonic()))
        if scan is None:
            now_ros = self.get_clock().now().nanoseconds / 1e9
            diagnostics = []
            for received, candidate in list(self.scans)[-5:]:
                stamp = Time.from_msg(candidate.header.stamp)
                debug = self.tf_buffer.can_transform(
                    grid.header.frame_id, candidate.header.frame_id, stamp,
                    return_debug_tuple=True,
                )
                diagnostics.append({
                    'receive_age_sec': round(time.monotonic() - received, 3),
                    'source_age_sec': round(
                        now_ros - stamp.nanoseconds / 1e9, 3),
                    'source_stamp_ns': stamp.nanoseconds,
                    'tf': debug,
                })
            emit('tf_guard_failure', costmap_frame=grid.header.frame_id,
                 scan_frame=(self.scans[-1][1].header.frame_id
                             if self.scans else None),
                 candidates=diagnostics)
            raise RuntimeError('scan-to-costmap source-time TF unavailable')
        sectors = [0]*4
        valid = []
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                valid.append(r)
                sectors[int(((scan.angle_min+i*scan.angle_increment) % (2*math.pi))/(math.pi/2)) % 4] += 1
        # Supervised micro-motion only: padded body circumradius <0.22 m,
        # pulse travel <=0.05 m, plus 0.13 m reserved clearance = 0.40 m.
        # The localization full-turn 0.55 m interlock is a different policy.
        # /scan has a 0.50 m blind range: this is NOT proof of near-field free
        # space. The operator's explicit ground-clearance confirmation is
        # essential; this guard must never be reused for autonomous recovery.
        if min(sectors) < 10 or not valid or min(valid) < .40:
            raise RuntimeError('insufficient open-area scan clearance/coverage: '
                               f'minimum={min(valid, default=None)}, quadrants={sectors}')
        odom = self.fresh('/odom')
        if abs(odom.twist.twist.linear.x) > .12 or abs(odom.twist.twist.angular.z) > .45:
            raise RuntimeError('measured speed exceeds pulse bounds')
        if self.travel > .08 or self.turn > .25:
            raise RuntimeError('measured travel/rotation budget exceeded')
        for topic, expected in (
            ('/cmd_vel_nav', {'controller_server', 'behavior_server', self.get_name()}),
            ('/cmd_vel_command', {'velocity_smoother'}),
            ('/cmd_vel', {'cmd_vel_compensator'}),
        ):
            pubs = self.get_publishers_info_by_topic(topic)
            if {p.node_name for p in pubs} != expected or any(p.node_namespace != '/' for p in pubs):
                raise RuntimeError('unexpected command ownership: ' + topic)
            if topic != '/cmd_vel_nav' and len(pubs) != 1:
                raise RuntimeError('duplicate command publisher: ' + topic)

    def send(self, linear=0., angular=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub.publish(msg)
        emit('command', linear=linear, angular=angular, phase=self.phase)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--supervised-ground-authorized', action='store_true')
    p.add_argument('--linear', type=float, default=0.)
    p.add_argument('--angular', type=float, default=0.)
    p.add_argument('--seconds', type=float, default=1.)
    p.add_argument('--watchdog-dropout-seconds', type=float, default=0.)
    args = p.parse_args()
    if (not args.supervised_ground_authorized or
            not all(math.isfinite(v) for v in (args.linear, args.angular, args.seconds)) or
            abs(args.linear) > .05 or abs(args.angular) > .40 or
            not 0 < args.seconds <= 1 or args.linear * args.angular != 0 or
            (args.angular and abs(args.angular) * args.seconds > .160001) or
            not 0 <= args.watchdog_dropout_seconds <= 1.5):
        p.error('requires supervision, <=0.05 m/s OR <=0.40 rad/s, '
                'duration <=1 s and commanded rotation <=0.16 rad')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Pulse()
    # SIGTERM also reaches the zero-command finally block. Firmware and the
    # smoother retain their own timeout protection if the process is killed.
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    result = 'FAILED'
    reason = ''
    started = False
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            pump(node)
        node.guard()
        o = node.fresh('/odom').twist.twist
        if abs(o.linear.x) > .015 or abs(o.angular.z) > .03:
            raise RuntimeError('not initially stopped')
        # Establish and observe zero through both downstream stages first.
        node.phase = 'zero_check'
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            node.guard()
            node.send()
            pump(node)
        for topic in ('/cmd_vel_command', '/cmd_vel'):
            cmd = node.fresh(topic)
            if abs(cmd.linear.x)+abs(cmd.angular.z) > 1e-6:
                raise RuntimeError('downstream is not zero')
        node.anchor = node.pose
        node.last_pose = node.pose
        node.phase = 'pulse'
        started = True
        emit('start', pose=node.anchor, linear=args.linear, angular=args.angular, seconds=args.seconds)
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            node.guard()
            node.send(args.linear, args.angular)
            pump(node)
        if args.watchdog_dropout_seconds:
            node.phase = 'watchdog_dropout'
            emit('watchdog_dropout_start', seconds=args.watchdog_dropout_seconds)
            deadline = time.monotonic()+args.watchdog_dropout_seconds
            while time.monotonic() < deadline:
                node.guard()
                # Intentionally publish nothing.  The node stays alive to
                # monitor odometry and both downstream stages.  finally still
                # sends explicit zeros after the bounded dropout interval.
                pump(node)
        result = 'PULSE_COMPLETE'
    except (Exception, KeyboardInterrupt) as exc:
        reason = str(exc) or 'interrupted'
    finally:
        node.phase = 'stop'
        emit('stop_requested', pose=node.pose)
        # Explicit zero; never clear a safety latch or publish below smoother.
        deadline = time.monotonic()+2
        stopped_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            node.send()
            pump(node)
            try:
                o = node.fresh('/odom').twist.twist
                cmds = [node.fresh(t) for t in ('/cmd_vel_command', '/cmd_vel')]
                stopped = (abs(o.linear.x) < .015 and abs(o.angular.z) < .03 and
                           all(abs(c.linear.x)+abs(c.angular.z) < 1e-6 for c in cmds))
                stopped_since = (stopped_since or time.monotonic()) if stopped else None
            except Exception:
                stopped_since = None
        settled = stopped_since is not None and time.monotonic()-stopped_since >= .5
        emit('result', result=result, reason=reason, started=started, settled=settled,
             anchor=node.anchor, final_pose=node.pose, path_length=node.travel,
             absolute_yaw_travel=node.turn, fault=node.fault)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result == 'PULSE_COMPLETE' and settled and not node.fault else 1


if __name__ == '__main__':
    raise SystemExit(main())
