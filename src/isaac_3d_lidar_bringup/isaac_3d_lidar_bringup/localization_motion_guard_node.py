"""
ROS shell around MotionGuardCore (Issue #13 PR3).

The 50 ms control timer, requests, odometry and the emergency flag share one
callback group; sweep evaluation runs in its own group so a slow scan never
delays the control cycle.  The velocity publisher exists only while motion
is permitted and a STOP handshake has opened a session; RELEASE destroys it.
"""

import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

try:  # Not every container builds carbot_msgs; without it, never move.
    from carbot_msgs.msg import CarbotStatus
except ImportError:
    CarbotStatus = None

from .localization_contracts import (
    ContractError,
    decode_motion_profile,
    decode_motion_request,
    encode_motion_status,
    GuardState,
    MotionPolicy,
    profile_permits_motion,
    RotationAttestation,
    SE2,
)
from .localization_motion_guard import (
    GuardConfig,
    MotionGuardCore,
    MotionPermission,
    OdomSample,
    profile_hash,
    SweepVerdict,
)
from .localization_rotation_policy import (
    evaluate_localization_rotation,
    evidence_from_scan,
    footprint_geometry_hash,
    ProbeBudgetLimits,
    RotationGateConfig,
)


CANONICAL_FOOTPRINT = [0.155, 0.133, 0.155, -0.133,
                       -0.130, -0.133, -0.130, 0.133]


def motion_permission(policy, profile, hashes):
    """Return the launch-time MotionPermission for this guard."""
    if MotionPolicy(policy) != MotionPolicy.GUARDED or profile is None:
        return MotionPermission(False, '')
    allowed, _reason = profile_permits_motion(profile, *hashes)
    if not allowed:
        return MotionPermission(False, profile_hash(profile))
    return MotionPermission(True, profile_hash(profile), profile.latency_s,
                            profile.stop_tail_rad, profile.center_drift_m)


def _yaw(rotation):
    return math.atan2(2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2))


class MotionGuardNode(Node):
    """Single startup-time velocity authority for segmented rotation."""

    def __init__(self, **node_options):
        # node_options (context, parameter_overrides) serve isolated tests.
        super().__init__('localization_motion_guard', **node_options)
        declare = self.declare_parameter
        declare('motion_policy', 'forbid')
        declare('motion_profile_path', '')
        declare('extrinsics_hash', '')
        declare('control_chain_hash', '')
        declare('operator_rotation_clear', False)
        declare('attestation_max_translation_m', 0.05)
        declare('attestation_max_age_sec', 240.0)
        declare('footprint_xy', CANONICAL_FOOTPRINT)
        declare('padding_m', 0.05)
        declare('request_topic', '/automatic_localization/motion_request')
        declare('status_topic', '/automatic_localization/motion_status')
        declare('cmd_vel_topic', '/cmd_vel_command')
        declare('odom_topic', '/odom')
        declare('scan_topic', '/scan')
        declare('emergency_topic', '/localization/emergency_stop')
        declare('chassis_status_topic', '/carbot/status')
        declare('odom_frame', 'odom')
        declare('base_frame', 'base_footprint')
        declare('evidence_half_extent_m', 1.0)
        declare('evidence_resolution', 0.05)
        declare('sweep_budget_sec', 0.2)
        declare('max_probe_segments', 6)
        declare('max_total_probe_yaw_rad', 2.0 * math.pi)
        declare('probe_motion_timeout_sec', 45.0)
        declare('session_timeout_sec', 240.0)
        declare('motion_request_timeout_sec', 0.30)
        declare('sensor_freshness_sec', 0.5)

        value = self._value
        flat = list(value('footprint_xy'))
        self._footprint = tuple(zip(flat[0::2], flat[1::2]))
        self._gate = RotationGateConfig(padding_m=float(value('padding_m')))
        geometry = footprint_geometry_hash(self._footprint,
                                           self._gate.padding_m)
        self._hashes = (geometry, value('extrinsics_hash'),
                        value('control_chain_hash'))
        self._profile = self._load_profile(value('motion_profile_path'))
        permission = motion_permission(value('motion_policy'),
                                       self._profile, self._hashes)
        if CarbotStatus is None and permission.permitted:
            self.get_logger().error(
                'carbot_msgs is unavailable: chassis status cannot be '
                'verified, so motion stays forbidden')
            permission = MotionPermission(False, permission.profile_hash)
        config = GuardConfig(
            request_timeout_s=float(value('motion_request_timeout_sec')),
            sensor_freshness_s=float(value('sensor_freshness_sec')))
        limits = ProbeBudgetLimits(
            max_segments=int(value('max_probe_segments')),
            max_total_abs_yaw_rad=float(value('max_total_probe_yaw_rad')),
            max_motion_time_s=float(value('probe_motion_timeout_sec')),
            max_session_s=float(value('session_timeout_sec')))
        self._core = MotionGuardCore(config, permission, limits)
        self._lock = threading.Lock()
        self._attest = bool(value('operator_rotation_clear'))
        self._attestation = None
        self._session_seen = ''
        self._odom_pose = None
        self._publisher = None
        self._released = False
        self._tick_count = 0

        control = MutuallyExclusiveCallbackGroup()
        sweep = MutuallyExclusiveCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(String, value('request_topic'),
                                 self._on_request, 10, callback_group=control)
        self.create_subscription(Odometry, value('odom_topic'), self._on_odom,
                                 qos_profile_sensor_data,
                                 callback_group=control)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, value('emergency_topic'),
                                 self._on_emergency, latched,
                                 callback_group=control)
        if CarbotStatus is not None:
            self.create_subscription(
                CarbotStatus, value('chassis_status_topic'),
                self._on_chassis, qos_profile_sensor_data,
                callback_group=control)
        self.create_subscription(LaserScan, value('scan_topic'),
                                 self._on_scan, qos_profile_sensor_data,
                                 callback_group=sweep)
        self._status = self.create_publisher(String, value('status_topic'), 1)
        self.create_timer(0.05, self._tick, callback_group=control)
        self.get_logger().info(
            f'motion guard: permitted={permission.permitted} '
            f'policy={value("motion_policy")} attestation={self._attest}')

    def _value(self, name):
        return self.get_parameter(name).value

    def _load_profile(self, path):
        if not path:
            return None
        try:
            with open(path, encoding='utf-8') as stream:
                return decode_motion_profile(stream.read())
        except (OSError, ContractError) as error:
            self.get_logger().error(f'motion profile rejected: {error}')
            return None

    def _on_request(self, message):
        try:
            request = decode_motion_request(message.data)
        except ContractError as error:
            # Ignored, not obeyed: the lease simply stops being renewed.
            self.get_logger().warning(f'invalid motion request: {error}')
            return
        with self._lock:
            self._core.on_request(request, time.monotonic())

    def _on_odom(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        stamp = message.header.stamp
        sample = OdomSample(
            stamp.sec * 1_000_000_000 + stamp.nanosec,
            pose.position.x, pose.position.y, _yaw(pose.orientation),
            math.hypot(twist.linear.x, twist.linear.y), twist.angular.z,
            time.monotonic())
        with self._lock:
            self._odom_pose = SE2(sample.x, sample.y, sample.yaw)
            self._core.on_odom(sample)

    def _on_chassis(self, message):
        with self._lock:
            self._core.on_chassis(message.agent_connected,
                                  message.motion_blocked, time.monotonic())

    def _on_emergency(self, message):
        with self._lock:
            self._core.on_emergency(message.data)

    def _on_scan(self, scan):
        with self._lock:
            angle = self._core.sweep_query()
            attestation = self._attestation
            session = self._core.session
        if angle is None:
            return
        received = time.monotonic()
        try:
            sensor = self._lookup(scan.header.frame_id, scan.header.stamp)
            base = self._lookup(self._value('base_frame'), scan.header.stamp)
            evidence = evidence_from_scan(
                scan, sensor, (base.x, base.y),
                half_extent_m=float(self._value('evidence_half_extent_m')),
                resolution=float(self._value('evidence_resolution')),
                frame_id=self._value('odom_frame'), received_mono=received,
                deadline=received + float(self._value('sweep_budget_sec')))
            decision = evaluate_localization_rotation(
                evidence, self._footprint, base, angle,
                profile=self._profile, hashes=self._hashes, config=self._gate,
                attestation=attestation, session=session,
                now_mono=received).decision
        except (TransformException, ContractError) as error:
            # No verdict: the core stops once the last one goes stale.
            self.get_logger().warning(f'sweep not evaluated: {error}',
                                      throttle_duration_sec=2.0)
            return
        with self._lock:
            self._core.on_sweep(SweepVerdict(
                angle, decision.allowed, decision.reason, received))

    def _lookup(self, source, stamp):
        transform = self._tf_buffer.lookup_transform(
            self._value('odom_frame'), source, Time.from_msg(stamp)).transform
        return SE2(transform.translation.x, transform.translation.y,
                   _yaw(transform.rotation))

    def _tick(self):
        try:
            self._control_cycle()
        except Exception:  # Zero first, then surface the fault loudly.
            self.stop_output()
            self.get_logger().fatal('motion guard cycle failed; output zeroed')
            raise

    def _control_cycle(self):
        now = time.monotonic()
        with self._lock:
            command = self._core.tick(now)
            state = self._core.state
            session = self._core.session
            status = self._core.status(now)
            if session and session != self._session_seen:
                self._session_seen = session
                self._attestation = (
                    RotationAttestation(
                        session, self._odom_pose, now,
                        float(self._value('attestation_max_translation_m')),
                        float(self._value('attestation_max_age_sec')))
                    if self._attest and self._odom_pose is not None
                    else None)
        permitted = self._core.permission.permitted
        if state == GuardState.RELEASED:
            self._release()
        elif permitted and session and self._publisher is None \
                and not self._released:
            self._publisher = self.create_publisher(
                Twist, self._value('cmd_vel_topic'), 10)
        if self._publisher is not None:
            message = Twist()
            message.angular.z = float(command)
            self._publisher.publish(message)
        self._tick_count += 1
        if self._tick_count % 2 == 0:
            self._status.publish(String(data=encode_motion_status(status)))

    def _release(self):
        if self._publisher is not None:
            self._publisher.publish(Twist())
            self.destroy_publisher(self._publisher)
            self._publisher = None
        self._released = True

    def stop_output(self):
        """Publish one zero command before the process exits."""
        if self._publisher is not None:
            self._publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MotionGuardNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_output()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
