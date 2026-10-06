"""
Issue #13 PR3 guard node: permission rules and an isolated ROS round trip.

The ROS test runs in its own DDS domain on /test_confined/* topics, never on
the robot domain or the real /cmd_vel_command.
"""

import json
import math
from pathlib import Path
import threading
import time

import pytest

from isaac_3d_lidar_bringup.localization_contracts import (
    encode_motion_profile,
    encode_motion_request,
    MotionOperation,
    MotionProfile,
    MotionRequest,
)
from isaac_3d_lidar_bringup.localization_motion_guard_node import (
    CANONICAL_FOOTPRINT,
    motion_permission,
    profile_hash,
)
from isaac_3d_lidar_bringup.localization_rotation_policy import (
    footprint_geometry_hash,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]
FOOTPRINT = tuple(zip(CANONICAL_FOOTPRINT[0::2], CANONICAL_FOOTPRINT[1::2]))
GEOMETRY = footprint_geometry_hash(FOOTPRINT, 0.05)
EXTRINSICS, CONTROL = 'a' * 64, 'b' * 64


def _profile(status='ACCEPTED', geometry=GEOMETRY):
    return MotionProfile(1, geometry, EXTRINSICS, CONTROL, ('bag-1',),
                         0.05, 0.03, 0.1, True, status)


@pytest.mark.parametrize('policy, profile, permitted', [
    ('forbid', _profile(), False),
    ('guarded', None, False),
    ('guarded', _profile('REVIEWED'), False),
    ('guarded', _profile(geometry='c' * 64), False),
    ('guarded', _profile(), True),
])
def test_motion_permission(policy, profile, permitted):
    result = motion_permission(policy, profile,
                               (GEOMETRY, EXTRINSICS, CONTROL))
    assert result.permitted is permitted
    if permitted:
        assert result.profile_hash == profile_hash(profile)
        assert (result.latency_s, result.stop_tail_rad,
                result.center_drift_m) == (0.1, 0.05, 0.03)


def test_profile_hash_is_stable_and_content_bound():
    assert profile_hash(_profile()) == profile_hash(_profile())
    assert profile_hash(_profile()) != profile_hash(_profile('REVIEWED'))


def test_velocity_publisher_is_created_only_after_permission_and_handshake():
    source = (PACKAGE_DIR / 'isaac_3d_lidar_bringup'
              / 'localization_motion_guard_node.py').read_text()
    creation = "Twist, self._value('cmd_vel_topic')"
    assert source.count(creation) == 1
    # Only the control cycle creates it, never the constructor.
    assert source.index(creation) > source.index('def _control_cycle')
    assert 'elif permitted and session and self._publisher is None' in source
    assert 'self.destroy_publisher(self._publisher)' in source


# --- isolated ROS round trip ------------------------------------------------

DOMAIN = 87
NS = '/test_confined'


def _room_ranges(count=360):
    ranges = []
    for index in range(count):
        angle = -math.pi + index * 2 * math.pi / count
        dx, dy = abs(math.cos(angle)), abs(math.sin(angle))
        ranges.append(1.2 / max(dx, dy))
    return ranges


@pytest.fixture
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=DOMAIN)
    yield rclpy, context
    rclpy.shutdown(context=context)


def test_guard_round_trip_in_isolated_domain(ros_context, tmp_path):
    rclpy, context = ros_context
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import StaticTransformBroadcaster

    import isaac_3d_lidar_bringup.localization_motion_guard_node as module

    profile_path = tmp_path / 'profile.json'
    profile_path.write_text(encode_motion_profile(_profile()))
    overrides = [
        Parameter('motion_policy', value='guarded'),
        Parameter('motion_profile_path', value=str(profile_path)),
        Parameter('extrinsics_hash', value=EXTRINSICS),
        Parameter('control_chain_hash', value=CONTROL),
        Parameter('operator_rotation_clear', value=True),
        Parameter('request_topic', value=f'{NS}/motion_request'),
        Parameter('status_topic', value=f'{NS}/motion_status'),
        Parameter('cmd_vel_topic', value=f'{NS}/cmd_vel_command'),
        Parameter('odom_topic', value=f'{NS}/odom'),
        Parameter('scan_topic', value=f'{NS}/scan'),
        Parameter('emergency_topic', value=f'{NS}/emergency_stop'),
        Parameter('chassis_status_topic', value=f'{NS}/carbot_status'),
    ]

    guard = module.MotionGuardNode(context=context,
                                   parameter_overrides=overrides)

    harness = Node('confined_harness', context=context)
    state = {'yaw': 0.0, 'rate': 0.0, 'commands': [], 'status': []}
    harness.create_subscription(
        Twist, f'{NS}/cmd_vel_command',
        lambda m: state['commands'].append(m.angular.z), 10)
    harness.create_subscription(
        String, f'{NS}/motion_status',
        lambda m: state['status'].append(json.loads(m.data)), 10)
    requests = harness.create_publisher(String, f'{NS}/motion_request', 10)
    odom = harness.create_publisher(Odometry, f'{NS}/odom',
                                    qos_profile_sensor_data)
    scan_pub = harness.create_publisher(LaserScan, f'{NS}/scan',
                                        qos_profile_sensor_data)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    harness.create_publisher(Bool, f'{NS}/emergency_stop',
                             latched).publish(Bool(data=False))
    static = StaticTransformBroadcaster(harness)
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_footprint'
    transform.transform.rotation.w = 1.0
    static.sendTransform(transform)

    from carbot_msgs.msg import CarbotStatus
    chassis = harness.create_publisher(CarbotStatus, f'{NS}/carbot_status',
                                       qos_profile_sensor_data)

    def publish_chassis():
        message = CarbotStatus()
        message.agent_connected = True
        message.motion_blocked = False
        chassis.publish(message)

    harness.create_timer(0.5, publish_chassis)
    lease = {'request': None}

    def publish_inputs():
        state['yaw'] += state['rate'] * 0.05
        now = harness.get_clock().now().to_msg()
        message = Odometry()
        message.header.stamp = now
        message.pose.pose.orientation.z = math.sin(state['yaw'] / 2)
        message.pose.pose.orientation.w = math.cos(state['yaw'] / 2)
        message.twist.twist.angular.z = state['rate']
        odom.publish(message)
        if state['commands']:
            state['rate'] = state['commands'][-1]

    def publish_scan():
        message = LaserScan()
        message.header.stamp = harness.get_clock().now().to_msg()
        message.header.frame_id = 'base_footprint'
        message.angle_min = -math.pi
        message.angle_increment = 2 * math.pi / 360
        message.range_min, message.range_max = 0.5, 8.0
        message.ranges = _room_ranges()
        scan_pub.publish(message)

    def publish_lease():
        if lease['request'] is not None:
            requests.publish(String(data=encode_motion_request(
                lease['request'])))

    harness.create_timer(0.05, publish_inputs)
    harness.create_timer(0.1, publish_scan)
    harness.create_timer(0.1, publish_lease)

    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(guard)
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        time.sleep(1.5)
        assert harness.count_publishers(f'{NS}/cmd_vel_command') == 0

        session = 'isolatedsession01'
        lease['request'] = MotionRequest(1, session, 1, MotionOperation.STOP,
                                         0.0, 0.0, '')
        time.sleep(1.5)
        assert harness.count_publishers(f'{NS}/cmd_vel_command') == 1
        assert state['status'][-1]['stopped'] is True

        lease['request'] = MotionRequest(
            1, session, 2, MotionOperation.ROTATE, math.radians(90), 0.4,
            profile_hash(_profile()))
        time.sleep(1.5)
        assert max(state['commands']) > 0.1, state['status'][-1]
        assert state['status'][-1]['state'] == 'ROTATING'

        lease['request'] = None           # manager goes silent
        time.sleep(0.30 + 0.05 + 0.3)
        assert state['commands'][-1] == 0.0
        assert state['status'][-1]['reason'] == 'CANCELED'

        time.sleep(1.0)
        lease['request'] = MotionRequest(1, session, 3,
                                         MotionOperation.RELEASE,
                                         0.0, 0.0, '')
        time.sleep(1.0)
        assert state['status'][-1]['state'] == 'RELEASED'
        assert harness.count_publishers(f'{NS}/cmd_vel_command') == 0
    finally:
        executor.shutdown()
        guard.destroy_node()
        harness.destroy_node()


def test_guard_without_carbot_msgs_never_permits_motion(ros_context,
                                                        tmp_path,
                                                        monkeypatch):
    rclpy, context = ros_context
    from rclpy.parameter import Parameter

    import isaac_3d_lidar_bringup.localization_motion_guard_node as module

    monkeypatch.setattr(module, 'CarbotStatus', None)
    profile_path = tmp_path / 'profile.json'
    profile_path.write_text(encode_motion_profile(_profile()))
    guard = module.MotionGuardNode(context=context, parameter_overrides=[
        Parameter('motion_policy', value='guarded'),
        Parameter('motion_profile_path', value=str(profile_path)),
        Parameter('extrinsics_hash', value=EXTRINSICS),
        Parameter('control_chain_hash', value=CONTROL),
        Parameter('cmd_vel_topic', value=f'{NS}/cmd_vel_command'),
    ])
    try:
        assert guard._core.permission.permitted is False
    finally:
        guard.destroy_node()
