#!/usr/bin/env python3
"""Collect Issue #9 runtime evidence using subscriptions and read-only services.

Print one JSON document to stdout. No goal, lifecycle change, velocity command,
or parameter write is sent. Run in the target's already configured ROS shell.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException, TransformListener


TOPICS = (
    '/local_costmap/costmap_raw', '/local_costmap/published_footprint',
    '/map', '/scan', '/odom', '/automatic_localization/status',
    '/carbot/status', '/cmd_vel_nav', '/cmd_vel_command', '/cmd_vel',
    '/carbot_nav_recovery/runtime', '/carbot_nav_recovery/controller_failure',
    '/carbot_nav_recovery/costmap_evidence', '/carbot_nav_recovery/runtime_status',
)
PARAMETERS = {
    '/controller_server': ['controller_frequency', 'controller_plugins',
                           'FollowPath.plugin', 'FollowPath.allow_reversing',
                           'FollowPath.use_collision_detection'],
    '/local_costmap/local_costmap': [
        'footprint', 'footprint_padding', 'global_frame', 'robot_base_frame',
        'plugins', 'resolution', 'update_frequency', 'publish_frequency',
        'obstacle_layer.scan.topic', 'obstacle_layer.scan.observation_persistence',
        'obstacle_layer.observation_sources'],
    '/behavior_server': ['behavior_plugins', 'global_frame', 'robot_base_frame',
                         'cycle_frequency', 'min_rotational_vel',
                         'max_rotational_vel'],
    '/velocity_smoother': ['max_velocity', 'min_velocity', 'max_decel',
                           'max_accel', 'velocity_timeout', 'feedback'],
    '/bt_navigator': ['default_nav_to_pose_bt_xml', 'global_frame'],
}


def serializable(value):
    if isinstance(value, dict):
        return {key: serializable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [serializable(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


class Audit(Node):
    def __init__(self):
        super().__init__('carbot_recovery_read_only_audit')
        self.samples = {}
        self.counts = {}
        self.types = {}
        self.subscriptions_held = []
        self.param_clients = {}
        self.param_futures = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def discover(self):
        offered = dict(self.get_topic_names_and_types())
        for topic in TOPICS:
            if topic in self.types or len(offered.get(topic, [])) != 1:
                continue
            type_name = offered[topic][0]
            try:
                msg_type = get_message(type_name)
            except (ImportError, AttributeError):
                continue
            self.types[topic] = type_name
            qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=(DurabilityPolicy.TRANSIENT_LOCAL
                            if topic == '/map'
                            else DurabilityPolicy.VOLATILE))
            self.subscriptions_held.append(self.create_subscription(
                msg_type, topic,
                lambda msg, topic=topic: self.capture(topic, msg), qos))
        for name, parameters in PARAMETERS.items():
            if name in self.param_futures:
                continue
            if name not in self.param_clients:
                self.param_clients[name] = self.create_client(
                    GetParameters, name + '/get_parameters')
            client = self.param_clients[name]
            if client.service_is_ready():
                request = GetParameters.Request()
                request.names = parameters
                self.param_futures[name] = client.call_async(request)

    def capture(self, topic, message):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.samples[topic] = {
            'received_ros_sec': self.get_clock().now().nanoseconds * 1e-9,
            'received_monotonic_sec': time.monotonic(),
            'message': message_to_ordereddict(message),
        }

    def result(self):
        report = {
            'schema': 'carbot_recovery_runtime_audit_v1',
            'read_only': True, 'motion_authorized': False,
            'captured_ros_sec': self.get_clock().now().nanoseconds * 1e-9,
            'captured_monotonic_sec': time.monotonic(),
            'samples': self.samples, 'counts': self.counts,
            'types': self.types, 'parameters': {}, 'publishers': {},
        }
        for node, future in self.param_futures.items():
            if future.done() and future.result() is not None:
                report['parameters'][node] = dict(zip(
                    PARAMETERS[node], [message_to_ordereddict(value)
                                       for value in future.result().values]))
        for topic in TOPICS:
            report['publishers'][topic] = [
                {'name': item.node_name, 'namespace': item.node_namespace,
                 'type': item.topic_type}
                for item in self.get_publishers_info_by_topic(topic)]
        costmap = self.samples.get('/local_costmap/costmap_raw')
        if costmap:
            header = costmap['message']['header']
            stamp = header['stamp']
            try:
                transform = self.tf.lookup_transform(
                    header['frame_id'], 'base_footprint',
                    Time(seconds=stamp['sec'], nanoseconds=stamp['nanosec']))
                report['costmap_pose_tf'] = message_to_ordereddict(transform)
            except TransformException as error:
                report['tf_error'] = str(error)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--workspace', type=Path)
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 60.0:
        parser.error('duration must be between 1 and 60 seconds')
    rclpy.init()
    node = Audit()
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            node.discover()
            rclpy.spin_once(node, timeout_sec=0.05)
        result = node.result()
        if args.workspace:
            result['file_sha256'] = {}
            relative = 'isaac_3d_lidar_bringup'
            for path in (
                    args.workspace / 'src' / relative / 'launch'
                    / 'carbot_navigation_real.launch.py',
                    args.workspace / 'install' / relative / 'share' / relative
                    / 'config/nav2/carbot_navigation_real.yaml'):
                if path.is_file():
                    result['file_sha256'][str(path)] = hashlib.sha256(
                        path.read_bytes()).hexdigest()
        print(json.dumps(serializable(result), allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
