#!/usr/bin/env python3
"""
Support node for replay_localization_node_chain.sh (bag replay only).

Stands in for what a stationary 3D-capture bag cannot provide to the real
automatic_localization_manager:

- the navigation lifecycle service (validation_only never calls it);
- static transforms, read from the bag's /tf_static so playback may start late;
- odom->base_footprint TF, rebuilt from /odom.  The recorded /tf has gaps
  (recorder losses); /odom carries the identical transform at the same
  stamps (2026-10-10 capture_02: 841 shared stamps, max difference 0);
- gap filling: recorded /odom and /fast_lio/imu_odom drop out for up to
  2 s, which the robot never saw.  The captures are stationary, so a gap
  longer than ODOM_HOLD_SEC is filled by re-stamping the last message.

It also records every distinct manager status (state or 3D result) as JSON
lines.  Never use it with a moving robot or on the robot itself.
"""

import json
import os
import sys

from geometry_msgs.msg import TransformStamped
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import rosbag2_py
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class ReplaySupport(Node):
    """Lifecycle stub, TF reconstruction, odometry gap filling, status log."""

    def __init__(self, bag, status_path):
        super().__init__('replay_support')
        self._status = open(status_path, 'w')
        self._last_key = None
        self.create_service(ManageLifecycleNodes,
                            '/lifecycle_manager_navigation/manage_nodes',
                            self._lifecycle)
        self.create_subscription(
            String, '/automatic_localization/status', self._on_status,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._tf = TransformBroadcaster(self)
        self._static = StaticTransformBroadcaster(self)
        self._static.sendTransform(self._bag_static_transforms(bag))
        self._hold_ns = float(os.environ.get('ODOM_HOLD_SEC', '0.1')) * 1e9
        self._outputs, self._latest, self.filled = {}, {}, 0
        for source, target in (('/rec/odom', '/odom'),
                               ('/rec/imu_odom', '/fast_lio/imu_odom')):
            self._outputs[source] = self.create_publisher(Odometry, target, 50)
            self.create_subscription(
                Odometry, source,
                lambda message, s=source: self._forward(s, message, False), 50)
        self.create_timer(0.05, self._fill)

    @staticmethod
    def _bag_static_transforms(bag):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
        transforms = []
        while reader.has_next():
            transforms += deserialize_message(reader.read_next()[1], TFMessage).transforms
        return transforms

    def _forward(self, source, message, filled):
        self._outputs[source].publish(message)
        if source == '/rec/odom':
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            position = message.pose.pose.position
            transform.transform.translation.x = position.x
            transform.transform.translation.y = position.y
            transform.transform.translation.z = position.z
            transform.transform.rotation = message.pose.pose.orientation
            self._tf.sendTransform(transform)
        if filled:
            self.filled += 1
        else:
            self._latest[source] = (message, self.get_clock().now().nanoseconds)

    def _fill(self):
        now = self.get_clock().now().nanoseconds
        for source, (message, seen) in list(self._latest.items()):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            if now - max(seen, stamp) > self._hold_ns and now - seen < 5e9:
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(now, 10**9)
                self._forward(source, message, True)

    def _lifecycle(self, _request, response):
        response.success = True
        return response

    def _on_status(self, message):
        status = json.loads(message.data)
        key = (status.get('state'),
               json.dumps(status.get('surface_recheck'), sort_keys=True))
        if key == self._last_key:
            return
        self._last_key = key
        self._status.write(message.data + '\n')
        self._status.flush()
        self.get_logger().info(
            f"STATE {status.get('state')} candidate={status.get('candidate_pose')} "
            f"failure={status.get('failure_reason')!r} "
            f"surface={status.get('surface_recheck')} filled={self.filled}")


def main():
    if len(sys.argv) != 3:
        raise SystemExit('usage: replay_localization_node_support.py BAG STATUS_JSONL')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = ReplaySupport(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
