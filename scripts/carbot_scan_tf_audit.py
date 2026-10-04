#!/usr/bin/env python3
"""Read-only source-time scan/TF audit. Never publishes or changes lifecycle."""
import argparse
from collections import Counter, deque
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('carbot_scan_tf_readonly_audit')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pending = deque(maxlen=100)
    rows = []
    edges = {}
    def tf_cb(msg):
        for t in msg.transforms:
            key = t.header.frame_id + ' -> ' + t.child_frame_id
            stamp = t.header.stamp.sec + t.header.stamp.nanosec/1e9
            entry = edges.setdefault(key, {'count': 0, 'first_stamp': stamp})
            entry.update(count=entry['count']+1, last_stamp=stamp,
                         source_age=node.get_clock().now().nanoseconds/1e9-stamp)
    def scan_cb(msg):
        pending.append((msg, time.monotonic(),
                        node.get_clock().now().nanoseconds/1e9))
    node.create_subscription(TFMessage, '/tf', tf_cb, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)
    deadline = time.monotonic()+args.duration
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        if not pending:
            continue
        msg, received, received_ros = pending[0]
        if time.monotonic()-received < .3:
            continue
        pending.popleft()
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        checks = {}
        for target in ('odom', 'map'):
            try:
                buffer.lookup_transform(target, msg.header.frame_id,
                                        Time.from_msg(msg.header.stamp))
                checks[target] = 'OK'
            except Exception as exc:
                checks[target] = str(exc)
        rows.append({'stamp': stamp, 'arrival_age': received_ros-stamp,
                     'after_wait': time.monotonic()-received, 'tf': checks})
    print(json.dumps({'read_only': True, 'edges': edges, 'scans': rows,
                      'counts': {t: dict(Counter(r['tf'][t] == 'OK' for r in rows))
                                 for t in ('odom', 'map')}}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
