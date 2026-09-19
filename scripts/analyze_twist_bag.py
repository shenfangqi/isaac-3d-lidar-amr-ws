#!/usr/bin/env python3
"""Summarize Twist command topics from a rosbag2 recording."""

import argparse
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument(
        '--topics', nargs='+',
        default=['/cmd_vel_nav', '/cmd_vel_diagnostic'])
    return parser.parse_args()


def main():
    args = parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = set(args.topics)
    missing = selected - set(topic_types)
    if missing:
        raise SystemExit(f'missing topics: {sorted(missing)}')

    message_classes = {
        topic: get_message(topic_types[topic]) for topic in selected
    }
    stats = defaultdict(lambda: {
        'count': 0,
        'nonzero': 0,
        'first_ns': None,
        'last_ns': None,
        'min_linear': float('inf'),
        'max_linear': float('-inf'),
        'min_angular': float('inf'),
        'max_angular': float('-inf'),
    })

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in selected:
            continue
        message = deserialize_message(data, message_classes[topic])
        linear = message.linear.x
        angular = message.angular.z
        item = stats[topic]
        item['count'] += 1
        if abs(linear) > 1e-6 or abs(angular) > 1e-6:
            item['nonzero'] += 1
        if item['first_ns'] is None:
            item['first_ns'] = timestamp
        item['last_ns'] = timestamp
        item['min_linear'] = min(item['min_linear'], linear)
        item['max_linear'] = max(item['max_linear'], linear)
        item['min_angular'] = min(item['min_angular'], angular)
        item['max_angular'] = max(item['max_angular'], angular)

    for topic in args.topics:
        item = stats[topic]
        duration = 0.0
        if item['first_ns'] is not None:
            duration = (item['last_ns'] - item['first_ns']) / 1e9
        print(
            f'{topic}: count={item["count"]} nonzero={item["nonzero"]} '
            f'duration={duration:.3f}s '
            f'linear=[{item["min_linear"]:.6f},'
            f'{item["max_linear"]:.6f}] '
            f'angular=[{item["min_angular"]:.6f},'
            f'{item["max_angular"]:.6f}]'
        )


if __name__ == '__main__':
    main()
