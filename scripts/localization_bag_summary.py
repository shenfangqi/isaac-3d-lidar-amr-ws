#!/usr/bin/env python3
"""Summarize timing, stopped-state, and localization evidence in a ROS bag."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def percentile(values, fraction):
    """Return a nearest-rank percentile for a nonempty sorted sequence."""
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
    return ordered[index]


def pose_dict(message, received):
    """Return the planar pose and useful covariance fields."""
    pose = message.pose.pose
    covariance = message.pose.covariance
    return {
        'received_ns': received,
        'source_stamp_ns': (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        ),
        'x': pose.position.x,
        'y': pose.position.y,
        'yaw': 2.0 * math.atan2(
            pose.orientation.z, pose.orientation.w),
        'xy_std': math.sqrt(max(covariance[0], covariance[7], 0.0)),
        'yaw_std': math.sqrt(max(covariance[35], 0.0)),
    }


def state_intervals(statuses, bag_start_ns):
    """Compress consecutive manager states into a readable timeline."""
    intervals = []
    for status in statuses:
        state = status.get('state', '')
        if not intervals or intervals[-1]['state'] != state:
            intervals.append({
                'state': state,
                'start_sec': (status['received_ns'] - bag_start_ns) / 1e9,
                'end_sec': (status['received_ns'] - bag_start_ns) / 1e9,
                'samples': 1,
            })
        else:
            intervals[-1]['end_sec'] = (
                status['received_ns'] - bag_start_ns) / 1e9
            intervals[-1]['samples'] += 1
    for interval in intervals:
        interval['duration_sec'] = interval['end_sec'] - interval['start_sec']
    return intervals


def verification_summary(statuses, odom, linear_limit, angular_limit):
    """Summarize one automatic or manual verification state."""
    if not statuses:
        return {
            'status_samples': 0,
            'duration_sec': None,
            'quality_failures': {},
            'maximum_quality_hold_sec': None,
            'scan_score': None,
            'candidate_start': None,
            'candidate_end': None,
            'candidate_shift_m': None,
            'candidate_shift_yaw_rad': None,
            'odom_samples': 0,
            'odom_over_limit_samples': 0,
            'odom_over_limit': [],
        }
    quality = defaultdict(int)
    for status in statuses:
        quality[status.get('quality_failure', '')] += 1
    scores = [
        status.get('scan_map_score')
        for status in statuses
        if isinstance(status.get('scan_map_score'), (int, float))
        and math.isfinite(status['scan_map_score'])
    ]
    scores_by_failure = defaultdict(list)
    for status in statuses:
        score = status.get('scan_map_score')
        if isinstance(score, (int, float)) and math.isfinite(score):
            scores_by_failure[status.get('quality_failure', '')].append(score)
    quality_intervals = []
    for status in statuses:
        failure = status.get('quality_failure', '')
        if not quality_intervals or quality_intervals[-1]['failure'] != failure:
            quality_intervals.append({
                'failure': failure,
                'start_sec': (status['received_ns']
                              - statuses[0]['received_ns']) / 1e9,
                'end_sec': (status['received_ns']
                            - statuses[0]['received_ns']) / 1e9,
                'samples': 1,
            })
        else:
            quality_intervals[-1]['end_sec'] = (
                status['received_ns'] - statuses[0]['received_ns']) / 1e9
            quality_intervals[-1]['samples'] += 1
    for interval in quality_intervals:
        interval['duration_sec'] = interval['end_sec'] - interval['start_sec']
    start_ns = statuses[0]['received_ns']
    end_ns = statuses[-1]['received_ns']
    window_odom = [
        entry for entry in odom
        if start_ns <= entry['received_ns'] <= end_ns
    ]
    over_limit = [
        entry for entry in window_odom
        if entry['linear'] > linear_limit or entry['angular'] > angular_limit
    ]
    candidates = [
        status['candidate_pose'] for status in statuses
        if isinstance(status.get('candidate_pose'), dict)
        and all(key in status['candidate_pose'] for key in ('x', 'y', 'yaw'))
    ]
    candidate_start = candidates[0] if candidates else None
    candidate_end = candidates[-1] if candidates else None
    return {
        'status_samples': len(statuses),
        'duration_sec': (end_ns - start_ns) / 1e9,
        'quality_failures': dict(quality),
        'maximum_quality_hold_sec': max(
            (status.get('quality_hold_age_sec') or 0.0
             for status in statuses), default=None),
        'scan_score': ({
            'samples': len(scores),
            'minimum': min(scores),
            'mean': sum(scores) / len(scores),
            'maximum': max(scores),
        } if scores else None),
        'scan_score_by_quality_failure': {
            failure: {
                'samples': len(values),
                'minimum': min(values),
                'mean': sum(values) / len(values),
                'maximum': max(values),
            }
            for failure, values in scores_by_failure.items()
        },
        'quality_intervals': quality_intervals,
        'candidate_start': candidate_start,
        'candidate_end': candidate_end,
        'candidate_shift_m': (
            math.hypot(candidate_end['x'] - candidate_start['x'],
                       candidate_end['y'] - candidate_start['y'])
            if candidate_start and candidate_end else None
        ),
        'candidate_shift_yaw_rad': (
            math.atan2(
                math.sin(candidate_end['yaw'] - candidate_start['yaw']),
                math.cos(candidate_end['yaw'] - candidate_start['yaw']),
            ) if candidate_start and candidate_end else None
        ),
        'odom_samples': len(window_odom),
        'odom_over_limit_samples': len(over_limit),
        'odom_over_limit': over_limit,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--linear-limit', type=float, default=0.02)
    parser.add_argument('--angular-limit', type=float, default=0.03)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=''),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {
        topic.name: get_message(topic.type)
        for topic in reader.get_all_topics_and_types()
    }
    counts = defaultdict(int)
    receive_stamps = defaultdict(list)
    odom = []
    commands = []
    statuses = []
    initial_poses = []
    amcl_poses = []

    while reader.has_next():
        topic, data, received = reader.read_next()
        counts[topic] += 1
        receive_stamps[topic].append(received)
        if topic not in {
            '/odom', '/cmd_vel', '/cmd_vel_command',
            '/automatic_localization/status', '/initialpose', '/amcl_pose',
        }:
            continue
        message = deserialize_message(data, types[topic])
        if topic == '/odom':
            twist = message.twist.twist
            odom.append({
                'received_ns': received,
                'linear': math.hypot(twist.linear.x, twist.linear.y),
                'angular': abs(twist.angular.z),
            })
        elif topic in {'/cmd_vel', '/cmd_vel_command'}:
            commands.append({
                'topic': topic,
                'received_ns': received,
                'linear': math.hypot(message.linear.x, message.linear.y),
                'angular': abs(message.angular.z),
            })
        elif topic == '/automatic_localization/status':
            try:
                status = json.loads(message.data)
            except (TypeError, ValueError):
                continue
            status['received_ns'] = received
            statuses.append(status)
        elif topic == '/initialpose':
            initial_poses.append(pose_dict(message, received))
        else:
            amcl_poses.append(pose_dict(message, received))

    bag_start_ns = min((stamps[0] for stamps in receive_stamps.values()),
                       default=0)
    topic_timing = {}
    for topic, stamps in receive_stamps.items():
        gap_rows = [
            {
                'gap_sec': (end - start) / 1e9,
                'start_offset_sec': (start - bag_start_ns) / 1e9,
                'end_offset_sec': (end - bag_start_ns) / 1e9,
            }
            for start, end in zip(stamps, stamps[1:])
        ]
        gaps = [row['gap_sec'] for row in gap_rows]
        topic_timing[topic] = {
            'count': len(stamps),
            'max_receive_gap_sec': max(gaps) if gaps else None,
            'p99_receive_gap_sec': percentile(gaps, 0.99) if gaps else None,
            'largest_receive_gaps': sorted(
                gap_rows, key=lambda row: row['gap_sec'], reverse=True)[:5],
        }

    over_limit = [
        entry for entry in odom
        if entry['linear'] > args.linear_limit
        or entry['angular'] > args.angular_limit
    ]
    nonzero_commands = [
        entry for entry in commands
        if entry['linear'] > 1e-6 or entry['angular'] > 1e-6
    ]
    state_counts = defaultdict(int)
    failure_counts = defaultdict(int)
    quality_counts = defaultdict(int)
    for status in statuses:
        state_counts[status.get('state', '')] += 1
        failure_counts[status.get('failure_reason', '')] += 1
        quality_counts[status.get('quality_failure', '')] += 1

    initial_received = initial_poses[-1]['received_ns'] if initial_poses else None
    verification_statuses = [
        status for status in statuses
        if initial_received is not None
        and status['received_ns'] >= initial_received
        and status.get('state') == 'VERIFY_MANUAL_POSE'
    ]
    verification_odom = [
        entry for entry in odom
        if initial_received is not None
        and entry['received_ns'] >= initial_received
        and (not verification_statuses
             or entry['received_ns'] <= verification_statuses[-1]['received_ns'])
    ]
    verification_over_limit = [
        entry for entry in verification_odom
        if entry['linear'] > args.linear_limit
        or entry['angular'] > args.angular_limit
    ]
    verification_quality = defaultdict(int)
    for status in verification_statuses:
        verification_quality[status.get('quality_failure', '')] += 1
    automatic_statuses = [
        status for status in statuses
        if status.get('state') == 'STOP_AND_VERIFY'
    ]

    result = {
        'bag': str(args.bag.resolve()),
        'topic_timing': topic_timing,
        'initial_poses': initial_poses,
        'amcl_poses': amcl_poses,
        'odom': {
            'samples': len(odom),
            'max_linear_mps': max((x['linear'] for x in odom), default=None),
            'max_angular_rps': max((x['angular'] for x in odom), default=None),
            'over_limit_samples': len(over_limit),
            'over_limit_fraction': len(over_limit) / len(odom) if odom else None,
            'first_over_limit': over_limit[:10],
        },
        'commands': {
            'samples': len(commands),
            'nonzero_samples': len(nonzero_commands),
            'first_nonzero': nonzero_commands[:10],
        },
        'status': {
            'samples': len(statuses),
            'states': dict(state_counts),
            'failure_reasons': dict(failure_counts),
            'quality_failures': dict(quality_counts),
            'state_intervals': state_intervals(statuses, bag_start_ns),
            'final': statuses[-1] if statuses else None,
        },
        'automatic_verification': verification_summary(
            automatic_statuses, odom, args.linear_limit, args.angular_limit),
        'manual_verification': {
            'status_samples': len(verification_statuses),
            'duration_sec': (
                (verification_statuses[-1]['received_ns']
                 - verification_statuses[0]['received_ns']) / 1e9
                if len(verification_statuses) > 1 else None
            ),
            'quality_failures': dict(verification_quality),
            'maximum_quality_hold_sec': max(
                (status.get('quality_hold_age_sec') or 0.0
                 for status in verification_statuses), default=None),
            'odom_samples': len(verification_odom),
            'odom_over_limit_samples': len(verification_over_limit),
            'odom_over_limit': verification_over_limit,
        },
    }
    encoded = json.dumps(result, indent=2, sort_keys=True, allow_nan=False)
    if args.output:
        with args.output.open('x') as stream:
            stream.write(encoded + '\n')
        print(args.output)
    else:
        print(encoded)


if __name__ == '__main__':
    main()
