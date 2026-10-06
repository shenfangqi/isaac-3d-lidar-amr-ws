#!/usr/bin/env python3
"""
Estimate an Issue #13 rotation motion profile from recorded bags.

Reads in-place rotation commands and odometry, measures stop latency, stop
tail and centre drift per stop, and writes an ESTIMATED (or INSUFFICIENT)
MotionProfile plus a Markdown report.  It never writes REVIEWED/ACCEPTED:
acceptance needs an external cross-check and an explicit human decision.
Commands and odometry are both timed by bag receive time.
"""

import argparse
import math
from pathlib import Path

import yaml

from isaac_3d_lidar_bringup.localization_contracts import (
    encode_motion_profile,
)
from isaac_3d_lidar_bringup.localization_profile_analysis import (
    analyze_segment,
    StopSample,
    build_profile,
    command_segments,
    OdomPoint,
    section_hash,
    summarize,
    TwistSample,
)
from isaac_3d_lidar_bringup.localization_rotation_policy import (
    footprint_geometry_hash,
)


def read_bag(path, command_topic, odom_topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if command_topic is None:
        command_topic = next((t for t in ('/cmd_vel_command', '/cmd_vel')
                              if t in types), None)
    if command_topic not in types or odom_topic not in types:
        return None, None, command_topic
    classes = {topic: get_message(types[topic])
               for topic in (command_topic, odom_topic)}
    commands, odometry = [], []
    while reader.has_next():
        topic, data, received = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(data, classes[topic])
        t = received / 1e9
        if topic == command_topic:
            commands.append(TwistSample(t, message.linear.x,
                                        message.angular.z))
        else:
            q = message.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
            odometry.append(OdomPoint(
                t, message.pose.pose.position.x,
                message.pose.pose.position.y, yaw,
                message.twist.twist.angular.z))
    return commands, odometry, command_topic


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--command-topic', default=None)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--parameters', type=Path, default=Path(
        'src/carbot_description/config/carbot_parameters.yaml'))
    parser.add_argument('--padding', type=float, default=0.05)
    parser.add_argument('--probe-speed', type=float, default=0.40,
                        help='only commands within 10%% of this rate count')
    parser.add_argument('--min-per-direction', type=int, default=3)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--output-md', type=Path, required=True)
    args = parser.parse_args()

    parameters = yaml.safe_load(args.parameters.read_text(encoding='utf-8'))
    footprint = tuple(tuple(point) for point in
                      parameters['geometry']['footprint_m'])
    hashes = (footprint_geometry_hash(footprint, args.padding),
              section_hash(parameters['sensors']['mid360']),
              section_hash(parameters['control']))

    samples, rows, used = [], [], []
    for bag in args.bags:
        commands, odometry, topic = read_bag(bag, args.command_topic,
                                             args.odom_topic)
        if commands is None:
            rows.append(f'| `{bag}` | - | missing {topic} or '
                        f'{args.odom_topic} | | | | |')
            continue
        contributed = False
        for segment in command_segments(commands):
            if abs(abs(segment[2]) - args.probe_speed) > 0.1 * args.probe_speed:
                # A profile is specific to the probe speed.
                sample = StopSample(1 if segment[2] > 0 else -1, segment[2],
                                    False, 'other speed')
            else:
                sample = analyze_segment(segment, odometry)
            samples.append(sample)
            contributed = contributed or sample.valid
            fmt = (lambda v, d=3: '' if v is None else f'{v:.{d}f}')
            rows.append(
                f'| `{bag.name}` | {segment[2]:+.2f} | '
                f'{"ok" if sample.valid else sample.note} | '
                f'{fmt(sample.start_latency_s)} | '
                f'{fmt(sample.stop_latency_s)} | '
                f'{fmt(sample.stop_tail_rad)} | '
                f'{fmt(sample.center_drift_m)} |')
        if contributed:
            used.append(str(bag))

    summary = summarize(samples, args.min_per_direction)
    profile = build_profile(summary, *hashes, used)
    args.output_json.write_text(encode_motion_profile(profile) + '\n',
                                encoding='utf-8')

    def stat(name, unit):
        value = summary[name]
        if value is None:
            return f'- {name}: no valid sample'
        return (f'- {name}: max {value["max"]:.3f} {unit}, median '
                f'{value["median"]:.3f} {unit} (n={value["n"]})')

    report = [
        '# Issue #13 rotation motion profile estimate', '',
        f'Status: **{profile.status.value}** (never REVIEWED/ACCEPTED '
        'automatically). Odometry-only; no external reference.', '',
        f'- geometry_hash `{hashes[0]}` (footprint + padding '
        f'{args.padding} m)',
        f'- extrinsics_hash `{hashes[1]}` (sensors.mid360)',
        f'- control_chain_hash `{hashes[2]}` (control)',
        f'- valid stops: {summary["valid"]}/{summary["samples"]} '
        f'(left {summary["valid_left"]}, right {summary["valid_right"]}; '
        f'need {args.min_per_direction} each)',
        f'- invalid reasons: {", ".join(summary["invalid_notes"]) or "none"}',
        stat('start_latency_s', 's'), stat('stop_latency_s', 's'),
        stat('stop_tail_rad', 'rad'), stat('center_drift_m', 'm'),
        stat('rate_at_stop', 'rad/s'), '',
        '| bag | command rad/s | result | start latency s | stop latency s '
        '| stop tail rad | centre drift m |',
        '| --- | --- | --- | --- | --- | --- | --- |',
        *rows, '',
    ]
    args.output_md.write_text('\n'.join(report), encoding='utf-8')
    print(f'{profile.status.value}: {summary["valid"]} valid stops; '
          f'wrote {args.output_json} and {args.output_md}')


if __name__ == '__main__':
    main()
