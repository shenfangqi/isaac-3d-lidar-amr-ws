#!/usr/bin/env python3
"""
Estimate an Issue #53 linear (translation) profile from recorded bags.

Reads the straight commands (/cmd_vel_command) and /odom of bags recorded
while scripts/jetson_linear_profile_capture.py ran, analyses every run
(localization_linear_profile_analysis) and writes an ESTIMATED
LinearProfile plus a Markdown report.  With too few valid stops per
direction or watchdog trials it writes the report only and exits 1.  It
never writes REVIEWED/ACCEPTED: acceptance needs an external cross-check
and an explicit human decision (then scripts/check_linear_profile.py).
Commands and odometry are both timed by bag receive time.
"""

import argparse
import math
from pathlib import Path
import sys

import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(WORKSPACE / 'src/isaac_3d_lidar_bringup'),
                str(WORKSPACE / 'src/carbot_nav_recovery'), str(WORKSPACE / 'scripts')]

from check_motion_profile import canonical_hashes  # noqa: E402
from isaac_3d_lidar_bringup.localization_linear_profile_analysis import (  # noqa: E402
    analyze_linear, build_linear_profile, Command, LinearSample, linear_segments, Pose,
    summarize)
from isaac_3d_lidar_bringup.localization_translation_contracts import (  # noqa: E402
    encode_linear_profile)


def read_bag(path, command_topic, odom_topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if command_topic not in types or odom_topic not in types:
        return None, None
    classes = {topic: get_message(types[topic]) for topic in (command_topic, odom_topic)}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[command_topic, odom_topic]))
    commands, odometry = [], []
    while reader.has_next():
        topic, data, received = reader.read_next()
        message = deserialize_message(data, classes[topic])
        t = received / 1e9
        if topic == command_topic:
            commands.append(Command(t, message.linear.x, message.angular.z))
        else:
            q = message.pose.pose.orientation
            odometry.append(Pose(t, message.pose.pose.position.x, message.pose.pose.position.y,
                                 math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))))
    odometry.sort(key=lambda p: p.t)
    return commands, odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--command-topic', default='/cmd_vel_command')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--parameters', type=Path, default=WORKSPACE / (
        'src/carbot_description/config/carbot_parameters.yaml'))
    parser.add_argument('--padding', type=float, default=0.05)
    parser.add_argument('--probe-speed', type=float, default=0.08,
                        help='only commands within 10%% of this speed count')
    parser.add_argument('--min-per-direction', type=int, default=3)
    parser.add_argument('--min-watchdog', type=int, default=3)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--output-md', type=Path, required=True)
    args = parser.parse_args()
    for path in (args.output_json, args.output_md):
        if path.exists():
            parser.error(f'{path} exists; remove it first')

    parameters = yaml.safe_load(args.parameters.read_text(encoding='utf-8'))
    hashes = canonical_hashes(parameters, args.padding)
    samples, rows, used = [], [], []
    for bag in args.bags:
        commands, odometry = read_bag(bag, args.command_topic, args.odom_topic)
        if commands is None:
            rows.append(f'| `{bag.name}` | - | missing {args.command_topic} or '
                        f'{args.odom_topic} | | | | | | |')
            continue
        contributed = False
        for segment in linear_segments(commands):
            if abs(abs(segment.speed) - args.probe_speed) > .1 * args.probe_speed:
                # A profile is specific to the probe speed.
                sample = LinearSample(1 if segment.speed > 0 else -1, segment.speed,
                                      segment.cut, False, 'other speed')
            else:
                sample = analyze_linear(segment, odometry)
            samples.append(sample)
            contributed = contributed or sample.valid

            def fmt(value, digits=3):
                return '' if value is None else f'{value:.{digits}f}'
            yaw = (None if sample.yaw_change_rad is None
                   else math.degrees(sample.yaw_change_rad))
            rows.append(
                f'| `{bag.name}` | {segment.speed:+.3f}{" cut" if segment.cut else ""} | '
                f'{"ok" if sample.valid else sample.note} | {fmt(sample.reaction_s)} | '
                f'{fmt(sample.stop_tail_m)} | {fmt(sample.watchdog_s)} | '
                f'{fmt(sample.travel_m)} | {fmt(sample.lateral_error_m)} | '
                f'{fmt(yaw, 1)} |')
        if contributed:
            used.append(bag.name)

    summary = summarize(samples, args.min_per_direction, args.min_watchdog)
    profile = build_linear_profile(summary, args.probe_speed, hashes, used)

    def stat(name, unit):
        value = summary[name]
        if value is None:
            return f'- {name}: no valid sample'
        return f'- {name}: max {value["max"]:.3f} {unit}, median {value["median"]:.3f} {unit} ' \
               f'(n={value["n"]})'
    tracking = summary['tracking']
    report = [
        '# Issue #53 linear motion profile estimate', '',
        f'Status: **{"ESTIMATED" if profile else "INSUFFICIENT"}** (never REVIEWED/ACCEPTED '
        'automatically). Odometry positions only; no external reference.', '',
        f'- probe speed {args.probe_speed} m/s; geometry_hash `{hashes[0]}`, '
        f'extrinsics_hash `{hashes[1]}`, control_chain_hash `{hashes[2]}`',
        f'- valid: {summary["valid"]}/{summary["samples"]} (forward stops '
        f'{summary["forward"]}, backward stops {summary["backward"]}, watchdog trials '
        f'{summary["watchdog_trials"]}; need {args.min_per_direction} stops each way and '
        f'{args.min_watchdog} trials)',
        f'- invalid reasons: {", ".join(summary["invalid_notes"]) or "none"}',
        stat('start_latency_s', 's'), stat('reaction_s', 's'), stat('stop_tail_m', 'm'),
        stat('watchdog_s', 's'), stat('lateral_error_m', 'm'), stat('yaw_change_rad', 'rad'),
        '- tracking (speed before the stop / command): ' + (
            'no valid sample' if tracking is None else
            f'min {tracking["min"]:.2f}, median {tracking["median"]:.2f}'), '',
        '| bag | command m/s | result | reaction s | stop tail m | watchdog s | travel m '
        '| lateral m | yaw deg |',
        '| --- | --- | --- | --- | --- | --- | --- | --- | --- |', *rows, '']
    args.output_md.write_text('\n'.join(report), encoding='utf-8')
    if profile is None:
        print(f'INSUFFICIENT: {summary["valid"]} valid runs; wrote {args.output_md} only')
        return 1
    args.output_json.write_text(encode_linear_profile(profile) + '\n', encoding='utf-8')
    print(f'ESTIMATED: {summary["valid"]} valid runs; wrote {args.output_json} and '
          f'{args.output_md}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
