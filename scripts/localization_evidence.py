#!/usr/bin/env python3
"""Record bounded ROS evidence or export a stationary bag for offline scoring.

Never publishes ROS messages or calls a lifecycle/motion service.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

import yaml

TOPICS = ['/map', '/scan', '/scan_localization', '/tf', '/tf_static',
          '/odom', '/fast_lio/imu_odom',
          '/amcl_pose', '/particle_cloud', '/initialpose',
          '/automatic_localization/status', '/cmd_vel_command', '/cmd_vel']

# Record deskewed points plus the full-attitude transform chain. The 2D
# scans alone cannot recover discarded heights. No driver custom types are
# needed to replay PointCloud2; raw packets/IMU may be added explicitly.
LOCALIZATION_3D_TOPICS = ['/fast_lio/cloud_registered_body',
                          '/fast_lio/imu_odom']


def record(args):
    topics = list(dict.fromkeys([
        *TOPICS, *(LOCALIZATION_3D_TOPICS if args.include_localization_3d else []),
        *args.extra_topic]))
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(args.map.read_text())
    image = (args.map.parent / config['image']).resolve()
    artifacts = [args.map, image, *args.artifact]
    manifest = {'schema': 1, 'kind': 'real_unverified', 'topics': topics,
                'created_unix': time.time(), 'reference_confirmed': False,
                'reference': None, 'files': {}, 'complete': False}
    for index, path in enumerate(artifacts):
        data = path.read_bytes()
        name = f'{index}_{path.name}'
        (directory / name).write_bytes(data)
        manifest['files'][str(path)] = {
            'copy': name, 'sha256': hashlib.sha256(data).hexdigest()}
    for name, command in [
            ('git_head', ['git', 'rev-parse', 'HEAD']),
            ('git_status', ['git', 'status', '--porcelain']),
            ('git_diff', ['git', 'diff', '--binary'])]:
        result = subprocess.run(command, capture_output=True, text=True)
        (directory / f'{name}.txt').write_text(result.stdout + result.stderr)
    # Runtime parameters, not just YAML defaults. Preserve failure diagnostics.
    for node in ['amcl', 'automatic_localization_manager',
                 'mid360_pointcloud_to_laserscan',
                 'mid360_localization_pointcloud_to_laserscan']:
        try:
            result = subprocess.run(['ros2', 'param', 'dump', '/' + node],
                                    capture_output=True, text=True, timeout=15)
            (directory / f'{node}.yaml').write_text(result.stdout)
            manifest[node + '_param_dump_returncode'] = result.returncode
        except subprocess.TimeoutExpired:
            manifest[node + '_param_dump_returncode'] = 'timeout'
    qos = {topic: {'reliability': 'best_effort', 'durability': 'volatile',
                   'history': 'keep_last', 'depth': 10} for topic in topics}
    for topic in ['/map', '/tf_static', '/automatic_localization/status']:
        qos[topic].update(reliability='reliable', durability='transient_local')
    qos_path = directory / 'qos.yaml'
    qos_path.write_text(yaml.safe_dump(qos))
    manifest_path = directory / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2))
    process = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(directory / 'bag'),
                                '--qos-profile-overrides-path', str(qos_path), *topics])
    try:
        process.wait(timeout=args.seconds)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=5)
    manifest['recorder_returncode'] = process.returncode
    manifest['complete'] = (directory / 'bag/metadata.yaml').exists()
    manifest['ended_unix'] = time.time()
    manifest_path.write_text(json.dumps(manifest, indent=2))
    if not manifest['complete']:
        raise SystemExit('Recording incomplete; keep files for diagnosis')
    print(manifest_path)


def export(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: get_message(topic.type)
             for topic in reader.get_all_topics_and_types()}
    grid = None
    scans = []
    first_stamp = None
    while reader.has_next():
        topic, data, received = reader.read_next()
        if first_stamp is None:
            first_stamp = received
        if topic not in ('/map', args.scan_topic):
            continue
        message = message_to_ordereddict(deserialize_message(data, types[topic]))
        if topic == '/map':
            if grid is not None and grid['data'] != message['data']:
                raise SystemExit('Map changed during recording; split the dataset')
            grid = message
        elif topic == args.scan_topic and args.start <= (received - first_stamp) / 1e9 <= args.end:
            if message['header']['frame_id'] != 'base_footprint':
                raise SystemExit('Exporter requires scan in base_footprint; transform explicitly')
            # JSON null denotes invalid/infinite ranges; offline scorer restores NaN.
            import math
            message['ranges'] = [x if math.isfinite(x) else None
                                 for x in message['ranges']]
            scans.append({'receive_stamp_ns': received, 'scan': message})
    if grid is None or not scans:
        raise SystemExit('Missing /map or selected /scan window')
    dataset = {'schema': 1, 'kind': 'real_unverified', 'map': grid,
               'scans': scans, 'bag': str(args.bag.resolve()),
               'scan_topic': args.scan_topic,
               'window_sec': [args.start, args.end],
               'requires_stationary_reference': True}
    with args.output.open('x') as stream:
        json.dump(dataset, stream, allow_nan=False)
    print(f'Exported {len(scans)} scans; stationary/reference verification still required')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('record')
    capture.add_argument('--map', type=Path, required=True)
    capture.add_argument('--output', type=Path, required=True)
    capture.add_argument('--seconds', type=float, default=60)
    capture.add_argument('--artifact', type=Path, action='append', default=[])
    capture.add_argument('--extra-topic', action='append', default=[],
                         help='additional topic to record; repeat as needed')
    capture.add_argument('--include-localization-3d', action='store_true',
                         help='also record deskewed body PointCloud2 for height-layer analysis')
    convert = commands.add_parser('export')
    convert.add_argument('--bag', type=Path, required=True)
    convert.add_argument('--start', type=float, required=True)
    convert.add_argument('--end', type=float, required=True)
    convert.add_argument('--output', type=Path, required=True)
    convert.add_argument('--scan-topic', default='/scan',
                         help='LaserScan topic to export (default: /scan)')
    args = parser.parse_args()
    if args.command == 'record':
        if not 1 <= args.seconds <= 600:
            parser.error('--seconds must be between 1 and 600')
        record(args)
    else:
        if not 0 <= args.start < args.end:
            parser.error('require 0 <= start < end')
        export(args)


if __name__ == '__main__':
    main()
