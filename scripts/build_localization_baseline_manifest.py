#!/usr/bin/env python3
"""Build the Issue #13 baseline bag manifest from rosbag2 metadata only.

Reads ``metadata.yaml`` files (no ROS needed), records topic types and
counts, content hashes and which confined-localization evidence channels each
bag can or cannot support.  Large bags stay where they are; only this small
manifest is committed.
"""

import argparse
import hashlib
import json
from pathlib import Path

import yaml


# Evidence channels named by the Issue #13 input contract.
CHANNELS = {
    'localization_scan': ('/scan_localization',),
    'safety_scan': ('/scan',),
    'raw_lidar': ('/livox/lidar',),
    'fast_lio_body_cloud': ('/fast_lio/cloud_registered_body',),
    'odom': ('/odom',),
    'fast_lio_odom': ('/fast_lio/imu_odom',),
    'map': ('/map',),
    'tf': ('/tf',),
    'tf_static': ('/tf_static',),
    'command_chain': ('/cmd_vel_command', '/cmd_vel'),
    'localization_status': ('/automatic_localization/status',),
    'localization_emergency_stop': ('/localization/emergency_stop',),
    'base_status': ('/carbot/status',),
}

# What each audit question needs before a bag can answer it.
USES = {
    'stationary_search_replay': (
        'localization_scan', 'odom', 'map', 'tf', 'tf_static'),
    'derived_scan_near_field_audit': ('safety_scan', 'tf', 'tf_static'),
    'raw_near_field_audit': ('raw_lidar', 'tf_static'),
    'body_cloud_near_field_audit': ('fast_lio_body_cloud', 'tf_static'),
    'rotation_response_analysis': ('command_chain', 'odom'),
    'control_source_audit': (
        'command_chain', 'localization_emergency_stop', 'base_status'),
}


def sha256_file(path, chunk=1 << 20):
    """Hash a file without loading it at once; None if unreadable."""
    digest = hashlib.sha256()
    try:
        with open(path, 'rb') as stream:
            while True:
                block = stream.read(chunk)
                if not block:
                    return digest.hexdigest()
                digest.update(block)
    except PermissionError:
        # Container-created files may be root-owned; record, never guess.
        return None


def describe_bag(metadata_path, root, hash_storage):
    """Return one manifest entry for a rosbag2 directory."""
    metadata_path = Path(metadata_path)
    info = yaml.safe_load(metadata_path.read_text('utf-8'))[
        'rosbag2_bagfile_information']
    topics = {
        item['topic_metadata']['name']: {
            'type': item['topic_metadata']['type'],
            'count': item['message_count'],
        }
        for item in info['topics_with_message_count']
    }
    channels = {}
    for channel, names in CHANNELS.items():
        present = [name for name in names if topics.get(name, {}).get('count')]
        channels[channel] = len(present) == len(names)
    uses = {use: all(channels[name] for name in needed)
            for use, needed in USES.items()}
    directory = metadata_path.parent
    files = {}
    for relative in info.get('relative_file_paths', []):
        path = directory / relative
        files[relative] = {
            'bytes': path.stat().st_size if path.exists() else None,
            'sha256': (sha256_file(path)
                       if hash_storage and path.exists() else None),
        }
    evidence_dir = directory.parent if directory.name == 'bag' else directory
    sidecars = {}
    for path in sorted(evidence_dir.iterdir()):
        if path.is_file() and path.suffix in ('.yaml', '.json', '.txt') \
                and path != metadata_path:
            sidecars[path.name] = sha256_file(path)
    return {
        'path': str(directory.relative_to(root)),
        'storage': info['storage_identifier'],
        'start_ns': info['starting_time']['nanoseconds_since_epoch'],
        'duration_sec': round(info['duration']['nanoseconds'] / 1e9, 3),
        'message_count': info['message_count'],
        'metadata_sha256': sha256_file(metadata_path),
        'files': files,
        'sidecar_sha256': sidecars,
        'topics': topics,
        'raw_lidar_type': topics.get('/livox/lidar', {}).get('type'),
        'channels': channels,
        'usable_for': uses,
    }


def build_manifest(root, search, hash_storage):
    """Describe every bag below ``search`` and summarize coverage gaps."""
    root = Path(root).resolve()
    bags = [describe_bag(path, root, hash_storage)
            for path in sorted(Path(search).resolve().rglob('metadata.yaml'))]
    coverage = {use: [bag['path'] for bag in bags if bag['usable_for'][use]]
                for use in USES}
    return {
        'schema_version': 1,
        'kind': 'issue13_pr0_baseline_bags',
        'search_root': str(Path(search).resolve().relative_to(root)),
        'storage_hashed': hash_storage,
        'bag_count': len(bags),
        'coverage': coverage,
        'gaps': sorted(use for use, paths in coverage.items() if not paths),
        'raw_lidar_types': sorted({bag['raw_lidar_type'] for bag in bags
                                   if bag['raw_lidar_type']}),
        'bags': bags,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--search', type=Path,
                        help='directory to scan (default: ROOT/calibration_data)')
    parser.add_argument('--hash-storage', action='store_true',
                        help='also hash .db3/.mcap payloads (slow)')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    search = args.search or args.root / 'calibration_data'
    manifest = build_manifest(args.root, search, args.hash_storage)
    encoded = json.dumps(manifest, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + '\n', 'utf-8')
    else:
        print(encoded)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
