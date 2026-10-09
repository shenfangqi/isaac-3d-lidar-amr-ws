#!/usr/bin/env python3
"""
Offline 3D re-check of 2D localization candidates against the nvblox mesh.

Read-only: reads a bag with deskewed clouds, a diagnosis report from
``diagnose_localization_bag.py`` and the map's PLY mesh; never publishes,
accepts a pose or uses a saved pose.  Each candidate places the same
stationary clouds in the map; per height band (base frame) the share of
points within ``tolerance`` of a mesh vertex is reported, plus a
point-weighted composite over bands with enough points.

The provisional decision rule in the report is uncalibrated (two labelled
samples on 2026-10-10) and is not used by any runtime component.  A truth
file, if given, is only compared with the result afterwards.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.localization_surface_check import (  # noqa: E402,F401
    band_inliers, compare, DEFAULT_BANDS, load_ply_vertices, planar,
    quaternion_matrix, transform,
)

PROVISIONAL_MIN_COMPOSITE_GAP = 0.15


def chain_to_base(frame, static):
    """Full 3D transform base_footprint <- frame from static edges."""
    matrix, seen = np.eye(4), set()
    while frame != 'base_footprint':
        if frame in seen or frame not in static:
            raise ValueError(f'no static TF chain from {frame} to base_footprint')
        seen.add(frame)
        parent, edge = static[frame]
        matrix = edge @ matrix
        frame = parent
    return matrix


def same_pose(a, b, xy=0.3, yaw=math.radians(15)):
    """Whether two planar poses agree within the evaluation tolerance."""
    turn = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= xy and turn <= yaw


def read_clouds(bag, start_ns, end_ns):
    """Static TF edges, odometry and deskewed clouds within a window."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cloud_topic = '/fast_lio/cloud_registered_body'
    wanted = ['/tf_static', '/odom', cloud_topic]
    missing = set(wanted) - types.keys()
    if missing:
        raise ValueError(f'missing topics: {sorted(missing)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    static, odom, clouds = {}, [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, get_message(types[topic]))
        if topic == '/tf_static':
            for tf in message.transforms:
                t, q = tf.transform.translation, tf.transform.rotation
                static[tf.child_frame_id] = (
                    tf.header.frame_id,
                    transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))
            continue
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if topic == '/odom':
            p, q = message.pose.pose.position, message.pose.pose.orientation
            odom.append((stamp, p.x, p.y, math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        elif start_ns <= stamp <= end_ns:
            points = np.array([(p[0], p[1], p[2]) for p in point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)])
            clouds.append((stamp, message.header.frame_id, points))
    odom.sort()
    return static, odom, clouds


def stationary_base_points(static, odom, clouds, reference_ns, min_range,
                           max_speed=0.02, max_turn=0.03):
    """
    Clouds in the reference base frame.

    Clouds while moving, or without odometry within 60 ms of the source
    stamp and the +-250 ms stationarity check, are refused and counted.
    """
    stamps = np.array([o[0] for o in odom])

    def pose_at(stamp):
        index = int(np.argmin(np.abs(stamps - stamp)))
        if abs(stamps[index] - stamp) > 60_000_000:
            raise ValueError('no odometry near a cloud stamp')
        return odom[index][1:]

    reference = planar(*pose_at(reference_ns))
    kept, refused = [], {'moving': 0, 'no_odometry': 0}
    for stamp, frame, points in clouds:
        try:
            before = pose_at(stamp - 250_000_000)
            after = pose_at(stamp + 250_000_000)
            current = pose_at(stamp)
        except ValueError:
            refused['no_odometry'] += 1
            continue
        turn = abs(math.atan2(math.sin(after[2] - before[2]),
                              math.cos(after[2] - before[2])))
        if (math.hypot(after[0] - before[0], after[1] - before[1]) / 0.5
                > max_speed or turn / 0.5 > max_turn):
            refused['moving'] += 1
            continue
        relative = np.linalg.inv(reference) @ planar(*current)
        base = (relative @ chain_to_base(frame, static)
                @ np.c_[points, np.ones(len(points))].T).T[:, :3]
        kept.append(base[np.hypot(base[:, 0], base[:, 1]) >= min_range])
    if not kept:
        raise ValueError('no stationary cloud in the window')
    return np.vstack(kept), len(kept), refused


def sha256(path):
    """File digest for the report."""
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--diagnosis', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--view', type=int, default=0)
    parser.add_argument('--top', type=int, default=5)
    parser.add_argument('--window', type=float, default=4.0,
                        help='seconds of clouds after the first TRAIN frame')
    parser.add_argument('--tolerance', type=float, default=0.10)
    parser.add_argument('--min-range', type=float, default=0.5)
    parser.add_argument('--min-band-points', type=int, default=2000)
    parser.add_argument('--truth', type=Path,
                        help='evaluation only; never used for scoring')
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f'refusing to overwrite {args.output}')
    from scipy.spatial import cKDTree

    report = json.loads(args.diagnosis.read_text())
    view = next(v for v in report['views'] if v['view_id'] == args.view)
    candidates = sorted(
        (c for c in view['candidates'] if c.get('holdout')),
        key=lambda c: -c['holdout']['score'])[:args.top]
    if len(candidates) < 2:
        raise SystemExit('need at least two candidates')
    reference_ns = min(view['train_stamps'])
    static, odom, clouds = read_clouds(
        args.bag, reference_ns - 1_000_000_000,
        reference_ns + int(args.window * 1e9))
    points, used_clouds, refused = stationary_base_points(
        static, odom, clouds, reference_ns, args.min_range)
    tree = cKDTree(load_ply_vertices(args.mesh))
    poses = [(c['pose_at_reference']['x'], c['pose_at_reference']['y'],
              c['pose_at_reference']['yaw']) for c in candidates]
    per_candidate = [band_inliers(points, pose, tree, DEFAULT_BANDS,
                                  args.tolerance) for pose in poses]
    result = compare(per_candidate, args.min_band_points)
    best = result['order'][0]
    output = {
        'schema': 1, 'navigation_accepted': False,
        'mode': 'offline_3d_recheck_no_prior',
        'limitations': [
            'Mesh vertices only: no visibility or free-space model.',
            'Provisional decision rule is uncalibrated (two labelled samples).',
            'Truth, if supplied, is compared after scoring and never used in it.',
        ],
        'bag': str(args.bag), 'diagnosis': str(args.diagnosis),
        'mesh': str(args.mesh), 'mesh_sha256': sha256(args.mesh),
        'view': args.view, 'reference_ns': reference_ns,
        'clouds_used': used_clouds, 'clouds_refused': refused,
        'points': int(len(points)), 'tolerance_m': args.tolerance,
        'bands_m': [list(b) for b in DEFAULT_BANDS],
        'band_points': [n for n, _ in per_candidate[0]],
        'candidates': [{
            'pose': {'x': p[0], 'y': p[1], 'yaw': p[2]},
            'holdout_2d': c['holdout']['score'],
        } for p, c in zip(poses, candidates)],
        'comparison': result,
        'provisional': {
            'min_composite_gap': PROVISIONAL_MIN_COMPOSITE_GAP,
            'leader': best,
            'would_resolve': (result['composite_gap'] is not None
                              and result['composite_gap']
                              >= PROVISIONAL_MIN_COMPOSITE_GAP),
        },
    }
    if args.truth:
        truth = json.loads(args.truth.read_text())['true_candidate']
        true_pose = (truth['x'], truth['y'], math.radians(truth['yaw_deg']))
        output['evaluation'] = {
            'truth_file': str(args.truth),
            'leader_matches_truth': same_pose(poses[best], true_pose),
            'truth_rank': next((rank for rank, index in enumerate(result['order'])
                                if same_pose(poses[index], true_pose)), None),
        }
    args.output.write_text(json.dumps(output, indent=2))
    labels = [f'({p[0]:.2f},{p[1]:.2f},{math.degrees(p[2]):.0f}°)' for p in poses]
    for rank, index in enumerate(result['order']):
        shares = ' '.join(
            '  --  ' if not used else f'{100 * s:5.1f}%'
            for s, used in zip(result['shares'][index], result['bands_used']))
        print(f'{rank + 1}. {labels[index]:22s} composite '
              f'{100 * result["composite"][index]:5.1f}%  bands {shares}')
    print(f'clouds used {used_clouds}, refused {refused}, points {len(points)}')
    print(f'composite gap {result["composite_gap"]}; provisional resolve: '
          f'{output["provisional"]["would_resolve"]}')
    if 'evaluation' in output:
        print(f'evaluation: leader matches truth = '
              f'{output["evaluation"]["leader_matches_truth"]}, truth rank = '
              f'{output["evaluation"]["truth_rank"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
