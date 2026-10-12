#!/usr/bin/env python3
"""Read-only, deterministic multi-view localization diagnosis from a rosbag.

Reconstructs frames, not the manager's unrecorded executor scheduling. Never
publishes, accepts a navigation pose, or uses a saved pose prior. ROS imports
are confined to the reader; the interpolation and map loader are testable.
"""

import argparse
from array import array
from bisect import bisect_left
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import struct
import time
from types import SimpleNamespace as NS

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup import localization_hypotheses as lh  # noqa: E402
from isaac_3d_lidar_bringup.automatic_localization_quality import quaternion_yaw  # noqa: E402
from isaac_3d_lidar_bringup.localization_contracts import (  # noqa: E402
    FrameRole, Keyframe, SE2, normalize_angle,
)
from isaac_3d_lidar_bringup.localization_observations import snapshot_scan  # noqa: E402


def load_map(path):
    """Nav2 trinary PGM/YAML semantics, including image flip and origin yaw."""
    from PIL import Image
    config = yaml.safe_load(path.read_text())
    if config.get('mode', 'trinary') != 'trinary':
        raise ValueError('only trinary maps are supported')
    if not 0 <= config['free_thresh'] < config['occupied_thresh'] <= 1:
        raise ValueError('invalid map thresholds')
    if not math.isfinite(config['resolution']) or config['resolution'] <= 0:
        raise ValueError('invalid resolution')
    image_path = path.parent / config['image']
    with Image.open(image_path) as image:
        if image.mode != 'L':
            raise ValueError('use an 8-bit grayscale map')
        width, height = image.size
        pixels = list(image.getdata())
    data = array('b')
    for y in range(height - 1, -1, -1):
        for pixel in pixels[y * width:(y + 1) * width]:
            occ = (pixel if config['negate'] else 255 - pixel) / 255.0
            data.append(100 if occ > config['occupied_thresh'] else
                        0 if occ < config['free_thresh'] else -1)
    x, y, yaw = config['origin']
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError('invalid origin')
    return NS(frame_id='map', data=data, info=NS(
        width=width, height=height,
        # nav_msgs/MapMetaData stores resolution as float32, unlike origin.
        resolution=struct.unpack('<f', struct.pack('<f', config['resolution']))[0],
        origin=NS(position=NS(x=x, y=y), orientation=NS(
            x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))


def interpolate_pose(samples, stamps, stamp, max_gap_ns=200_000_000):
    """Interpolate bracketing source stamps; no extrapolation or nearest TF."""
    index = bisect_left(stamps, stamp)
    if index < len(stamps) and stamps[index] == stamp:
        return samples[index][1]
    if index == 0 or index == len(stamps):
        raise ValueError('source stamp outside odometry')
    ta, a = samples[index - 1]
    tb, b = samples[index]
    if tb - ta > max_gap_ns:
        raise ValueError('odometry interpolation gap exceeds limit')
    if (math.hypot(a.x - b.x, a.y - b.y) > 0.20
            or abs(normalize_angle(b.yaw - a.yaw)) > 0.35):
        raise ValueError('odometry discontinuity')
    weight = (stamp - ta) / (tb - ta)
    return SE2(a.x + weight * (b.x - a.x), a.y + weight * (b.y - a.y),
               normalize_angle(a.yaw + weight * normalize_angle(b.yaw - a.yaw)))


def state_intervals(statuses):
    intervals = []
    for stamp, status in statuses:
        state = status['state']
        if not intervals or state != intervals[-1]['state']:
            if intervals:
                intervals[-1]['end_ns'] = stamp
            intervals.append({'state': state, 'start_ns': stamp, 'end_ns': stamp})
        intervals[-1]['end_ns'] = stamp
    return intervals


def stationary_capture_intervals(scans):
    """
    Synthetic state windows for a capture recorded after localization ended.

    Since 2026-10-10 no bag is recorded during localization (it delayed
    /odom on the Jetson); the scene is captured afterwards with the robot
    still.  Such a bag has no COLLECT/SEARCH/VERIFY states, so its first
    part stands in for collection and search and its second part for an
    independent VERIFY window.
    """
    first, last = scans[0][0], scans[-1][0]
    if last - first < 20_000_000_000:
        raise ValueError('a stationary capture needs at least 20 s of scans')
    start, middle, end = first + 2_000_000_000, (first + last) // 2, last - 1_000_000_000
    return [{'state': 'COLLECT_STATIC', 'start_ns': start, 'end_ns': start + 1_000_000_000},
            {'state': 'SEARCH_MULTI_VIEW', 'start_ns': start + 1_000_000_000,
             'end_ns': middle},
            {'state': 'VERIFY_HYPOTHESES', 'start_ns': middle, 'end_ns': end}]


def read_bag(path, scan_topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = {scan_topic, '/odom', '/tf_static', '/automatic_localization/status'}
    missing = wanted - types.keys()
    if missing:
        raise ValueError(f'missing topics: {sorted(missing)}')
    # Do not resolve unrelated custom message types; they need not be installed.
    classes = {topic: get_message(types[topic]) for topic in wanted}
    scans, odom, statuses, static, counts = [], [], [], {}, {}
    while reader.has_next():
        topic, data, received = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic not in wanted:
            continue
        message = deserialize_message(data, classes[topic])
        if topic == scan_topic:
            scans.append((received, snapshot_scan(message)))
        elif topic == '/odom':
            if (message.header.frame_id != 'odom'
                    or message.child_frame_id != 'base_footprint'):
                raise ValueError('expected odom -> base_footprint odometry')
            p = message.pose.pose
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            odom.append((stamp, SE2(p.position.x, p.position.y,
                                   quaternion_yaw(p.orientation))))
        elif topic == '/automatic_localization/status':
            statuses.append((received, json.loads(message.data)))
        else:
            for tf in message.transforms:
                transform = tf.transform
                # A nonplanar sensor transform cannot be silently flattened.
                if abs(transform.rotation.x) > 1e-6 or abs(transform.rotation.y) > 1e-6:
                    continue
                edge = (tf.header.frame_id, SE2(
                    transform.translation.x, transform.translation.y,
                    quaternion_yaw(transform.rotation)))
                if tf.child_frame_id in static and static[tf.child_frame_id] != edge:
                    raise ValueError('static TF changed within bag')
                static[tf.child_frame_id] = edge
    odom.sort(key=lambda item: item[0])
    if len({t for t, _ in odom}) != len(odom):
        raise ValueError('duplicate odometry stamps')
    sessions = {d['session'] for _, d in statuses if d.get('session')}
    if len(sessions) != 1:
        raise ValueError('select a bag with exactly one localization session')
    if not scans or not odom:
        raise ValueError('empty scan/odometry data')
    return scans, odom, statuses, static, counts, types, sessions.pop()


def base_to_scan(frame, static):
    chain, seen = [], set()
    while frame != 'base_footprint':
        if frame in seen or frame not in static:
            raise ValueError(f'missing/invalid planar static TF chain for {frame}')
        seen.add(frame)
        frame, transform = static[frame]
        chain.append(transform)
    result = SE2(0.0, 0.0, 0.0)
    for transform in reversed(chain):
        result = result.compose(transform)
    return result


def select_frames(scans, odom, static, start, end, session, view, role,
                  count, used_stamps, next_id):
    """Choose disjoint stationary frames wholly within a recorded stopped phase."""
    stamps = [t for t, _ in odom]
    selected, rejected = [], {}
    for received, scan in scans:
        if not start <= received < end or scan.stamp_ns in used_stamps:
            continue
        try:
            if not -50_000_000 <= received - scan.stamp_ns <= 500_000_000:
                raise ValueError('stale/future scan at recording time')
            pose = interpolate_pose(odom, stamps, scan.stamp_ns)
            before = interpolate_pose(odom, stamps, scan.stamp_ns - 250_000_000)
            after = interpolate_pose(odom, stamps, scan.stamp_ns + 250_000_000)
            if (math.hypot(after.x - before.x, after.y - before.y) / 0.5 > 0.02
                    or abs(normalize_angle(after.yaw - before.yaw)) / 0.5 > 0.03):
                raise ValueError('not stationary in source-time window')
            sensor = base_to_scan(scan.frame_id, static)
        except ValueError as error:
            reason = str(error)
            rejected[reason] = rejected.get(reason, 0) + 1
            continue
        selected.append(Keyframe(
            next_id + len(selected), session, scan.stamp_ns, view, scan,
            pose, sensor, received / 1e9, role))
        used_stamps.add(scan.stamp_ns)
        if len(selected) == count:
            break
    if len(selected) != count:
        raise ValueError(f'view {view} {role.value}: {len(selected)}/{count} frames; {rejected}')
    return tuple(selected), rejected


def frames_from_trace(references, scans, session, role):
    """Recover actual worker frames when the opt-in diagnostic trace exists."""
    by_stamp = {scan.stamp_ns: (received, scan) for received, scan in scans}
    frames, seen = [], set()
    for ref in references:
        stamp = ref['stamp_ns']
        if (ref['session'] != session or ref['role'] != role.value
                or stamp in seen or stamp not in by_stamp):
            raise ValueError('invalid trace or referenced scan absent from bag')
        seen.add(stamp)
        received, scan = by_stamp[stamp]
        frames.append(Keyframe(ref['id'], session, stamp, ref['view_id'], scan,
                               SE2(**ref['T_odom_base']), SE2(**ref['T_base_scan']),
                               received / 1e9, role))
    return tuple(frames)


def configs(parameters, grid):
    """Mirror the manager's YAML-to-search mapping, not unrelated defaults."""
    names = {
        'occupied_threshold': 'occupied_threshold',
        'coarse_step_m': 'global_search_position_step_m',
        'coarse_yaw_step_rad': 'global_search_yaw_step_rad',
        'coarse_beams': 'global_search_coarse_beams',
        'refine_beams': 'scan_score_max_beams',
        'cluster_xy_m': 'independent_cluster_xy_m',
        'cluster_yaw_rad': 'independent_cluster_yaw_rad',
        'max_refined_clusters': 'max_refined_clusters',
        'max_extra_refined_clusters': 'max_extra_refined_clusters',
    }
    config = lh.SearchConfig(**{k: parameters[v] for k, v in names.items()},
                             tolerance_cells=math.ceil(
                                 parameters['scan_match_tolerance_m'] / grid.info.resolution))
    thresholds = lh.ValidationThresholds(
        min_score=parameters['min_scan_map_score'],
        min_coverage=parameters['min_scan_map_coverage'],
        min_known=parameters['min_valid_scan_beams'],
        max_conflict=parameters['global_search_max_wall_conflict_ratio'],
        min_margin=parameters['global_search_min_score_margin'])
    return config, thresholds


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def diagnose(args):
    grid = load_map(args.map)
    scans, odom, statuses, static, counts, types, session = read_bag(args.bag, args.scan_topic)
    digest = lh.map_hash(grid)
    hashes = {d['map_hash'] for _, d in statuses if d.get('map_hash')}
    if hashes != {digest}:
        raise ValueError(f'input map hash {digest} does not match bag {hashes}')
    parameters = yaml.safe_load(args.config.read_text())[
        'automatic_localization_manager']['ros__parameters']
    config, thresholds = configs(parameters, grid)
    intervals = state_intervals(statuses)
    if args.stationary_capture:
        intervals = stationary_capture_intervals(scans)
    t0 = statuses[0][0]
    report = {
        'schema': 1, 'navigation_accepted': False, 'complete': False,
        'mode': 'offline_no_prior',
        'limitations': [
            'Recorded state boundaries approximate executor frame selection.',
            'Train windows may extend into the immediately following stationary search.',
            'Search-phase holdout is explicitly labelled when no VERIFY was recorded.',
            'Uses current supplied configuration, not an assertion of historical parity.',
            'No independently measured ground truth is supplied; a leading candidate is not truth.',
            '2D bags cannot reconstruct discarded point heights.',
        ] + (['Stationary capture: synthetic state windows (first half '
              'collection/search, second half VERIFY), not a recorded session.']
             if args.stationary_capture else []),
        'bag': str(args.bag), 'session': session, 'map_hash': digest,
        'config_sha256': sha256(args.config), 'search_config': asdict(config),
        'code_sha256': {'diagnose_localization_bag.py': sha256(Path(__file__)),
                        'localization_hypotheses.py': sha256(Path(lh.__file__))},
        'thresholds': asdict(thresholds), 'topics': counts,
        'has_3d_points': any(t in ('sensor_msgs/msg/PointCloud2',
                                 'livox_ros_driver2/msg/CustomMsg') for t in types.values()),
        'bag_files_sha256': {p.name: sha256(p) for p in args.bag.glob('*.db3')},
        'recorded_timeline': [dict(
            state=v['state'], start_sec=(v['start_ns'] - t0) / 1e9,
            duration_sec=(v['end_ns'] - v['start_ns']) / 1e9) for v in intervals],
        'recorded_final_status': statuses[-1][1], 'views': [],
    }
    train_all, basis, cache = (), None, None
    used, next_id = set(), 0
    for i, interval in enumerate(intervals):
        if interval['state'] != 'COLLECT_STATIC':
            continue
        view = len(report['views'])
        following = []
        for item in intervals[i + 1:]:
            if item['state'] in ('PLAN_PROBE', 'EXECUTE_PROBE', 'SAFE_STOP',
                                 'COLLECT_STATIC', 'STOP_AND_VERIFY'):
                break
            following.append(item)
        search = next((v for v in following if v['state'] == 'SEARCH_MULTI_VIEW'), None)
        verify = next((v for v in following if v['state'] == 'VERIFY_HYPOTHESES'), None)
        if search is None:
            raise ValueError('COLLECT_STATIC has no following search window')
        next_collect = next((v for v in intervals[i + 1:]
                             if v['state'] == 'COLLECT_STATIC'), None)
        trace_end = next_collect['start_ns'] if next_collect else statuses[-1][0] + 1
        traces = [d['localization_diagnostics'] for t, d in statuses
                  if interval['start_ns'] <= t < trace_end
                  and d.get('localization_diagnostics', {}).get('result') is not None
                  and d['localization_diagnostics'].get('train')]
        trace = max(traces, key=lambda tr: (bool(tr.get('decision')), len(tr.get('holdout', ())))) if traces else None
        if trace:
            train_all = frames_from_trace(trace['train'], scans, session, FrameRole.TRAIN)
            train = tuple(f for f in train_all if f.view_id == view)
            if not train:
                raise ValueError('trace has no current view')
            rejected = {}
            used.update(f.stamp_ns for f in train_all)
            next_id = max(f.id for f in train_all) + 1
        else:
            train, rejected = select_frames(
                scans, odom, static, interval['start_ns'], search['end_ns'],
                session, view, FrameRole.TRAIN, parameters['train_frames_per_view'],
                used, next_id)
            next_id += len(train)
            train_all += train
        hold_start = verify['start_ns'] if verify else max(
            search['start_ns'] + 2_000_000_000,
            int(train[-1].receipt_mono * 1e9) + 1_000_000_000)
        hold_end = verify['end_ns'] if verify else search['end_ns']
        if trace and trace.get('holdout'):
            holdout = frames_from_trace(trace['holdout'], scans, session, FrameRole.HOLDOUT)
            if any(f.stamp_ns in used for f in holdout):
                raise ValueError('trace reuses TRAIN/previous HOLDOUT stamp')
            used.update(f.stamp_ns for f in holdout)
            hold_rejected = {}
        else:
            holdout, hold_rejected = select_frames(
                scans, odom, static, hold_start, hold_end, session, view,
                FrameRole.HOLDOUT, parameters['holdout_frames_per_view'], used, next_id)
        next_id += len(holdout)
        started = time.monotonic()
        if basis is None:
            output = lh.search_multiview_cached(
                grid, train_all, config, deadline=started + args.search_timeout,
                coarse_cache=cache)
            result, cache, kind = output.result, output.coarse_cache, 'map_wide'
        else:
            result = lh.recheck_hypotheses(
                grid, basis[0], basis[1], train_all, config,
                deadline=started + args.search_timeout)
            kind = 'candidate_recheck'
            if result.complete and not any(h.score >= thresholds.min_score for h in result.hypotheses):
                output = lh.search_multiview_cached(
                    grid, train_all, config, deadline=time.monotonic() + args.search_timeout,
                    coarse_cache=cache)
                result, cache, kind = output.result, output.coarse_cache, 'recheck_then_map_wide'
        reference = train_all[0].T_odom_base
        decision = lh.validate_hypotheses(
            grid, result, train_all, holdout, config, thresholds,
            deadline=time.monotonic() + args.search_timeout)
        candidates = []
        for h in result.hypotheses:
            pose = SE2(h.x, h.y, h.yaw)
            metrics = lh.score_pose(grid, pose, holdout, reference, config, config.refine_beams)
            candidates.append(dict(
                cluster_id=h.cluster_id, pose_at_reference=asdict(pose),
                pose_at_view=asdict(lh.seed_pose_at_current_time(pose, reference, train[0].T_odom_base)),
                train_score=h.score, train_per_view=h.per_view, holdout=metrics))
        candidates.sort(key=lambda h: (h['holdout']['score'], -h['holdout']['conflict']), reverse=True)
        margin = (candidates[0]['holdout']['score'] - candidates[1]['holdout']['score']
                  if len(candidates) > 1 else None)
        row = dict(view_id=view, kind=kind, search_complete=result.complete,
                   search_reason=result.reason, elapsed_sec=time.monotonic() - started,
                   # Coarse seeds of competitors a budget-limited search
                   # left unrefined; the 3D decision must cover them too.
                   unrefined=[list(seed) for seed in result.unrefined],
                   decision=asdict(decision), score_margin=margin, candidates=candidates,
                   holdout_origin='VERIFY_HYPOTHESES' if verify else 'SEARCH_MULTI_VIEW_diagnostic_only',
                   frame_source={
                       'train': 'recorded_worker_trace' if trace else 'reconstructed_state_windows',
                       'holdout': 'recorded_worker_trace' if trace and trace.get('holdout')
                       else 'reconstructed_state_windows'},
                   train_stamps=[f.stamp_ns for f in train], holdout_stamps=[f.stamp_ns for f in holdout],
                   train_rejections=rejected, holdout_rejections=hold_rejected,
                   odom_at_view=asdict(train[0].T_odom_base))
        report['views'].append(row)
        basis = (result, reference) if result.complete and result.hypotheses else None
        print(f'{args.bag.name} view={view} {kind} complete={result.complete} '
              f'n={len(candidates)} margin={margin} decision={decision.reason or "PASS"}', flush=True)
        # Preserve partial diagnostics if a later view cannot be reconstructed.
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    report['complete'] = bool(report['views'])
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT / 'src/isaac_3d_lidar_bringup/config/nav2/carbot_auto_localization_real.yaml')
    parser.add_argument('--scan-topic', default='/scan_localization')
    parser.add_argument('--search-timeout', type=float, default=120.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument(
        '--stationary-capture', action='store_true',
        help='the bag was recorded with the robot still after localization; '
             'use its first half as collection/search and its second half '
             'as the VERIFY window')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; choose a new path')
    if not math.isfinite(args.search_timeout) or args.search_timeout <= 0:
        parser.error('search timeout must be finite and positive')
    report = diagnose(args)
    if not report['views']:
        raise SystemExit('no localization views')


if __name__ == '__main__':
    main()
