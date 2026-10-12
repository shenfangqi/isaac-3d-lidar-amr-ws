#!/usr/bin/env python3
"""
Where does a bag's scene differ from the map?  Local-submap version.  (read-only)

Builds a local submap (isaac_3d_lidar_bringup/localization_submap.py) from
all deskewed clouds of a bag with their /odom poses, places it at a known
map pose and reports the added, removed and see-through structure grouped
into objects.  The multi-frame successor of check_scene_consistency.py.

The pose comes from an operator ground-truth JSON (the robot pose while the
reference cloud was taken) or --pose, optionally refined on the 3D fit
(--refine, at most SurfaceRefineConfig limits).  A wrong pose shows up as
change everywhere.  Every change is labelled with the variable regions and
surface additions it overlaps, so a run with --compare-to bare can be
checked against the known causes.  No ROS node is started.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src/isaac_3d_lidar_bringup'))
from isaac_3d_lidar_bringup.localization_submap import (  # noqa: E402
    build_submap, compare_submap, CompareConfig, SubmapConfig, SubmapFrame)
from isaac_3d_lidar_bringup.localization_surface_additions import (  # noqa: E402
    load_surface_additions)
from isaac_3d_lidar_bringup.localization_surface_check import (  # noqa: E402
    planar, surface_tree, transform)
from isaac_3d_lidar_bringup.localization_surface_model import (  # noqa: E402
    load_surface_model)
from isaac_3d_lidar_bringup.localization_surface_validation import (  # noqa: E402
    refine_surface_pose, SurfaceRefineConfig)
from isaac_3d_lidar_bringup.localization_variable_regions import (  # noqa: E402
    load_variable_regions)
from recheck_candidates_3d import chain_to_base  # noqa: E402

CLOUD_TOPIC = '/fast_lio/cloud_registered_body'


def read_frames(bag, start_ns=None, end_ns=None, every=1):
    """
    Submap frames of a bag: every ``every``-th cloud with its odometry pose.

    The planar /odom pose is interpolated at the cloud stamp (FAST-LIO
    stamps a deskewed cloud with its scan end); clouds outside the odometry
    span are skipped.  Returns ``(frames, odom)``, ``odom`` the sorted
    ``(stamp_ns, x, y, yaw)`` rows.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = ['/tf_static', '/odom', CLOUD_TOPIC]
    missing = set(wanted) - types.keys()
    if missing:
        raise ValueError(f'missing topics: {sorted(missing)}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    static, odom, clouds, seen = {}, [], [], 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, get_message(types[topic]))
        if topic == '/tf_static':
            for tf in message.transforms:
                t, q = tf.transform.translation, tf.transform.rotation
                static[tf.child_frame_id] = (
                    tf.header.frame_id, transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))
            continue
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if topic == '/odom':
            p, q = message.pose.pose.position, message.pose.pose.orientation
            odom.append((stamp, p.x, p.y, math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        elif ((start_ns is None or stamp >= start_ns)
              and (end_ns is None or stamp <= end_ns)):
            seen += 1
            if (seen - 1) % every:
                continue
            points = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
            clouds.append((stamp, message.header.frame_id, np.asarray(points, float)))
    odom.sort()
    table = np.asarray(odom, float)
    yaw = np.unwrap(table[:, 3])
    frames = []
    for stamp, frame, points in clouds:
        if not table[0, 0] <= stamp <= table[-1, 0]:
            continue
        x, y, heading = (np.interp(stamp, table[:, 0], column)
                         for column in (table[:, 1], table[:, 2], yaw))
        frames.append(SubmapFrame(stamp, planar(x, y, heading),
                                  chain_to_base(frame, static), points))
    return frames, odom


def overlaps(change, box):
    low, high = np.asarray(change.low), np.asarray(change.high)
    return bool((low <= box[3:]).all() and (high >= box[:3]).all())


def label(change, regions, additions_tree):
    """Known causes a change overlaps: variable regions and surface additions."""
    names = []
    if regions is not None:
        names += [name for name, box in zip(regions.names, regions.boxes)
                  if overlaps(change, box)]
    if additions_tree is not None:
        distance = additions_tree.query([change.centroid])[0][0]
        low, high = np.asarray(change.low) - .1, np.asarray(change.high) + .1
        inside = ((additions_tree.data >= low) & (additions_tree.data <= high)).all(1)
        if inside.any() or distance < .2:
            names.append('surface additions')
    return names


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--truth', type=Path, help='operator ground-truth JSON')
    parser.add_argument('--pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'))
    parser.add_argument('--mesh', type=Path, required=True)
    parser.add_argument('--spacing', type=float, default=.02)
    parser.add_argument('--variable-regions', type=Path,
                        help='label changes with these regions (and drop them from the '
                             'map with --compare-to configured)')
    parser.add_argument('--surface-additions', type=Path)
    parser.add_argument('--compare-to', choices=('bare', 'configured'), default='bare',
                        help='bare mesh, or the mesh with the regions and additions as '
                             'localization uses it')
    parser.add_argument('--refine', action='store_true',
                        help='refine the pose on the 3D fit before comparing')
    parser.add_argument('--every', type=int, default=1, help='use every n-th cloud')
    parser.add_argument('--reference', choices=('middle', 'last'), default='middle',
                        help='the cloud whose robot pose the pose describes')
    parser.add_argument('--seconds', type=float, help='only the last N s of clouds')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (args.pose is None) == (args.truth is None):
        parser.error('give exactly one of --pose or --truth')
    if args.output.exists():
        parser.error(f'{args.output} exists; remove it first')
    if args.truth:
        truth = json.loads(args.truth.read_text())['true_candidate']
        pose = (truth['x'], truth['y'], math.radians(truth['yaw_deg']))
    else:
        pose = (args.pose[0], args.pose[1], math.radians(args.pose[2]))

    started = time.monotonic()
    model = load_surface_model(args.mesh, args.spacing)
    regions = (load_variable_regions(args.variable_regions, model.mesh_sha256)
               if args.variable_regions else None)
    additions = (load_surface_additions(args.surface_additions, model.mesh_sha256)
                 if args.surface_additions else None)
    configured = args.compare_to == 'configured'
    tree = surface_tree(model.points, regions if configured else None,
                        additions if configured else None)
    end_ns = None
    frames, odom = read_frames(args.bag, every=args.every)
    if args.seconds:
        end_ns = frames[-1].stamp_ns
        frames = [f for f in frames if f.stamp_ns >= end_ns - args.seconds * 1e9]
    reference = frames[-1 if args.reference == 'last' else len(frames) // 2]
    read_s = time.monotonic() - started
    submap = build_submap(frames, reference.odom_T_base, SubmapConfig())
    build_s = time.monotonic() - started - read_s
    refined = pose
    if args.refine:
        refined, converged = refine_surface_pose(
            tree, submap.points, pose, SurfaceRefineConfig(max_shift_m=.3,
                                                           max_turn_rad=math.radians(10)),
            lambda: False)
        if not converged:
            print('warning: refinement hit its limit; using the given pose')
            refined = pose
    result = compare_submap(submap, refined, tree, CompareConfig())
    compare_s = time.monotonic() - started - read_s - build_s

    from scipy.spatial import cKDTree
    additions_tree = None if additions is None else cKDTree(additions)
    travel = [math.hypot(o[1] - odom[0][1], o[2] - odom[0][2]) for o in odom]
    changes = [dict(kind=c.kind, points=c.points, low=c.low, high=c.high,
                    centroid=c.centroid, known=label(c, regions, additions_tree))
               for c in result.changes]
    report = dict(
        schema=1, mode='offline_submap_changes', bag=str(args.bag),
        mesh_sha256=model.mesh_sha256, compare_to=args.compare_to,
        pose_given=[pose[0], pose[1], math.degrees(pose[2])],
        pose_used=[refined[0], refined[1], math.degrees(refined[2])],
        frames=submap.frame_count, odom_travel_max_m=round(max(travel), 3),
        voxels=int(len(submap.points)), dropped_transient=int(len(submap.dropped)),
        added_share=round(float(result.added.mean()), 4),
        see_through=round(result.see_through, 4),
        removed_voxels=int(len(result.removed)), changes=changes,
        seconds=dict(read=round(read_s, 1), build=round(build_s, 1),
                     compare=round(compare_s, 1)))
    args.output.write_text(json.dumps(report, indent=1) + '\n')
    print(f'{args.bag.name}: {submap.frame_count} frames, {len(submap.points)} voxels, '
          f'{len(submap.dropped)} transient; pose shift '
          f'{math.hypot(refined[0] - pose[0], refined[1] - pose[1]):.3f} m '
          f'{math.degrees(refined[2] - pose[2]):+.2f} deg; added {report["added_share"]:.1%}, '
          f'see-through {report["see_through"]:.1%}; read {read_s:.0f} s, build '
          f'{build_s:.0f} s, compare {compare_s:.0f} s')
    for change in changes[:14]:
        print(f'  {change["kind"]:7s} {change["points"]:5d} low {change["low"]} high '
              f'{change["high"]} {",".join(change["known"]) or "-"}')


if __name__ == '__main__':
    main()
