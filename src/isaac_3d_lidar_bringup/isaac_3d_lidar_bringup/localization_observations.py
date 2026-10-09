"""
Stopped-scan keyframes with source-time transforms (Issue #13).

Pure Python and picklable, so keyframes can be sent to the search worker
process.  A keyframe is only created when every transform is available at
the scan's own source stamp; the newest transform is never substituted.
"""

from dataclasses import dataclass
import math

from isaac_3d_lidar_bringup.localization_contracts import (
    ContractError,
    FrameRole,
    Keyframe,
    RejectReason,
)


@dataclass(frozen=True)
class ScanSnapshot:
    """Immutable copy of the LaserScan fields used for map matching."""

    frame_id: str
    stamp_ns: int
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: tuple

    def __post_init__(self):
        values = (self.angle_min, self.angle_increment, self.range_min,
                  self.range_max)
        if not all(isinstance(v, (int, float)) and math.isfinite(v)
                   for v in values):
            raise ContractError('scan geometry must be finite')
        if self.angle_increment <= 0.0 or not (
                0.0 <= self.range_min < self.range_max):
            raise ContractError('unsupported scan geometry')
        if not self.frame_id or self.stamp_ns <= 0 or not self.ranges:
            raise ContractError('scan needs a frame, a stamp and ranges')


def snapshot_scan(message):
    """Copy a sensor_msgs/LaserScan into a picklable snapshot."""
    stamp = message.header.stamp
    return ScanSnapshot(
        frame_id=message.header.frame_id,
        stamp_ns=stamp.sec * 1_000_000_000 + stamp.nanosec,
        angle_min=float(message.angle_min),
        angle_increment=float(message.angle_increment),
        range_min=float(message.range_min),
        range_max=float(message.range_max),
        ranges=tuple(float(value) for value in message.ranges),
    )


@dataclass(frozen=True)
class Reject:
    """A refused keyframe or frame collection, with the contract reason."""

    reason: RejectReason
    detail: str


def make_keyframe(scan, tf_lookup, session, view_id, role, keyframe_id,
                  receipt_mono, odom_frame='odom',
                  base_frame='base_footprint'):
    """
    Return a Keyframe, or Reject when a source-time transform is missing.

    ``tf_lookup(target, source, stamp_ns)`` must return an SE2 for exactly
    that stamp or raise LookupError.
    """
    try:
        odom_base = tf_lookup(odom_frame, base_frame, scan.stamp_ns)
        base_scan = tf_lookup(base_frame, scan.frame_id, scan.stamp_ns)
    except LookupError as error:
        return Reject(RejectReason.TF_AT_SOURCE_MISSING, str(error))
    return Keyframe(
        id=keyframe_id, session=session, stamp_ns=scan.stamp_ns,
        view_id=view_id, scan=scan, T_odom_base=odom_base,
        T_base_scan=base_scan, receipt_mono=receipt_mono, role=role)


def _pose_step(first, second):
    return (math.hypot(first.x - second.x, first.y - second.y),
            abs(math.atan2(math.sin(first.yaw - second.yaw),
                           math.cos(first.yaw - second.yaw))))


def collect_keyframes(buffer, role, max_views, max_age_s, now_mono, session,
                      max_view_translation_m=0.05,
                      max_view_yaw_rad=math.radians(3.0)):
    """
    Select one decision's frames of a single role.

    Frames from another session, of another role or older than
    ``max_age_s`` are excluded.  Frames inside one view must agree on the
    odom pose; a disagreement means the robot was moved or odometry jumped.
    When more than ``max_views`` views exist they are thinned by yaw
    coverage instead of accumulating without bound.
    """
    role = FrameRole(role)
    if max_views < 1:
        raise ContractError('max_views must be positive')
    frames = [
        frame for frame in buffer
        if frame.session == session and frame.role == role
        and 0.0 <= now_mono - frame.receipt_mono <= max_age_s
    ]
    if not frames:
        return Reject(RejectReason.SENSOR_STALE,
                      f'no fresh {role.value} frames')
    stamps = [frame.stamp_ns for frame in frames]
    if len(set(stamps)) != len(stamps):
        return Reject(RejectReason.SENSOR_STALE, 'duplicate scan stamps')
    views = {}
    for frame in sorted(frames, key=lambda item: item.stamp_ns):
        views.setdefault(frame.view_id, []).append(frame)
    for view_id, members in views.items():
        for frame in members[1:]:
            translation, yaw = _pose_step(frame.T_odom_base,
                                          members[0].T_odom_base)
            if translation > max_view_translation_m or yaw > max_view_yaw_rad:
                return Reject(RejectReason.ODOM_JUMP,
                              f'view {view_id} moved by {translation:.3f} m '
                              f'/ {math.degrees(yaw):.1f} deg')
    ordered = sorted(views, key=lambda v: views[v][0].T_odom_base.yaw)
    if len(ordered) > max_views:
        poses = {v: views[v][0].T_odom_base for v in views}
        spread = max(math.hypot(poses[a].x - poses[b].x, poses[a].y - poses[b].y)
                     for a in views for b in views)
        if spread <= 0.10:
            # Evenly spaced by odom yaw: keep viewpoint diversity, not recency.
            step = len(ordered) / max_views
            ordered = [ordered[int(i * step)] for i in range(max_views)]
        else:
            # After translation, positions differ too: farthest-point choice
            # over heading and position (0.5 m ~ 1 rad), keeping the first
            # (reference) view so the reference keyframe never changes.
            def distance(a, b):
                return (abs(math.atan2(math.sin(poses[a].yaw - poses[b].yaw),
                                       math.cos(poses[a].yaw - poses[b].yaw)))
                        + math.hypot(poses[a].x - poses[b].x,
                                     poses[a].y - poses[b].y) / 0.5)
            chosen = [min(views)]
            while len(chosen) < max_views:
                chosen.append(max((v for v in views if v not in chosen),
                                  key=lambda v: (min(distance(v, c) for c in chosen), -v)))
            ordered = chosen
    selected = []
    for view_id in sorted(ordered):
        selected.extend(views[view_id])
    return tuple(selected)
