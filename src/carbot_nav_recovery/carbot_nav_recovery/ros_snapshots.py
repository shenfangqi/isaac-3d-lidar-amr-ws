"""Strict ROS message adapters for read-only geometry snapshots."""

import math

from .swept_footprint import CostmapSnapshot, ObservedFreeSpaceSnapshot


def _shape(width, height, resolution, origin, data):
    if (width <= 0 or height <= 0 or width * height > 1000000
            or len(data) != width * height):
        raise ValueError('invalid grid dimensions or data length')
    q = origin.orientation
    values = (resolution, origin.position.x, origin.position.y,
              q.x, q.y, q.z, q.w)
    if (not all(math.isfinite(v) for v in values) or resolution <= 0
            or abs(q.x) > 1e-5 or abs(q.y) > 1e-5 or abs(q.z) > 1e-5
            or abs(q.w * q.w - 1) > 1e-5):
        raise ValueError('grid requires finite, unrotated planar origin')


def _arguments(header, width, height, resolution, origin, received):
    return dict(
        width=width, height=height, resolution=resolution,
        origin_x=origin.position.x, origin_y=origin.position.y,
        frame_id=header.frame_id,
        stamp_sec=header.stamp.sec + header.stamp.nanosec * 1e-9,
        received_monotonic_sec=received)


def raw_costmap(message, received):
    metadata = message.metadata
    _shape(metadata.size_x, metadata.size_y, metadata.resolution,
           metadata.origin, message.data)
    return CostmapSnapshot(
        **_arguments(message.header, metadata.size_x, metadata.size_y,
                     metadata.resolution, metadata.origin, received),
        costs=tuple(message.data))


def occupancy_snapshot(message, received, *, visibility=False):
    """Use only explicit zero as free; never infer sensor visibility from map.

    visibility=True is for a dedicated recent sensor-cleared mask topic only.
    It is a diagnostic input, not proof of the producer's sensor provenance.
    """
    info = message.info
    _shape(info.width, info.height, info.resolution, info.origin, message.data)
    if any(value < -1 or value > 100 for value in message.data):
        raise ValueError('occupancy values outside [-1, 100]')
    arguments = _arguments(message.header, info.width, info.height,
                           info.resolution, info.origin, received)
    if visibility:
        return ObservedFreeSpaceSnapshot(
            **arguments, observed_free=tuple(v == 0 for v in message.data))
    return CostmapSnapshot(
        **arguments, costs=tuple(0 if v == 0 else 255 if v == -1 else 254
                                 for v in message.data))
