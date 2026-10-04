"""ROS marker construction tests for the read-only validation preview."""

import math

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker

from carbot_nav_recovery.swept_footprint import Pose2D
from carbot_nav_recovery.validation_visualizer import (
    _blocker_marker,
    _footprint_marker,
    _sweep_marker,
    _transform_footprint,
    _yaw,
)


def test_footprint_pose_transform_uses_map_pose():
    polygon = ((0.2, 0.1), (0.2, -0.1), (-0.2, -0.1), (-0.2, 0.1))
    transformed = _transform_footprint(
        polygon, Pose2D(1.0, 2.0, math.pi / 2.0))
    assert transformed[0][0] == pytest.approx(0.9)
    assert transformed[0][1] == pytest.approx(2.2)


def test_candidate_markers_show_sweep_and_blocker_location():
    footprint = ((0.2, 0.1), (0.2, -0.1), (-0.2, -0.1), (-0.2, 0.1))
    pose = Pose2D(1.0, 2.0, 0.0)
    stamp = Time()
    outline = _footprint_marker(
        'map', stamp, footprint, pose, 1, (0.2, 0.8, 1.0))
    sweep = _sweep_marker(
        'map', stamp, footprint, pose, math.pi / 2.0, 2, (1.0, 0.0, 0.0))
    blocker = _blocker_marker('map', stamp, (1.2, 2.3), 3)
    assert outline.type == Marker.LINE_STRIP
    assert len(outline.points) == len(footprint) + 1
    assert sweep.type == Marker.LINE_LIST
    assert len(sweep.points) >= len(footprint) * 2 * 2
    assert blocker.type == Marker.SPHERE
    assert blocker.pose.position.x == pytest.approx(1.2)
    assert blocker.pose.position.y == pytest.approx(2.3)


def test_quaternion_yaw_conversion():
    quaternion = Quaternion()
    quaternion.z = math.sin(math.pi / 8.0)
    quaternion.w = math.cos(math.pi / 8.0)
    assert _yaw(quaternion) == pytest.approx(math.pi / 4.0)
