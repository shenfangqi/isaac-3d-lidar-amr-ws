import math

import pytest

from isaac_3d_lidar_bringup.fast_lio_base_adapter import compose_pose
from isaac_3d_lidar_bringup.fast_lio_base_adapter import rotate_vector
from isaac_3d_lidar_bringup.fast_lio_base_adapter import yaw_quaternion


def test_compose_pose_rotates_child_translation():
    yaw_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    position, orientation = compose_pose(
        (1.0, 2.0, 0.0), yaw_90, (1.0, 0.0, 0.0), (0, 0, 0, 1)
    )
    assert position == pytest.approx((1.0, 3.0, 0.0))
    assert orientation == pytest.approx(yaw_90)


def test_yaw_projection_removes_roll_and_pitch():
    roll_30 = (math.sin(math.pi / 12), 0.0, 0.0, math.cos(math.pi / 12))
    projected = yaw_quaternion(roll_30)
    assert projected == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_rotate_vector_preserves_length():
    yaw_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    rotated = rotate_vector(yaw_90, (2.0, 0.0, 0.0))
    assert rotated == pytest.approx((0.0, 2.0, 0.0))
