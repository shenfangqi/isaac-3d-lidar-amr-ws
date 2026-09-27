"""Unit tests for manual saved-map alignment."""

import math

import pytest

from isaac_3d_lidar_bringup.manual_map_localizer import (
    map_to_odom_from_poses,
)


def _compose(transform, odom_base):
    x, y, yaw = transform
    ox, oy, oyaw = odom_base
    return (
        x + math.cos(yaw) * ox - math.sin(yaw) * oy,
        y + math.sin(yaw) * ox + math.cos(yaw) * oy,
        math.atan2(math.sin(yaw + oyaw), math.cos(yaw + oyaw)),
    )


@pytest.mark.parametrize(
    'map_base,odom_base',
    [
        ((1.31, 0.132, 1.172), (-2.253, 1.093, 0.373)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((-1.0, 2.0, -2.8), (0.4, -0.3, 2.9)),
    ],
)
def test_alignment_reconstructs_requested_map_pose(map_base, odom_base):
    transform = map_to_odom_from_poses(map_base, odom_base)
    reconstructed = _compose(transform, odom_base)
    assert reconstructed[:2] == pytest.approx(map_base[:2])
    assert reconstructed[2] == pytest.approx(map_base[2])
