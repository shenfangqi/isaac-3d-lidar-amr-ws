import random
import struct

from sensor_msgs.msg import PointCloud2, PointField

from isaac_3d_lidar_bringup.pointcloud_evidence_degrader import (
    degrade_xyz_data,
)


def cloud(values):
    message = PointCloud2()
    message.height = 1
    message.width = len(values)
    message.point_step = 12
    message.row_step = 12 * len(values)
    message.fields = [
        PointField(name=name, offset=index * 4, datatype=7, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    message.data = b"".join(struct.pack("<fff", *value) for value in values)
    return message


def test_full_dropout_sets_xyz_to_nan():
    message = cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    result = degrade_xyz_data(message, 1.0, 0.0, random.Random(1))
    assert all(value != value for value in struct.unpack("<ffffff", result))


def test_zero_degradation_preserves_cloud():
    message = cloud([(1.0, 2.0, 3.0)])
    result = degrade_xyz_data(message, 0.0, 0.0, random.Random(1))
    assert result == bytes(message.data)
