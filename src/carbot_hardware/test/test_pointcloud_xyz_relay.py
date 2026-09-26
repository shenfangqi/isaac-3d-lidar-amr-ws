import struct

from sensor_msgs.msg import PointCloud2, PointField

from carbot_hardware.pointcloud_xyz_relay import (
    LIVOX_FULL_FIELD_NAMES,
    compact_xyz_cloud,
    missing_pointcloud_fields,
    should_publish_frame,
)


def _field(name, offset):
    return PointField(
        name=name,
        offset=offset,
        datatype=PointField.FLOAT32,
        count=1,
    )


def test_compact_xyz_cloud_strips_extra_fields():
    message = PointCloud2()
    message.header.frame_id = 'livox_frame'
    message.height = 1
    message.width = 2
    message.fields = [
        _field('x', 0),
        _field('y', 4),
        _field('z', 8),
        _field('intensity', 12),
    ]
    message.point_step = 16
    message.row_step = 32
    message.is_dense = True
    message.data = struct.pack('<ffffffff', 1, 2, 3, 9, 4, 5, 6, 8)
    original_fields = list(message.fields)
    original_data = bytes(message.data)

    output = compact_xyz_cloud(message)

    assert output.header.frame_id == 'livox_frame'
    assert output.width == 2
    assert output.height == 1
    assert output.point_step == 12
    assert output.row_step == 24
    assert [field.name for field in output.fields] == ['x', 'y', 'z']
    assert struct.unpack('<ffffff', output.data) == (1, 2, 3, 4, 5, 6)
    assert output.is_dense
    assert message.fields == original_fields
    assert bytes(message.data) == original_data


def test_full_livox_field_contract_is_checked_before_deriving_xyz():
    message = PointCloud2()
    message.fields = [
        PointField(name=name, offset=index, datatype=PointField.UINT8, count=1)
        for index, name in enumerate(LIVOX_FULL_FIELD_NAMES)
    ]

    assert missing_pointcloud_fields(message) == ()
    message.fields = [
        field for field in message.fields if field.name not in {'tag', 'line'}
    ]
    assert missing_pointcloud_fields(message) == ('tag', 'line')


def test_compact_xyz_cloud_applies_stride_and_phase():
    message = PointCloud2()
    message.height = 1
    message.width = 4
    message.fields = [_field('x', 0), _field('y', 4), _field('z', 8)]
    message.point_step = 12
    message.row_step = 48
    message.data = struct.pack(
        '<ffffffffffff',
        1, 2, 3,
        4, 5, 6,
        7, 8, 9,
        10, 11, 12,
    )

    output = compact_xyz_cloud(message, point_stride=2, phase=1)

    assert output.width == 2
    assert output.row_step == 24
    assert struct.unpack('<ffffff', output.data) == (4, 5, 6, 10, 11, 12)


def test_should_publish_frame_reduces_output_cadence():
    assert [
        index for index in range(12) if should_publish_frame(index, 5)
    ] == [0, 5, 10]


def test_should_publish_frame_rejects_invalid_arguments():
    import pytest

    with pytest.raises(ValueError, match='frame_index'):
        should_publish_frame(-1, 1)
    with pytest.raises(ValueError, match='frame_stride'):
        should_publish_frame(0, 0)
