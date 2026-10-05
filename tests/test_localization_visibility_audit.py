"""Issue #13 PR0: read-only visibility audit and baseline manifest."""

import json
import math
from pathlib import Path

import pytest
import yaml

from scripts.audit_mid360_localization_visibility import (
    analytic_sweep_coverage,
    classify_cloud_points,
    classify_scan_sweep,
    footprint_boundary_distance,
    load_fast_lio_preprocess,
    load_robot_geometry,
    observable_band,
    OBSERVED_FREE,
    OBSERVED_OCCUPIED,
    rotation_envelope_radius,
    SELF,
    UNKNOWN,
)
from scripts.build_localization_baseline_manifest import build_manifest


SQUARE = [(0.1, 0.1), (0.1, -0.1), (-0.1, -0.1), (-0.1, 0.1)]


def test_footprint_boundary_distance_along_axes_and_diagonal():
    assert footprint_boundary_distance(SQUARE, 0.0) == pytest.approx(0.1)
    assert footprint_boundary_distance(SQUARE, math.pi / 2) == (
        pytest.approx(0.1))
    assert footprint_boundary_distance(SQUARE, math.pi / 4) == (
        pytest.approx(0.1 * math.sqrt(2.0)))


def test_rotation_envelope_uses_farthest_vertex_plus_padding():
    assert rotation_envelope_radius(SQUARE, 0.05) == pytest.approx(
        0.1 * math.sqrt(2.0) + 0.05)
    with pytest.raises(ValueError):
        rotation_envelope_radius(SQUARE, -0.01)


def test_min_range_splits_the_visible_height_band():
    intervals = observable_band(0.2, 0.3, (-60.0, 60.0), 0.5)
    hidden = math.sqrt(0.5 ** 2 - 0.3 ** 2)
    assert intervals == [
        (pytest.approx(0.2 - 0.3 * math.tan(math.radians(60.0))),
         pytest.approx(0.2 - hidden)),
        (pytest.approx(0.2 + hidden),
         pytest.approx(0.2 + 0.3 * math.tan(math.radians(60.0)))),
    ]


def test_height_filter_can_remove_every_visible_height():
    assert observable_band(0.2, 1.0, (-7.0, 52.0), 0.1, 2.0, 3.0) == []


def test_scan_beam_beyond_sweep_is_free_only_when_range_min_allows():
    ranges = [1.0]
    counts, _ = classify_scan_sweep(ranges, 0.0, 0.01, 0.05, 20.0,
                                    SQUARE, 0.2)
    assert counts[OBSERVED_FREE] == 1
    counts, reasons = classify_scan_sweep(ranges, 0.0, 0.01, 0.5, 20.0,
                                          SQUARE, 0.2)
    assert counts[OBSERVED_FREE] == 0
    assert counts[UNKNOWN] == 1
    assert reasons['range_min_hides_sweep'] == 1


def test_infinite_and_invalid_beams_never_clear_unknown():
    counts, reasons = classify_scan_sweep(
        [math.inf, math.nan, 0.01, 30.0], 0.0, 0.01, 0.05, 20.0,
        SQUARE, 0.2)
    assert counts[UNKNOWN] == 4
    assert counts[OBSERVED_FREE] == 0
    assert reasons['nonfinite_or_out_of_range'] == 4


def test_scan_returns_inside_body_and_sweep_are_separated():
    counts, _ = classify_scan_sweep([0.08, 0.15], 0.0, 0.01, 0.05, 20.0,
                                    SQUARE, 0.2)
    assert counts[SELF] == 1
    assert counts[OBSERVED_OCCUPIED] == 1


def test_sparse_cloud_never_reports_free_space():
    counts = classify_cloud_points(
        [(0.0, 0.0, 0.1), (0.15, 0.0, 0.1), (0.15, 0.0, 1.0),
         (2.0, 0.0, 0.1)], SQUARE, 0.2, (0.0, 0.3))
    assert counts == {OBSERVED_OCCUPIED: 1, SELF: 1, 'outside_sweep': 2}
    assert OBSERVED_FREE not in counts


def test_repository_geometry_cannot_prove_rotation_sweep_free():
    geometry = load_robot_geometry()
    fast_lio = load_fast_lio_preprocess()
    report = analytic_sweep_coverage(geometry, fast_lio, padding_m=0.05)
    channels = report['channels']
    # Derived channels drop everything within 0.5 m: the whole sweep.
    for name in ('/scan', '/scan_localization',
                 'fast_lio_cloud_registered_body'):
        assert not channels[name]['can_prove_sweep_free']
        assert all(row['unobservable_fraction_of_band'] == 1.0
                   for row in channels[name]['radii'])
    # The raw MID-360 sees only the top of the band near the body.
    raw = channels['raw_mid360']
    assert not raw['can_prove_sweep_free']
    assert all(0.0 < row['unobservable_fraction_of_band'] < 1.0
               for row in raw['radii'])


def test_fast_lio_preprocess_records_custom_message_expectation():
    fast_lio = load_fast_lio_preprocess()
    assert fast_lio['lid_topic'] == '/livox/lidar'
    assert fast_lio['expected_message_type'] == (
        'livox_ros_driver2/msg/CustomMsg')
    assert fast_lio['blind_m'] == pytest.approx(0.5)


def _write_bag(directory, topics, sidecar=True):
    directory.mkdir(parents=True)
    (directory / 'bag_0.db3').write_bytes(b'payload')
    metadata = {'rosbag2_bagfile_information': {
        'storage_identifier': 'sqlite3',
        'starting_time': {'nanoseconds_since_epoch': 1},
        'duration': {'nanoseconds': 2_000_000_000},
        'message_count': sum(count for _, _, count in topics),
        'relative_file_paths': ['bag_0.db3'],
        'topics_with_message_count': [
            {'topic_metadata': {'name': name, 'type': type_name},
             'message_count': count}
            for name, type_name, count in topics],
    }}
    (directory / 'metadata.yaml').write_text(yaml.safe_dump(metadata))
    if sidecar:
        (directory / 'notes.txt').write_text('x')


def test_manifest_reports_channel_gaps(tmp_path):
    root = tmp_path
    _write_bag(root / 'data/static', [
        ('/scan', 'sensor_msgs/msg/LaserScan', 5),
        ('/scan_localization', 'sensor_msgs/msg/LaserScan', 5),
        ('/odom', 'nav_msgs/msg/Odometry', 5),
        ('/map', 'nav_msgs/msg/OccupancyGrid', 1),
        ('/tf', 'tf2_msgs/msg/TFMessage', 5),
        ('/tf_static', 'tf2_msgs/msg/TFMessage', 1),
        ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 0),
    ])
    manifest = build_manifest(root, root / 'data', hash_storage=True)
    bag = manifest['bags'][0]
    assert bag['path'] == 'data/static'
    assert bag['usable_for']['stationary_search_replay']
    # A declared topic with zero messages is not evidence.
    assert not bag['channels']['raw_lidar']
    assert 'raw_near_field_audit' in manifest['gaps']
    assert bag['files']['bag_0.db3']['sha256'] is not None
    assert 'notes.txt' in bag['sidecar_sha256']
    json.dumps(manifest)


def test_committed_manifest_matches_schema():
    path = (Path(__file__).resolve().parents[1]
            / 'docs/evidence/issue13_pr0_baseline_bag_manifest.json')
    manifest = json.loads(path.read_text('utf-8'))
    assert manifest['schema_version'] == 1
    assert manifest['kind'] == 'issue13_pr0_baseline_bags'
    assert set(manifest['coverage']) >= {
        'raw_near_field_audit', 'stationary_search_replay'}
