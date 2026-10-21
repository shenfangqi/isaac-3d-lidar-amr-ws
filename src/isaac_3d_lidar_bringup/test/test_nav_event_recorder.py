"""Event logic, bag writing and launch wiring of the navigation black box."""

import json
import os
from pathlib import Path

import pytest
import yaml

from isaac_3d_lidar_bringup.nav_event_recorder import (
    event_dir_name,
    EventCapture,
    GoalStatusTrigger,
    LastValue,
    MessageRing,
    parse_topics,
    ProgressMonitor,
    prune_event_dirs,
    RosoutTrigger,
    write_event_bag,
)


PACKAGE_DIR = Path(__file__).resolve().parents[1]
S = 1_000_000_000


def test_ring_keeps_the_window_and_bounds_memory():
    ring = MessageRing(max_bytes=1000)
    for second in range(10):
        ring.append(second * S, '/a', b'x' * 10)
    ring.prune(5 * S)
    assert [item[0] for item in ring.window(0, 100 * S)] == [
        s * S for s in range(5, 10)]
    assert [item[0] for item in ring.window(6 * S, 7 * S)] == [6 * S, 7 * S]
    ring.append(11 * S, '/big', b'y' * 990)
    # The oldest messages go first once the byte limit is exceeded.
    assert ring.size_bytes <= 1000
    assert ring.window(0, 100 * S)[-1][1] == '/big'


def test_events_merge_into_one_window_capped_after_the_first():
    capture = EventCapture(pre_ns=30 * S, post_ns=10 * S, max_post_ns=60 * S)
    capture.trigger(100 * S, 'rosout_error', 'collision ahead')
    assert capture.poll(105 * S) is None
    capture.trigger(108 * S, 'rosout_match', 'Goal was canceled')
    assert capture.poll(117 * S) is None
    start, end, events = capture.poll(118 * S)
    assert (start, end) == (70 * S, 118 * S)
    assert [e['reason'] for e in events] == ['rosout_error', 'rosout_match']
    assert not capture.active
    # A stream of events cannot hold the window open forever.
    for second in range(200, 300, 5):
        capture.trigger(second * S, 'no_progress')
    start, end, _ = capture.poll(400 * S)
    assert (start, end) == (170 * S, 260 * S)


def test_open_window_can_be_forced_closed_at_shutdown():
    capture = EventCapture(pre_ns=30 * S, post_ns=10 * S, max_post_ns=60 * S)
    assert capture.poll(100 * S, force=True) is None
    capture.trigger(100 * S, 'rosout_error')
    assert capture.poll(103 * S) is None
    start, end, events = capture.poll(103 * S, force=True)
    assert (start, end) == (70 * S, 103 * S)
    assert len(events) == 1 and not capture.active


def test_last_value_gives_the_message_current_at_a_window_start():
    last = LastValue()
    assert last.before(10 * S) is None
    for second, data in ((1, b'plan1'), (50, b'plan2'), (80, b'plan3')):
        last.append(second * S, data)
    assert last.before(60 * S) == (50 * S, b'plan2')
    assert last.before(50 * S) == (1 * S, b'plan1')
    # Pruning to 70 s keeps plan2: it is still current at 70 s.
    last.prune(70 * S)
    assert last.before(75 * S) == (50 * S, b'plan2')
    assert last.before(100 * S) == (80 * S, b'plan3')
    last.prune(90 * S)
    assert last.before(100 * S) == (80 * S, b'plan3')


def test_rosout_trigger_levels_patterns_and_repeats():
    trigger = RosoutTrigger(
        level=40, patterns=['Goal was canceled'],
        ignore_patterns=['^ignored'], ignore_names=['nav_event_recorder'],
        repeat_s=30.0)
    assert trigger.check(0, 40, 'controller_server',
                         'RegulatedPurePursuitController detected collision '
                         'ahead!')[0] == 'rosout_error'
    # The same message again within 30 s is the same event.
    assert trigger.check(5 * S, 40, 'controller_server',
                         'RegulatedPurePursuitController detected collision '
                         'ahead!') is None
    assert trigger.check(31 * S, 40, 'controller_server',
                         'RegulatedPurePursuitController detected collision '
                         'ahead!') is not None
    reason, detail = trigger.check(
        0, 20, 'controller_server', 'Goal was canceled. Stopping the robot.')
    assert reason == 'rosout_match'
    assert detail.startswith('[controller_server] ')
    assert trigger.check(0, 30, 'amcl', 'some warning') is None
    assert trigger.check(0, 50, 'x', 'ignored fatal') is None
    assert trigger.check(0, 40, 'nav_event_recorder', 'own error') is None


def test_goal_status_reports_each_abort_once_and_tracks_activity():
    goals = GoalStatusTrigger()
    assert goals.update([(b'a', 2)]) == []
    assert goals.active
    assert goals.update([(b'a', 6)]) == [b'a']
    assert not goals.active
    assert goals.update([(b'a', 6), (b'b', 1)]) == []
    assert goals.active
    assert goals.update([(b'a', 6), (b'b', 4)]) == []


def test_progress_monitor_fires_once_per_stuck_place():
    monitor = ProgressMonitor(window_s=20.0, min_distance_m=0.05)
    fired = [monitor.update(t * S, 0.001 * t, 0.0, True)
             for t in range(0, 30)]
    assert fired.index(True) == 20
    assert fired.count(True) == 1
    # Moving away re-arms it; stopping again fires again.
    assert not monitor.update(31 * S, 0.5, 0.0, True)
    fired = [monitor.update(t * S, 0.5, 0.0, True) for t in range(32, 60)]
    assert fired.count(True) == 1
    # Driving at 0.1 m/s never fires, and no goal means no event.
    driving = ProgressMonitor(window_s=20.0, min_distance_m=0.05)
    assert not any(driving.update(t * S, 0.1 * t, 0.0, True)
                   for t in range(60))
    idle = ProgressMonitor(window_s=20.0, min_distance_m=0.05)
    assert not any(idle.update(t * S, 0.0, 0.0, False) for t in range(60))


def test_prune_keeps_the_newest_within_limits(tmp_path):
    for index in range(5):
        path = tmp_path / f'2026102{index}T000000Z_event'
        path.mkdir()
        (path / 'data').write_bytes(b'x' * 1000)
    keep = tmp_path / '20261029T000000Z_new'
    keep.mkdir()
    (keep / 'data').write_bytes(b'x' * 1000)
    removed = prune_event_dirs(str(tmp_path), max_total_mb=1.0,
                               max_events=3, keep=str(keep))
    assert removed == ['20261020T000000Z_event', '20261021T000000Z_event',
                       '20261022T000000Z_event']
    assert sorted(os.listdir(tmp_path)) == [
        '20261023T000000Z_event', '20261024T000000Z_event',
        '20261029T000000Z_new']


def test_event_dir_name_is_sortable_and_safe():
    assert event_dir_name(1792587330 * S, 'rosout_error') == (
        '20261021T125530Z_rosout_error')
    assert event_dir_name(0, 'a b/c') == '19700101T000000Z_a_b_c'


def test_config_topics_parse_and_cover_the_manual_recorder():
    config = yaml.safe_load((PACKAGE_DIR / 'config/nav2/'
                             'carbot_event_recorder_real.yaml').read_text())
    parameters = config['nav_event_recorder']['ros__parameters']
    topics = parse_topics(parameters['topics'])
    for topic in ('/cmd_vel', '/plan', '/scan', '/scan_near',
                  '/local_costmap/costmap_raw', '/odom', '/tf', '/tf_static',
                  '/novel_obstacle_keepoff', '/rosout',
                  '/navigate_to_pose/_action/status'):
        assert topic in topics
    # No raw LiDAR or IMU: the black box must stay cheap on the Jetson.
    assert '/livox/lidar' not in topics
    assert not any('imu' in topic for topic in topics)
    assert parameters['latched_topics'] == ['/tf_static']
    assert parameters['keep_last_topics'] == [
        '/plan', '/navigate_to_pose/_action/status']
    assert set(parameters['keep_last_topics']) <= set(topics)
    assert parameters['rosout_repeat_s'] <= 10.0
    with pytest.raises(ValueError):
        parse_topics(['/scan'])


def test_recorder_starts_with_navigation_by_default():
    source = (PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'
              ).read_text()
    block = source.split(
        "DeclareLaunchArgument(\n            'nav_event_recorder',", 1)[1]
    assert "default_value='true'," in block.split('),', 1)[0]
    assert "executable='nav_event_recorder'" in source
    assert 'condition=IfCondition(event_recorder),' in source
    assert 'event_recorder_node,\n    ])' in source
    setup = (PACKAGE_DIR / 'setup.py').read_text()
    assert 'isaac_3d_lidar_bringup.nav_event_recorder:main' in setup


def test_written_bag_reads_back(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import deserialize_message, serialize_message
    from std_msgs.msg import String

    types = {'/a': 'std_msgs/msg/String', '/tf_static': 'std_msgs/msg/String'}
    messages = [(t * S, '/a', serialize_message(String(data=str(t))))
                for t in (12, 10, 11)]
    latched = [('/tf_static', serialize_message(String(data='static')))]
    summary = write_event_bag(str(tmp_path), messages, latched, types,
                              9 * S, {'events': [{'reason': 'test'}]})
    assert summary['message_counts'] == {'/a': 3, '/tf_static': 1}
    assert json.loads((tmp_path / 'event.json').read_text())[
        'events'] == [{'reason': 'test'}]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path / 'bag'),
                                          storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    read = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        read.append((stamp, topic, deserialize_message(data, String).data))
    assert read == [(9 * S, '/tf_static', 'static'), (10 * S, '/a', '10'),
                    (11 * S, '/a', '11'), (12 * S, '/a', '12')]
