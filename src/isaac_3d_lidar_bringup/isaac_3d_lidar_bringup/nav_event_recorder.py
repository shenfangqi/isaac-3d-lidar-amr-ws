"""
Always-on navigation event recorder (black box).

Robot sessions kept losing the moment that mattered: the operator sent a goal
before the recorder started, or the recorder had already been stopped when
RPP refused to move (2026-10-21).  Starting a recorder while the robot drives
is not allowed either: three FAST-LIO pose-jump faults on 2026-10-19 followed
a new ROS process by 3-6 s.  This node therefore starts with the navigation
launch and never records continuously.  It keeps the last ``pre_s`` seconds of
the configured topics in memory, as serialized messages (no deserialization),
and writes a rosbag2 only when an event fires:

* a /rosout message at or above ``rosout_trigger_level`` (ERROR by default,
  e.g. "RegulatedPurePursuitController detected collision ahead!"), or one
  containing one of ``rosout_trigger_patterns`` (by default the controller's
  "Goal was canceled", i.e. a route-check stop or a goal change);
* a NavigateToPose goal that ends ABORTED;
* an active goal while the robot moved less than ``no_progress_distance_m``
  in ``no_progress_window_s``;
* a non-empty /localization/fault_reason, or /localization/emergency_stop
  turning true.

Events within ``post_s`` of each other share one bag; the bag covers
``pre_s`` before the first event to ``post_s`` after the last one, at most
``max_post_s`` after the first.  Bags go to ``output_dir/<UTC time>_<reason>``
with an ``event.json`` listing the events, and the oldest are deleted beyond
``max_total_mb`` or ``max_events``.  Writing runs on a separate thread, so the
subscriptions keep buffering while a bag is written.
"""

import collections
import json
import math
import os
import re
import shutil
import threading
import time

GOAL_ACTIVE = (1, 2, 3)   # ACCEPTED, EXECUTING, CANCELING
GOAL_ABORTED = 6
ROSOUT_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR',
                 50: 'FATAL'}


class MessageRing:
    """Serialized messages of the last ``keep_ns``, oldest first."""

    def __init__(self, max_bytes):
        self._items = collections.deque()
        self._bytes = 0
        self._max_bytes = max_bytes
        self._lock = threading.Lock()

    def append(self, stamp_ns, topic, data):
        with self._lock:
            self._items.append((stamp_ns, topic, data))
            self._bytes += len(data)
            while self._bytes > self._max_bytes and self._items:
                self._bytes -= len(self._items.popleft()[2])

    def prune(self, oldest_ns):
        with self._lock:
            while self._items and self._items[0][0] < oldest_ns:
                self._bytes -= len(self._items.popleft()[2])

    def window(self, start_ns, end_ns):
        with self._lock:
            return [item for item in self._items
                    if start_ns <= item[0] <= end_ns]

    @property
    def size_bytes(self):
        return self._bytes


class EventCapture:
    """Merge events into capture windows [first - pre, last + post]."""

    def __init__(self, pre_ns, post_ns, max_post_ns):
        self.pre_ns = pre_ns
        self.post_ns = post_ns
        self.max_post_ns = max_post_ns
        self._events = []
        self._end_ns = None

    @property
    def active(self):
        return bool(self._events)

    def trigger(self, stamp_ns, reason, detail=''):
        if not self._events:
            self._end_ns = stamp_ns + self.post_ns
        else:
            first = self._events[0]['stamp_ns']
            self._end_ns = min(max(self._end_ns, stamp_ns + self.post_ns),
                               first + self.max_post_ns)
        self._events.append(
            {'stamp_ns': stamp_ns, 'reason': reason, 'detail': detail})

    def poll(self, now_ns):
        """Return (start_ns, end_ns, events) once the window has closed."""
        if not self._events or now_ns < self._end_ns:
            return None
        events = self._events
        window = (events[0]['stamp_ns'] - self.pre_ns, self._end_ns, events)
        self._events = []
        self._end_ns = None
        return window


class RosoutTrigger:
    """Pick /rosout messages that mark an event; rate-limit repeats."""

    def __init__(self, level, patterns, ignore_patterns, ignore_names,
                 repeat_s):
        self._level = level
        self._patterns = [p for p in patterns if p]
        self._ignore = [re.compile(p) for p in ignore_patterns if p]
        self._ignore_names = set(ignore_names)
        self._repeat_ns = int(repeat_s * 1e9)
        self._last = {}

    def check(self, stamp_ns, level, name, text):
        if name in self._ignore_names:
            return None
        if any(p.search(text) for p in self._ignore):
            return None
        if level >= self._level:
            reason = 'rosout_' + ROSOUT_LEVELS.get(level, str(level)).lower()
        elif any(p in text for p in self._patterns):
            reason = 'rosout_match'
        else:
            return None
        # The same node repeating the same message is one event.
        key = (name, text[:80])
        last = self._last.get(key)
        if last is not None and stamp_ns - last < self._repeat_ns:
            return None
        self._last[key] = stamp_ns
        return reason, f'[{name}] {text}'


class GoalStatusTrigger:
    """Report goals that newly reached ABORTED; track whether one is active."""

    def __init__(self):
        self._seen = {}
        self.active = False

    def update(self, statuses):
        """``statuses`` is a list of (goal_id_bytes, status)."""
        aborted = []
        current = {}
        for goal_id, status in statuses:
            current[goal_id] = status
            if (status == GOAL_ABORTED
                    and self._seen.get(goal_id) != GOAL_ABORTED):
                aborted.append(goal_id)
        self._seen = current
        self.active = any(s in GOAL_ACTIVE for s in current.values())
        return aborted


class ProgressMonitor:
    """Fire once when an active goal makes no progress for a window."""

    def __init__(self, window_s, min_distance_m):
        self._window_ns = int(window_s * 1e9)
        self._min_distance = min_distance_m
        self._history = collections.deque()
        self._stuck_at = None

    def update(self, stamp_ns, x, y, goal_active):
        if not goal_active:
            self._history.clear()
            self._stuck_at = None
            return False
        self._history.append((stamp_ns, x, y))
        while (len(self._history) > 1
               and self._history[1][0] <= stamp_ns - self._window_ns):
            self._history.popleft()
        if self._stuck_at is not None:
            # Re-arm only after the robot has moved away from where it stuck.
            if math.hypot(x - self._stuck_at[0],
                          y - self._stuck_at[1]) >= self._min_distance:
                self._stuck_at = None
                self._history.clear()
                self._history.append((stamp_ns, x, y))
            return False
        oldest = self._history[0]
        if stamp_ns - oldest[0] < self._window_ns:
            return False
        moved = max(math.hypot(px - x, py - y) for _, px, py in self._history)
        if moved < self._min_distance:
            self._stuck_at = (x, y)
            return True
        return False


def event_dir_name(stamp_ns, reason):
    stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime(stamp_ns / 1e9))
    safe = re.sub(r'[^A-Za-z0-9_]+', '_', reason)[:40]
    return f'{stamp}_{safe}'


def prune_event_dirs(output_dir, max_total_mb, max_events, keep=None):
    """Delete the oldest event directories beyond the limits."""
    entries = []
    for name in os.listdir(output_dir):
        path = os.path.join(output_dir, name)
        if not os.path.isdir(path) or path == keep:
            continue
        size = 0
        for root, _, files in os.walk(path):
            for file_name in files:
                try:
                    size += os.path.getsize(os.path.join(root, file_name))
                except OSError:
                    pass
        entries.append((name, path, size))
    entries.sort()
    keep_size = 0
    if keep is not None and os.path.isdir(keep):
        for root, _, files in os.walk(keep):
            keep_size += sum(os.path.getsize(os.path.join(root, f))
                             for f in files)
    total = keep_size + sum(size for _, _, size in entries)
    count = len(entries) + (1 if keep is not None else 0)
    removed = []
    while entries and (total > max_total_mb * 1e6 or count > max_events):
        name, path, size = entries.pop(0)
        shutil.rmtree(path, ignore_errors=True)
        total -= size
        count -= 1
        removed.append(name)
    return removed


def write_event_bag(path, messages, latched, topic_types, start_ns, summary):
    """Write ``latched`` (at ``start_ns``) and ``messages`` to a rosbag2."""
    import rosbag2_py

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=os.path.join(path, 'bag'),
                                  storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'))
    created = set()
    counts = collections.Counter()
    items = [(start_ns, topic, data) for topic, data in latched]
    items += sorted(messages, key=lambda item: item[0])
    for stamp_ns, topic, data in items:
        if topic not in created:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=topic_types[topic],
                serialization_format='cdr'))
            created.add(topic)
        writer.write(topic, data, stamp_ns)
        counts[topic] += 1
    del writer
    summary = dict(summary, message_counts=dict(sorted(counts.items())))
    with open(os.path.join(path, 'event.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
        f.write('\n')
    return summary


def parse_topics(entries):
    """Parse ``topic:pkg/msg/Type`` strings into an ordered dict."""
    topics = collections.OrderedDict()
    for entry in entries:
        topic, _, msg_type = entry.partition(':')
        topic, msg_type = topic.strip(), msg_type.strip()
        if not topic.startswith('/') or msg_type.count('/') != 2:
            raise ValueError(f'bad topic entry {entry!r}; '
                             'expected /topic:pkg/msg/Type')
        topics[topic] = msg_type
    return topics


def main(args=None):
    import queue

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from rclpy.qos import ReliabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    class NavEventRecorder(Node):
        """Buffer navigation topics and write a bag around each event."""

        def __init__(self):
            super().__init__('nav_event_recorder')
            self.declare_parameter(
                'output_dir', '/workspaces/isaac_ros-dev/nav_events')
            self.declare_parameter('topics', [''])
            self.declare_parameter('latched_topics', ['/tf_static'])
            self.declare_parameter('reliable_topics', ['/rosout'])
            self.declare_parameter('pre_s', 30.0)
            self.declare_parameter('post_s', 10.0)
            self.declare_parameter('max_post_s', 60.0)
            self.declare_parameter('max_buffer_mb', 200.0)
            self.declare_parameter('max_total_mb', 3000.0)
            self.declare_parameter('max_events', 200)
            self.declare_parameter('rosout_topic', '/rosout')
            self.declare_parameter('rosout_trigger_level', 40)
            self.declare_parameter(
                'rosout_trigger_patterns', ['Goal was canceled'])
            self.declare_parameter('rosout_ignore_patterns', [''])
            self.declare_parameter('rosout_repeat_s', 30.0)
            self.declare_parameter(
                'goal_status_topic', '/navigate_to_pose/_action/status')
            self.declare_parameter('odom_topic', '/odom')
            self.declare_parameter('no_progress_window_s', 20.0)
            self.declare_parameter('no_progress_distance_m', 0.05)
            self.declare_parameter(
                'fault_reason_topic', '/localization/fault_reason')
            self.declare_parameter(
                'emergency_topic', '/localization/emergency_stop')

            value = self.get_parameter
            self._output_dir = value('output_dir').value
            os.makedirs(self._output_dir, exist_ok=True)
            self._topics = parse_topics(
                [t for t in value('topics').value if t])
            self._latched_names = set(value('latched_topics').value)
            reliable = set(value('reliable_topics').value)
            self._max_total_mb = value('max_total_mb').value
            self._max_events = int(value('max_events').value)
            pre_ns = int(value('pre_s').value * 1e9)
            self._ring = MessageRing(int(value('max_buffer_mb').value * 1e6))
            self._capture = EventCapture(
                pre_ns, int(value('post_s').value * 1e9),
                int(value('max_post_s').value * 1e9))
            self._keep_ns = pre_ns + self._capture.max_post_ns
            self._latched = collections.OrderedDict()
            self._rosout = RosoutTrigger(
                int(value('rosout_trigger_level').value),
                value('rosout_trigger_patterns').value,
                value('rosout_ignore_patterns').value,
                [self.get_name()], value('rosout_repeat_s').value)
            self._goals = GoalStatusTrigger()
            self._progress = ProgressMonitor(
                value('no_progress_window_s').value,
                value('no_progress_distance_m').value)
            self._last_pose = None
            self._last_fault = ''
            self._emergency = False
            self._jobs = queue.Queue()
            self._writer = threading.Thread(target=self._write_loop,
                                            daemon=True)
            self._writer.start()

            # Messages that also drive triggers are decoded in the callback.
            decoders = {
                value('rosout_topic').value: self._on_rosout,
                value('goal_status_topic').value: self._on_goal_status,
                value('odom_topic').value: self._on_odom,
                value('fault_reason_topic').value: self._on_fault_reason,
                value('emergency_topic').value: self._on_emergency,
            }
            trigger_types = {
                value('rosout_topic').value: 'rcl_interfaces/msg/Log',
                value('goal_status_topic').value:
                    'action_msgs/msg/GoalStatusArray',
                value('odom_topic').value: 'nav_msgs/msg/Odometry',
                value('fault_reason_topic').value: 'std_msgs/msg/String',
                value('emergency_topic').value: 'std_msgs/msg/Bool',
            }
            subscribed = dict(self._topics)
            for topic, msg_type in trigger_types.items():
                subscribed.setdefault(topic, msg_type)
            self._types = subscribed
            for topic, msg_type in subscribed.items():
                latched = topic in self._latched_names
                qos = QoSProfile(
                    depth=100 if latched or topic in reliable else 50,
                    history=HistoryPolicy.KEEP_LAST,
                    reliability=(ReliabilityPolicy.RELIABLE
                                 if latched or topic in reliable
                                 else ReliabilityPolicy.BEST_EFFORT),
                    durability=(DurabilityPolicy.TRANSIENT_LOCAL
                                if latched else DurabilityPolicy.VOLATILE))
                decoder = decoders.get(topic)
                message_class = get_message(msg_type)
                self.create_subscription(
                    message_class, topic,
                    self._make_callback(topic, message_class, decoder,
                                        recorded=topic in self._topics,
                                        latched=latched),
                    qos, raw=True)
            self.create_timer(0.5, self._tick)
            self.get_logger().info(
                f'nav event recorder: {len(self._topics)} topics, '
                f'{value("pre_s").value:.0f} s before / '
                f'{value("post_s").value:.0f} s after each event -> '
                f'{self._output_dir}')

        def _now_ns(self):
            return self.get_clock().now().nanoseconds

        def _make_callback(self, topic, message_class, decoder, recorded,
                           latched):
            def callback(data):
                stamp_ns = self._now_ns()
                if recorded:
                    if latched:
                        self._latched.setdefault(topic, [])
                        self._latched[topic] = (
                            self._latched[topic] + [data])[-20:]
                    else:
                        self._ring.append(stamp_ns, topic, data)
                if decoder is not None:
                    decoder(stamp_ns,
                            deserialize_message(data, message_class))
            return callback

        def _trigger(self, stamp_ns, reason, detail=''):
            if not self._capture.active:
                self.get_logger().info(f'event {reason}: {detail}')
            self._capture.trigger(stamp_ns, reason, detail)

        def _on_rosout(self, stamp_ns, message):
            hit = self._rosout.check(stamp_ns, message.level, message.name,
                                     message.msg)
            if hit is not None:
                self._trigger(stamp_ns, *hit)

        def _on_goal_status(self, stamp_ns, message):
            statuses = [(bytes(s.goal_info.goal_id.uuid), s.status)
                        for s in message.status_list]
            for goal_id in self._goals.update(statuses):
                self._trigger(stamp_ns, 'goal_aborted', goal_id.hex()[:8])

        def _on_odom(self, stamp_ns, message):
            position = message.pose.pose.position
            self._last_pose = (position.x, position.y)
            if self._progress.update(stamp_ns, position.x, position.y,
                                     self._goals.active):
                self._trigger(stamp_ns, 'no_progress',
                              f'odom ({position.x:.2f}, {position.y:.2f})')

        def _on_fault_reason(self, stamp_ns, message):
            if message.data and message.data != self._last_fault:
                self._trigger(stamp_ns, 'localization_fault', message.data)
            self._last_fault = message.data

        def _on_emergency(self, stamp_ns, message):
            if message.data and not self._emergency:
                self._trigger(stamp_ns, 'localization_emergency_stop')
            self._emergency = bool(message.data)

        def _tick(self):
            now_ns = self._now_ns()
            window = self._capture.poll(now_ns)
            if window is not None:
                start_ns, end_ns, events = window
                messages = self._ring.window(start_ns, end_ns)
                latched = [(topic, data)
                           for topic, items in self._latched.items()
                           for data in items]
                self._jobs.put((start_ns, end_ns, events, messages, latched))
            if not self._capture.active:
                self._ring.prune(now_ns - self._keep_ns)

        def _write_loop(self):
            while True:
                start_ns, end_ns, events, messages, latched = self._jobs.get()
                first = events[0]
                path = os.path.join(
                    self._output_dir,
                    event_dir_name(first['stamp_ns'], first['reason']))
                try:
                    os.makedirs(path, exist_ok=False)
                    began = time.monotonic()
                    summary = write_event_bag(
                        path, messages, latched, self._types, start_ns, {
                            'start_ns': start_ns,
                            'end_ns': end_ns,
                            'odom_at_write': self._last_pose,
                            'events': [
                                dict(e, offset_s=round(
                                    (e['stamp_ns'] - start_ns) / 1e9, 3))
                                for e in events],
                        })
                    removed = prune_event_dirs(
                        self._output_dir, self._max_total_mb,
                        self._max_events, keep=path)
                    self.get_logger().info(
                        f'wrote {path} ({len(events)} events, '
                        f'{sum(summary["message_counts"].values())} msgs, '
                        f'{time.monotonic() - began:.1f} s)'
                        + (f'; removed {len(removed)} old' if removed
                           else ''))
                except Exception as error:  # keep the recorder alive
                    self.get_logger().error(
                        f'event bag {path} failed: {error}')

    rclpy.init(args=args)
    node = NavEventRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
