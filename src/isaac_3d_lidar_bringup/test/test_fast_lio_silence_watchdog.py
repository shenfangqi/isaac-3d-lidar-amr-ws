"""
FAST-LIO silence watchdog arming, driven through the node's own methods.

The ROS node is built without rclpy: publishers, the Nav2 lifecycle client
and the monotonic clock are replaced by recording doubles.
"""

from types import SimpleNamespace as NS

from nav2_msgs.srv import ManageLifecycleNodes
import pytest

import isaac_3d_lidar_bringup.fast_lio_base_adapter as adapter_module


GRACE = 20.0
SILENCE = 2.0


@pytest.fixture
def adapter(monkeypatch):
    clock = {'now': 100.0}
    monkeypatch.setattr(adapter_module, 'time',
                        NS(monotonic=lambda: clock['now']))
    node = object.__new__(adapter_module.FastLioBaseAdapter)
    node.clock = clock
    node.emergency = []
    node.reasons = []
    node.pause_requests = []
    node._max_output_silence = SILENCE
    node._startup_grace_period = GRACE
    node._last_source_wall = None
    node._first_source_wall = None
    node._guard = adapter_module.PosePlausibilityGuard(grace_period=GRACE)
    node._fault_latched = False
    node._pause_requested = False
    # The IMU->base transform never resolves, so _on_odometry returns
    # before PosePlausibilityGuard.evaluate(): source stamps never advance.
    node._lookup_base_in_imu = lambda: None
    node._emergency_publisher = NS(
        publish=lambda msg: node.emergency.append(msg.data))
    node._reason_publisher = NS(
        publish=lambda msg: node.reasons.append(msg.data))
    node._navigation_manager = NS(
        service_is_ready=lambda: True,
        call_async=lambda request: node.pause_requests.append(
            request.command))
    node.get_logger = lambda: NS(fatal=lambda msg: None,
                                 error=lambda msg: None)
    return node


def _stream(node, start, end, period=0.1):
    """Deliver FAST-LIO odometry from ``start`` to ``end`` seconds."""
    t = start
    while t <= end + 1e-9:
        node.clock['now'] = 100.0 + t
        node._on_odometry(None)
        node._watchdog_callback()
        t += period


def _watch_at(node, t):
    node.clock['now'] = 100.0 + t
    node._watchdog_callback()


def test_stream_dying_inside_grace_trips_once_grace_elapses(adapter):
    # PR #16 review finding 1: the watchdog armed on source stamps, so a
    # stream that died during the grace period never armed it.
    _stream(adapter, 0.0, 5.0)
    _watch_at(adapter, GRACE - 0.1)
    assert adapter.reasons == []

    _watch_at(adapter, GRACE)

    assert adapter.reasons == [
        f'FAST-LIO odometry silent for more than {SILENCE:.2f} s']
    assert adapter.emergency[-1] is True
    assert adapter.pause_requests == [
        ManageLifecycleNodes.Request.PAUSE]


def test_startup_gaps_inside_grace_do_not_trip(adapter):
    # FAST-LIO may pause while it initialises; only arm after the grace.
    _stream(adapter, 0.0, 0.0)
    _watch_at(adapter, 15.0)
    _stream(adapter, 15.0, 25.0)
    assert adapter.reasons == []
    assert adapter._fault_latched is False


def test_healthy_stream_after_grace_does_not_trip(adapter):
    _stream(adapter, 0.0, 40.0)
    assert adapter.reasons == []
    assert adapter.pause_requests == []


def test_silence_after_grace_trips_at_the_limit(adapter):
    _stream(adapter, 0.0, 30.0)
    _watch_at(adapter, 30.0 + SILENCE - 0.05)
    assert adapter.reasons == []

    _watch_at(adapter, 30.0 + SILENCE + 0.05)
    assert len(adapter.reasons) == 1
    assert adapter._fault_latched is True


def test_no_message_ever_does_not_arm(adapter):
    # Never-started FAST-LIO is a preflight failure, not a runtime fault.
    _watch_at(adapter, GRACE * 3)
    assert adapter.reasons == []
    assert adapter.emergency == []


def test_latched_fault_keeps_stopping_and_pauses_once(adapter):
    _stream(adapter, 0.0, 1.0)
    _watch_at(adapter, GRACE)
    _stream(adapter, GRACE, GRACE + 1.0)   # stream resumes after the trip

    assert len(adapter.reasons) == 1
    assert adapter.emergency[-1] is True
    assert all(value is True for value in adapter.emergency)
    assert adapter.pause_requests == [
        ManageLifecycleNodes.Request.PAUSE]
