"""Scans wait for their transform instead of blocking the executor."""

from isaac_3d_lidar_bringup.tf_wait_queue import TfWaitQueue


class Transforms:
    """Fake TF buffer that knows transforms up to ``latest``."""

    def __init__(self, latest, missing=()):
        self.latest = latest
        self.missing = set(missing)

    def __call__(self, frame_id, stamp_s):
        if stamp_s > self.latest or stamp_s in self.missing:
            raise LookupError('extrapolation into the future')
        return (frame_id, stamp_s)


def test_a_scan_waits_until_its_transform_arrives():
    queue = TfWaitQueue()
    tf = Transforms(latest=9.9)
    queue.push(10.0, 'base_footprint', 'scan10')
    assert queue.pop_ready(tf, now_s=10.05) == []
    assert len(queue) == 1
    tf.latest = 10.0
    assert queue.pop_ready(tf, now_s=10.1) == [
        ('scan10', ('base_footprint', 10.0))]
    assert len(queue) == 0 and queue.dropped == 0


def test_scans_are_processed_in_arrival_order():
    queue = TfWaitQueue()
    tf = Transforms(latest=10.05)
    queue.push(10.0, 'f', 'a')
    queue.push(10.1, 'f', 'b')
    queue.push(10.2, 'f', 'c')
    assert [item for item, _ in queue.pop_ready(tf, 10.2)] == ['a']
    tf.latest = 10.2
    assert [item for item, _ in queue.pop_ready(tf, 10.3)] == ['b', 'c']


def test_a_scan_whose_transform_never_comes_is_dropped():
    queue = TfWaitQueue(max_wait_s=0.5)
    tf = Transforms(latest=0.0, missing={10.0})
    queue.push(10.0, 'f', 'lost')
    queue.push(10.4, 'f', 'later')
    assert queue.pop_ready(tf, now_s=10.45) == []
    tf.latest = 10.4
    # 'lost' is now 0.6 s old and still has no transform; 'later' proceeds.
    assert queue.pop_ready(tf, now_s=10.6) == [('later', ('f', 10.4))]
    assert queue.dropped == 1


def test_the_queue_is_bounded():
    queue = TfWaitQueue(max_items=2)
    for stamp in (1.0, 2.0, 3.0):
        queue.push(stamp, 'f', stamp)
    assert len(queue) == 2 and queue.dropped == 1
