"""
Hold scans until their transform arrives, without blocking the executor.

On the robot /scan and odom -> base_footprint carry the same stamp and arrive
together, so the scan callback often runs before that transform is in the
buffer.  A blocking lookup with a timeout cannot help: the TF listener shares
the node's executor and receives nothing while the callback waits, so every
such scan timed out and the backlog grew.  Instead the scan is queued and
retried from later callbacks until its transform exists or it is too old.
"""

import collections


class TfWaitQueue:
    """FIFO of (stamp, frame, item) waiting for a transform."""

    def __init__(self, max_items=10, max_wait_s=0.5):
        self._items = collections.deque()
        self._max_items = max_items
        self._max_wait_s = max_wait_s
        self.dropped = 0

    def __len__(self):
        return len(self._items)

    def push(self, stamp_s, frame_id, item):
        if len(self._items) >= self._max_items:
            self._items.popleft()
            self.dropped += 1
        self._items.append((stamp_s, frame_id, item))

    def pop_ready(self, lookup, now_s):
        """
        Return [(item, transform)] that can be processed now, oldest first.

        ``lookup(frame_id, stamp_s)`` returns a transform or raises.  An item
        whose lookup fails waits for a later call, and is dropped once it is
        older than ``max_wait_s``.  Items behind a waiting one also wait, so
        scans are always processed in arrival order.
        """
        ready = []
        while self._items:
            stamp_s, frame_id, item = self._items[0]
            try:
                transform = lookup(frame_id, stamp_s)
            except Exception:  # noqa: B902 - any lookup failure means wait
                if now_s - stamp_s <= self._max_wait_s:
                    break
                self._items.popleft()
                self.dropped += 1
                continue
            self._items.popleft()
            ready.append((item, transform))
        return ready
