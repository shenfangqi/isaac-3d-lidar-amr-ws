#!/usr/bin/env python3
"""Block mapping startup until host wall time is synchronized and stable."""

import argparse
import os
import sys
import time


def _log(message):
    print(f"[mapping-time-gate] {message}", flush=True)


def wait_for_time_sync(marker, stable_seconds, timeout_seconds, jump_limit_seconds):
    """Wait for systemd-timesyncd's marker and reject later wall-clock jumps."""
    started = time.monotonic()
    next_wait_log = started

    while not os.path.exists(marker):
        now = time.monotonic()
        if now - started >= timeout_seconds:
            raise TimeoutError(
                f"time synchronization marker did not appear within "
                f"{timeout_seconds:.0f} s: {marker}"
            )
        if now >= next_wait_log:
            _log(f"waiting for host time synchronization marker: {marker}")
            next_wait_log = now + 30.0
        time.sleep(0.5)

    marker_age = max(0.0, time.time() - os.path.getmtime(marker))
    _log(
        f"host reports synchronized time; marker age={marker_age:.3f} s; "
        f"checking {stable_seconds:.1f} s clock stability"
    )

    stable_started = time.monotonic()
    previous_wall = time.time()
    previous_monotonic = stable_started
    while True:
        time.sleep(0.25)
        wall_now = time.time()
        monotonic_now = time.monotonic()
        wall_elapsed = wall_now - previous_wall
        monotonic_elapsed = monotonic_now - previous_monotonic
        clock_step = abs(wall_elapsed - monotonic_elapsed)
        if clock_step > jump_limit_seconds:
            _log(
                f"wall-clock step {clock_step:.3f} s detected; restarting "
                "stability window"
            )
            stable_started = monotonic_now

        if monotonic_now - started >= timeout_seconds:
            raise TimeoutError(
                f"wall clock did not remain stable for {stable_seconds:.1f} s"
            )
        if monotonic_now - stable_started >= stable_seconds:
            _log("time gate passed; FAST-LIO2 may start")
            return

        previous_wall = wall_now
        previous_monotonic = monotonic_now


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--marker",
        default="/run/host-systemd-timesync/synchronized",
        help="systemd-timesyncd synchronization marker mounted from the host",
    )
    parser.add_argument("--stable-seconds", type=float, default=10.0)
    parser.add_argument("--timeout-seconds", type=float, default=1800.0)
    parser.add_argument("--jump-limit-seconds", type=float, default=0.25)
    args = parser.parse_args()

    if args.stable_seconds <= 0.0:
        parser.error("--stable-seconds must be positive")
    if args.timeout_seconds < args.stable_seconds:
        parser.error("--timeout-seconds must be at least --stable-seconds")
    if args.jump_limit_seconds <= 0.0:
        parser.error("--jump-limit-seconds must be positive")

    try:
        wait_for_time_sync(
            args.marker,
            args.stable_seconds,
            args.timeout_seconds,
            args.jump_limit_seconds,
        )
    except TimeoutError as error:
        _log(f"ERROR: {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
