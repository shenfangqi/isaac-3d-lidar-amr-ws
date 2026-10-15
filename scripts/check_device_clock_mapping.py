#!/usr/bin/env python3
"""Replay a recorded /wheel_ticks stream through the Jetson device-clock mapping.

Uses carbot_hardware.device_clock (issue #97), the code wheel_odometry runs
with stamp_mode device_clock, on the recorder's receive time and each sample's
device_stamp_us.  It reports the mapped stamp's lag behind the receive time,
resets, stamps in the future, monotonicity and, when the ESP32 header is known
to be correct, how steady header-minus-mapped is.  Read-only.
"""

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/carbot_hardware"))
sys.path.insert(0, str(ROOT / "scripts"))

from carbot_hardware.device_clock import DeviceClockMapper  # noqa: E402


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(fraction * len(ordered) + 0.999999) - 1))
    return ordered[index]


def map_stream(samples):
    """Return per-sample mapped stamps (ns) and the mapper after replay."""
    mapper = DeviceClockMapper()
    stamps = [mapper.update(s.boot_id, s.device_us, s.receive_ns)
              for s in samples]
    return stamps, mapper


def evaluate(samples, expect_header_correct=False, lag_range_ms=(0.0, 20.0),
             max_resets=1, header_spread_ms=2.0):
    """Return the mapping report and its checks for one wheel stream."""
    stamps, mapper = map_stream(samples)
    lags = [(s.receive_ns - stamp) / 1e6 for s, stamp in zip(samples, stamps)]
    backwards = sum(b < a for a, b in zip(stamps, stamps[1:]))
    start = samples[0].receive_ns
    minutes = {}
    for sample, stamp in zip(samples, stamps):
        minute = (sample.receive_ns - start) // 60_000_000_000
        minutes.setdefault(minute, []).append((sample.header_ns - stamp) / 1e6)
    header_minus_mapped = [percentile(values, 0.5)
                           for _, values in sorted(minutes.items())]
    report = {
        "count": len(samples),
        "resets": mapper.resets,
        "last_reset_reason": mapper.last_reset_reason,
        "lag_ms": {"min": min(lags), "p50": percentile(lags, 0.5),
                   "p99": percentile(lags, 0.99), "max": max(lags)},
        "future_stamps": sum(lag < 0 for lag in lags),
        "stamp_backwards": backwards,
        "header_minus_mapped_minute_median_ms": [
            min(header_minus_mapped), max(header_minus_mapped)],
        "header_minus_mapped_spread_ms": (
            max(header_minus_mapped) - min(header_minus_mapped)),
    }
    low, high = lag_range_ms
    checks = [
        ("future_stamps == 0", report["future_stamps"] == 0),
        (f"lag p50 within [{low}, {high}] ms",
         low <= report["lag_ms"]["p50"] <= high),
        (f"resets <= {max_resets}", report["resets"] <= max_resets),
    ]
    if expect_header_correct:
        checks.append((f"header-minus-mapped spread <= {header_spread_ms} ms",
                       report["header_minus_mapped_spread_ms"]
                       <= header_spread_ms))
    report["checks"] = [{"name": name, "pass": bool(passed)}
                        for name, passed in checks]
    report["passed"] = all(passed for _, passed in checks)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/wheel_ticks")
    parser.add_argument("--expect-header-correct", action="store_true",
                        help="also require a steady header-minus-mapped offset")
    parser.add_argument("--max-resets", type=int, default=1)
    args = parser.parse_args()
    from analyze_observation_parity import read_bag
    streams, _ = read_bag(args.bag)
    samples = streams.get(args.topic)
    if not samples or samples[0].device_us is None:
        raise SystemExit(f"{args.topic} with device_stamp_us not in {args.bag}")
    report = evaluate(samples, args.expect_header_correct,
                      max_resets=args.max_resets)
    report["bag"] = str(args.bag)
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
