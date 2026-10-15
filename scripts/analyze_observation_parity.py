#!/usr/bin/env python3
"""Analyze a static Carbot observation bag for the observation_parity gate.

The bag is recorded on the Jetson with the robot stationary (see
scripts/record_observation_parity.sh).  The report measures, per stream:
rate, receive gaps (recorder clock), device/header gaps, receive-minus-header
latency per minute, sequence skips, duplicates and reorders; for the ESP32
status: reconnects, ping failures, time synchronization and clock-offset
corrections.  It replays the wheel-odometry duplicate filter to show that the
samples it drops are exactly the duplicates and reorders, checks the result
against configs/carbot/observation_parity_acceptance.yaml, and compares the
measured transport with the evidence-degraded simulation profile.

Read-only: this script never publishes or commands motion.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
ACCEPTANCE = ROOT / "configs/carbot/observation_parity_acceptance.yaml"
EVIDENCE_PROFILE = ROOT / "configs/carbot/evidence_degraded.yaml"

WHEEL_TYPE = "carbot_msgs/msg/WheelTicks"
STATUS_TYPE = "carbot_msgs/msg/CarbotStatus"
HEADER_TYPES = {
    "sensor_msgs/msg/Imu",
    "nav_msgs/msg/Odometry",
    "sensor_msgs/msg/LaserScan",
    "sensor_msgs/msg/PointCloud2",
}


@dataclass(frozen=True)
class Sample:
    """One received message reduced to its timing and identity fields."""

    receive_ns: int
    header_ns: int
    sequence: int = None
    device_us: int = None
    boot_id: int = None
    payload: tuple = ()


@dataclass(frozen=True)
class StatusSample:
    """The /carbot/status fields the communication gate uses."""

    receive_ns: int
    boot_id: int
    agent_connected: bool
    time_synchronized: bool
    reconnect_count: int
    consecutive_ping_failures: int
    session_uptime_ms: int
    clock_offset_ns: int


def percentile(values, fraction):
    """Return the nearest-rank percentile of a non-empty sequence."""
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def _median(values):
    return percentile(values, 0.5) if values else None


def _ms(ns):
    return ns / 1.0e6


def stream_stats(samples):
    """Return transport statistics for one stream, in receive order."""
    if len(samples) < 2:
        return {"count": len(samples)}
    duration_s = (samples[-1].receive_ns - samples[0].receive_ns) / 1.0e9
    receive_gaps = [_ms(b.receive_ns - a.receive_ns)
                    for a, b in zip(samples, samples[1:])]
    header_gaps = [_ms(b.header_ns - a.header_ns)
                   for a, b in zip(samples, samples[1:])]
    latencies = [_ms(s.receive_ns - s.header_ns) for s in samples]

    start = samples[0].receive_ns
    minutes = {}
    for sample in samples:
        minute = int((sample.receive_ns - start) // 60_000_000_000)
        minutes.setdefault(minute, []).append(
            _ms(sample.receive_ns - sample.header_ns))
    minute_medians = [_median(minutes[key]) for key in sorted(minutes)]

    report = {
        "count": len(samples),
        "duration_s": duration_s,
        "rate_hz": (len(samples) - 1) / duration_s if duration_s > 0 else None,
        "receive_gap_ms": {
            "p50": percentile(receive_gaps, 0.50),
            "p95": percentile(receive_gaps, 0.95),
            "p99": percentile(receive_gaps, 0.99),
            "p999": percentile(receive_gaps, 0.999),
            "max": max(receive_gaps),
        },
        "receive_gaps_over_ms": {
            str(limit): sum(gap > limit for gap in receive_gaps)
            for limit in (100, 200, 300, 400)
        },
        "header_gap_max_ms": max(header_gaps),
        "header_backwards": sum(gap < 0 for gap in header_gaps),
        "latency_ms": {
            "p50": percentile(latencies, 0.50),
            "p95": percentile(latencies, 0.95),
            "min": min(latencies),
            "max": max(latencies),
        },
        "latency_minute_median_ms": minute_medians,
        "latency_minute_median_spread_ms": (
            max(minute_medians) - min(minute_medians)),
    }
    if samples[0].sequence is not None:
        report.update(sequence_stats(samples))
    return report


def sequence_stats(samples):
    """Count duplicates, reorders, skips and device-clock behaviour."""
    duplicates = 0
    reorders = 0
    skipped = 0
    max_step = 0
    boot_changes = 0
    device_gaps = []
    offset_steps = []
    newest = samples[0]
    for sample in samples[1:]:
        if sample.boot_id != newest.boot_id:
            boot_changes += 1
            newest = sample
            continue
        if sample.sequence == newest.sequence:
            if (sample.device_us, sample.payload) == (
                    newest.device_us, newest.payload):
                duplicates += 1
            else:
                reorders += 1
            continue
        if sample.sequence < newest.sequence:
            reorders += 1
            continue
        step = sample.sequence - newest.sequence
        skipped += step - 1
        max_step = max(max_step, step)
        device_gaps.append((sample.device_us - newest.device_us) / 1.0e3)
        # header = device + offset; a change is a firmware clock correction.
        offset_steps.append(_ms(
            (sample.header_ns - sample.device_us * 1000)
            - (newest.header_ns - newest.device_us * 1000)))
        newest = sample
    span = samples[-1].sequence - samples[0].sequence
    corrections = [step for step in offset_steps if abs(step) > 0.001]
    return {
        "duplicates": duplicates,
        "reorders": reorders,
        "skipped_samples": skipped,
        "skip_fraction": skipped / span if span > 0 else 0.0,
        "duplicate_fraction": duplicates / len(samples),
        "max_sequence_step": max_step,
        "boot_changes": boot_changes,
        "device_gap_max_ms": max(device_gaps) if device_gaps else None,
        "device_gap_min_ms": min(device_gaps) if device_gaps else None,
        "clock_corrections": len(corrections),
        "clock_correction_max_abs_ms": (
            max(abs(step) for step in corrections) if corrections else 0.0),
    }


def replay_consumer_filter(samples):
    """Apply carbot_wheel_odometry's accept rule and count what it drops.

    The node keeps the newest accepted sample and drops a sample whose
    sequence or device stamp is not newer (wheel_odometry.py, same boot).
    """
    accepted = 0
    dropped = 0
    newest = None
    for sample in samples:
        if newest is None or sample.boot_id != newest.boot_id:
            newest = sample
            accepted += 1
            continue
        if (sample.sequence <= newest.sequence
                or sample.device_us <= newest.device_us):
            dropped += 1
            continue
        newest = sample
        accepted += 1
    return {"accepted": accepted, "dropped": dropped}


def status_stats(samples):
    """Summarize ESP32 session health and clock corrections."""
    if not samples:
        return {"count": 0}
    offsets = [s.clock_offset_ns for s in samples]
    steps = [_ms(b - a) for a, b in zip(offsets, offsets[1:]) if b != a]
    change_times = [
        b.receive_ns for a, b in zip(samples, samples[1:])
        if b.clock_offset_ns != a.clock_offset_ns]
    intervals = [(b - a) / 1.0e9
                 for a, b in zip(change_times, change_times[1:])]
    uptimes = [s.session_uptime_ms for s in samples]
    receive_gaps = [_ms(b.receive_ns - a.receive_ns)
                    for a, b in zip(samples, samples[1:])]
    return {
        "count": len(samples),
        "receive_gap_max_ms": max(receive_gaps) if receive_gaps else None,
        "boot_ids": sorted({s.boot_id for s in samples}),
        "agent_connected_always": all(s.agent_connected for s in samples),
        "time_synchronized_always": all(
            s.time_synchronized for s in samples),
        "reconnect_count_delta": (
            samples[-1].reconnect_count - samples[0].reconnect_count),
        "ping_failures_max": max(
            s.consecutive_ping_failures for s in samples),
        "session_uptime_monotonic": all(
            b > a for a, b in zip(uptimes, uptimes[1:])),
        "clock_offset_changes": len(steps),
        "clock_offset_step_max_abs_ms": (
            max(abs(step) for step in steps) if steps else 0.0),
        "clock_offset_change_interval_median_s": _median(intervals),
        "clock_offset_range_ms": _ms(max(offsets) - min(offsets)),
    }


def _check(name, value, limit, passed):
    return {"name": name, "value": value, "limit": limit,
            "pass": bool(passed)}


def evaluate(report, acceptance):
    """Return the acceptance checks; every check must pass."""
    checks = []
    minimum = acceptance["minimum_duration_s"]
    for topic, rules in acceptance["streams"].items():
        stats = report["streams"].get(topic)
        if not stats or stats.get("count", 0) < 2:
            checks.append(_check(f"{topic} present", 0, "> 1 sample", False))
            continue
        checks.append(_check(f"{topic} duration_s", stats["duration_s"],
                             f">= {minimum}", stats["duration_s"] >= minimum))
        nominal = rules["nominal_rate_hz"]
        tolerance = rules["rate_tolerance_fraction"]
        checks.append(_check(
            f"{topic} rate_hz", stats["rate_hz"],
            f"{nominal} +/- {tolerance:.0%}",
            abs(stats["rate_hz"] - nominal) <= tolerance * nominal))
        limit = rules["receive_gap_max_ms"]
        checks.append(_check(f"{topic} receive_gap_max_ms",
                             stats["receive_gap_ms"]["max"], f"<= {limit}",
                             stats["receive_gap_ms"]["max"] <= limit))
        limit = rules["receive_gap_p999_ms"]
        checks.append(_check(f"{topic} receive_gap_p999_ms",
                             stats["receive_gap_ms"]["p999"], f"<= {limit}",
                             stats["receive_gap_ms"]["p999"] <= limit))
        checks.append(_check(f"{topic} header_backwards",
                             stats["header_backwards"], "== 0",
                             stats["header_backwards"] == 0))
        low, high = rules["latency_minute_median_ms"]
        medians = stats["latency_minute_median_ms"]
        checks.append(_check(
            f"{topic} latency_minute_median_ms",
            [min(medians), max(medians)], f"within [{low}, {high}]",
            low <= min(medians) and max(medians) <= high))
        limit = rules["latency_minute_median_spread_ms"]
        checks.append(_check(
            f"{topic} latency_minute_median_spread_ms",
            stats["latency_minute_median_spread_ms"], f"<= {limit}",
            stats["latency_minute_median_spread_ms"] <= limit))
        if "sequence" in rules:
            sequence = rules["sequence"]
            checks.append(_check(f"{topic} reorders", stats["reorders"],
                                 "== 0", stats["reorders"] == 0))
            checks.append(_check(f"{topic} boot_changes",
                                 stats["boot_changes"], "== 0",
                                 stats["boot_changes"] == 0))
            limit = sequence["skip_fraction_max"]
            checks.append(_check(f"{topic} skip_fraction",
                                 stats["skip_fraction"], f"<= {limit}",
                                 stats["skip_fraction"] <= limit))
            limit = sequence["device_gap_max_ms"]
            checks.append(_check(f"{topic} device_gap_max_ms",
                                 stats["device_gap_max_ms"], f"<= {limit}",
                                 stats["device_gap_max_ms"] <= limit))
            limit = sequence["clock_correction_max_abs_ms"]
            checks.append(_check(
                f"{topic} clock_correction_max_abs_ms",
                stats["clock_correction_max_abs_ms"], f"<= {limit}",
                stats["clock_correction_max_abs_ms"] <= limit))
            replay = report["consumer_filter"].get(topic)
            expected = stats["duplicates"] + stats["reorders"]
            checks.append(_check(
                f"{topic} consumer drops exactly duplicates+reorders",
                replay["dropped"], f"== {expected}",
                replay["dropped"] == expected))

    rules = acceptance["chassis_status"]
    status = report["status"]
    if status.get("count", 0) == 0:
        checks.append(_check("status present", 0, "> 0 samples", False))
    else:
        checks.extend([
            _check("status single boot_id", status["boot_ids"], "1 id",
                   len(status["boot_ids"]) == 1),
            _check("status agent_connected", status["agent_connected_always"],
                   "always", status["agent_connected_always"]),
            _check("status time_synchronized",
                   status["time_synchronized_always"], "always",
                   status["time_synchronized_always"]),
            _check("status reconnect_count_delta",
                   status["reconnect_count_delta"], "== 0",
                   status["reconnect_count_delta"] == 0),
            _check("status ping_failures_max", status["ping_failures_max"],
                   "== 0", status["ping_failures_max"] == 0),
            _check("status session_uptime_monotonic",
                   status["session_uptime_monotonic"], "true",
                   status["session_uptime_monotonic"]),
        ])
        limit = rules["receive_gap_max_ms"]
        checks.append(_check(
            "status receive_gap_max_ms", status["receive_gap_max_ms"],
            f"<= {limit}", status["count"] > 1
            and status["receive_gap_max_ms"] <= limit))
        minimum = rules["clock_offset_changes_min"]
        checks.append(_check(
            "status clock_offset_changes (periodic resync active)",
            status["clock_offset_changes"], f">= {minimum}",
            status["clock_offset_changes"] >= minimum))
        limit = rules["clock_offset_step_max_abs_ms"]
        checks.append(_check(
            "status clock_offset_step_max_abs_ms",
            status["clock_offset_step_max_abs_ms"], f"<= {limit}",
            status["clock_offset_step_max_abs_ms"] <= limit))
    return checks


def evidence_coverage(report, profile, wheel_topic, tolerance=0.0):
    """Check that the simulation profile models at least the measured faults.

    A failed check does not fail the hardware gate; it means
    scripts/build_sim_real_evidence_profile.py inputs must be refreshed so
    training sees the measured transport.
    """
    checks = []
    wheel = report["streams"].get(wheel_topic)
    if wheel and "skip_fraction" in wheel:
        encoder = profile["encoder"]
        for name, measured, modeled in (
                ("encoder drop_fraction", wheel["skip_fraction"],
                 encoder["drop_fraction"]),
                ("encoder duplicate_fraction", wheel["duplicate_fraction"],
                 encoder["duplicate_fraction"]),
                ("encoder receive_gap_max_ms", wheel["receive_gap_ms"]["max"],
                 encoder["maximum_observed_receive_gap_s"] * 1000.0)):
            checks.append(_check(name, measured, f"<= modeled {modeled:g}",
                                 measured <= modeled * (1.0 + tolerance)))
    return checks


def summarize(streams, status, acceptance, profile):
    """Build the full report from already-decoded samples."""
    report = {
        "schema_version": 1,
        "motion_commanded": False,
        "streams": {topic: stream_stats(samples)
                    for topic, samples in streams.items()},
        "consumer_filter": {
            topic: replay_consumer_filter(samples)
            for topic, samples in streams.items()
            if samples and samples[0].sequence is not None},
        "status": status_stats(status),
    }
    checks = evaluate(report, acceptance)
    report["acceptance_status"] = acceptance.get("status")
    report["checks"] = checks
    report["passed"] = all(check["pass"] for check in checks)
    report["evidence_profile_coverage"] = evidence_coverage(
        report, profile, acceptance["wheel_topic"],
        acceptance.get("evidence_profile_rounding_tolerance", 0.0))
    return report


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def read_bag(path):
    """Decode the timing fields of every supported topic in a rosbag2."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    metadata = yaml.safe_load(
        (Path(path) / "metadata.yaml").read_text(encoding="utf-8"))
    storage = metadata["rosbag2_bagfile_information"]["storage_identifier"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {topic.name: topic.type
             for topic in reader.get_all_topics_and_types()}
    wanted = {name: kind for name, kind in types.items()
              if kind in HEADER_TYPES | {WHEEL_TYPE, STATUS_TYPE}}
    classes = {name: get_message(kind) for name, kind in wanted.items()}
    streams = {name: [] for name, kind in wanted.items()
               if kind != STATUS_TYPE}
    status = []
    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(data, classes[topic])
        if wanted[topic] == STATUS_TYPE:
            status.append(StatusSample(
                receive_ns, message.boot_id, message.agent_connected,
                message.time_synchronized, message.reconnect_count,
                message.consecutive_ping_failures,
                message.session_uptime_ms, message.clock_offset_ns))
        elif wanted[topic] == WHEEL_TYPE:
            streams[topic].append(Sample(
                receive_ns, _stamp_ns(message.header.stamp),
                message.sequence, message.device_stamp_us, message.boot_id,
                (message.left_ticks, message.right_ticks)))
        else:
            streams[topic].append(
                Sample(receive_ns, _stamp_ns(message.header.stamp)))
    return streams, status


def markdown(report):
    """Render a short Chinese summary for the evidence directory."""
    lines = ["# observation_parity 静态通信分析", "",
             f"- 验收阈值状态：`{report['acceptance_status']}`",
             f"- 总结论：**{'通过' if report['passed'] else '未通过'}**", "",
             "| 检查项 | 实测 | 阈值 | 结果 |", "|---|---|---|---|"]
    for check in report["checks"]:
        value = check["value"]
        if isinstance(value, float):
            value = f"{value:.4g}"
        lines.append(f"| {check['name']} | {value} | {check['limit']} | "
                     f"{'PASS' if check['pass'] else 'FAIL'} |")
    lines += ["", "## 仿真证据配置覆盖（不影响硬件结论）", "",
              "| 检查项 | 实测 | 建模 | 结果 |", "|---|---|---|---|"]
    for check in report["evidence_profile_coverage"]:
        value = check["value"]
        if isinstance(value, float):
            value = f"{value:.4g}"
        lines.append(f"| {check['name']} | {value} | {check['limit']} | "
                     f"{'覆盖' if check['pass'] else '需更新配置'} |")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag", type=Path)
    parser.add_argument("--acceptance", type=Path, default=ACCEPTANCE)
    parser.add_argument("--evidence-profile", type=Path,
                        default=EVIDENCE_PROFILE)
    parser.add_argument("--output", type=Path,
                        help="write <output>.json and <output>.md")
    args = parser.parse_args()
    acceptance = yaml.safe_load(args.acceptance.read_text(encoding="utf-8"))
    profile = yaml.safe_load(
        args.evidence_profile.read_text(encoding="utf-8"))
    streams, status = read_bag(args.bag)
    report = summarize(streams, status, acceptance, profile)
    report["bag"] = str(args.bag)
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.with_suffix(".json").write_text(encoded + "\n", "utf-8")
        args.output.with_suffix(".md").write_text(markdown(report), "utf-8")
    print(markdown(report))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
