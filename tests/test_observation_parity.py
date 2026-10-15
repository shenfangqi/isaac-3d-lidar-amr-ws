from pathlib import Path
import re
import subprocess

import yaml

from scripts.analyze_observation_parity import (
    evidence_coverage,
    replay_consumer_filter,
    Sample,
    StatusSample,
    status_stats,
    stream_stats,
    summarize,
)


ROOT = Path(__file__).resolve().parents[1]
ACCEPTANCE = yaml.safe_load(
    (ROOT / "configs/carbot/observation_parity_acceptance.yaml").read_text(
        encoding="utf-8"))
PROFILE = yaml.safe_load(
    (ROOT / "configs/carbot/evidence_degraded.yaml").read_text(
        encoding="utf-8"))
START_NS = 1_000_000_000_000


def wheel_stream(count=500, period_ms=20, latency_ms=5.0, offset_ms=0.0):
    """Return an ideal 50 Hz wheel stream: header = device + offset."""
    samples = []
    for index in range(count):
        device_us = 10_000_000 + index * period_ms * 1000
        header_ns = START_NS + device_us * 1000 + int(offset_ms * 1e6)
        samples.append(Sample(
            header_ns + int(latency_ms * 1e6), header_ns, index + 1,
            device_us, 7, (index, -index)))
    return samples


def imu_stream(count, period_ms, latency_ms):
    return [
        Sample(START_NS + index * int(period_ms * 1e6)
               + int(latency_ms * 1e6),
               START_NS + index * int(period_ms * 1e6))
        for index in range(count)
    ]


def status_stream(count, offsets_ms):
    return [
        StatusSample(START_NS + index * 500_000_000, 7, True, True, 3, 0,
                     1000 + index * 500, int(offsets_ms(index) * 1e6))
        for index in range(count)
    ]


def test_ideal_stream_rate_gaps_and_latency():
    stats = stream_stats(wheel_stream())
    assert abs(stats["rate_hz"] - 50.0) < 1e-9
    assert stats["receive_gap_ms"]["max"] == 20.0
    assert stats["header_backwards"] == 0
    assert stats["latency_ms"]["p50"] == 5.0
    assert stats["latency_minute_median_spread_ms"] == 0.0
    assert stats["duplicates"] == stats["reorders"] == 0
    assert stats["skipped_samples"] == 0
    assert stats["device_gap_max_ms"] == 20.0
    assert stats["clock_corrections"] == 0


def test_duplicates_reorders_and_skips_are_separated():
    samples = wheel_stream(20)
    # An exact repeat, a stale (older) sample and two missing sequences.
    stream = (samples[:5] + [samples[4]] + samples[5:8] + [samples[6]]
              + samples[10:])
    stats = stream_stats(stream)
    assert stats["duplicates"] == 1
    assert stats["reorders"] == 1
    assert stats["skipped_samples"] == 2
    assert stats["max_sequence_step"] == 3
    assert stats["device_gap_max_ms"] == 60.0


def test_same_sequence_with_different_data_is_a_reorder_not_a_duplicate():
    samples = wheel_stream(5)
    changed = Sample(samples[2].receive_ns + 1, samples[2].header_ns,
                     samples[2].sequence, samples[2].device_us, 7, (99, 99))
    stats = stream_stats(samples[:3] + [changed] + samples[3:])
    assert stats["duplicates"] == 0
    assert stats["reorders"] == 1


def test_consumer_filter_drops_exactly_duplicates_and_reorders():
    samples = wheel_stream(30)
    stream = (samples[:10] + [samples[9], samples[9]] + samples[10:15]
              + [samples[3]] + samples[15:])
    stats = stream_stats(stream)
    replay = replay_consumer_filter(stream)
    assert replay["dropped"] == stats["duplicates"] + stats["reorders"] == 3
    assert replay["accepted"] == 30


def test_boot_change_resets_the_baseline():
    first = wheel_stream(10)
    second = [Sample(s.receive_ns + 10**9, s.header_ns + 10**9, s.sequence,
                     s.device_us, 8, s.payload) for s in wheel_stream(10)]
    stats = stream_stats(first + second)
    assert stats["boot_changes"] == 1
    assert stats["reorders"] == 0
    assert replay_consumer_filter(first + second)["dropped"] == 0


def test_clock_corrections_come_from_header_minus_device():
    samples = wheel_stream(200)
    corrected = samples[:100] + [
        Sample(s.receive_ns + 1_500_000, s.header_ns + 1_500_000,
               s.sequence, s.device_us, s.boot_id, s.payload)
        for s in samples[100:]]
    stats = stream_stats(corrected)
    assert stats["clock_corrections"] == 1
    assert abs(stats["clock_correction_max_abs_ms"] - 1.5) < 1e-9


def test_status_reports_resync_cadence_and_session_health():
    status = status_stats(status_stream(
        480, lambda index: 0.5 * (index // 120)))
    assert status["clock_offset_changes"] == 3
    assert status["clock_offset_step_max_abs_ms"] == 0.5
    assert status["clock_offset_change_interval_median_s"] == 60.0
    assert status["reconnect_count_delta"] == 0
    assert status["receive_gap_max_ms"] == 500.0
    assert status["session_uptime_monotonic"] is True


def passing_inputs():
    count = 50 * 1830
    streams = {
        "/wheel_ticks": wheel_stream(count),
        "/mid360/imu/data_raw": imu_stream(200 * 1830, 5, 10.8),
    }
    status = status_stream(2 * 1830, lambda index: 0.3 * (index // 120))
    return streams, status


def test_a_clean_capture_passes_every_check():
    streams, status = passing_inputs()
    report = summarize(streams, status, ACCEPTANCE, PROFILE)
    failed = [check for check in report["checks"] if not check["pass"]]
    assert failed == []
    assert report["passed"] is True
    assert report["acceptance_status"] == "CONFIRMED"


def test_latency_drift_and_fixed_offset_firmware_fail():
    streams, _ = passing_inputs()
    # Fixed-offset firmware: latency drifts 14 ms (5 -> 19 ms, still inside
    # the absolute band) and the status offset never changes.
    streams["/wheel_ticks"] = [
        Sample(s.receive_ns + int(14e6 * index / len(streams["/wheel_ticks"])),
               s.header_ns, s.sequence, s.device_us, s.boot_id, s.payload)
        for index, s in enumerate(streams["/wheel_ticks"])]
    status = status_stream(2 * 1830, lambda index: 0.0)
    report = summarize(streams, status, ACCEPTANCE, PROFILE)
    failed = {check["name"] for check in report["checks"]
              if not check["pass"]}
    assert failed == {
        "/wheel_ticks latency_minute_median_spread_ms",
        "status clock_offset_changes (periodic resync active)",
    }


def test_missing_required_stream_fails():
    streams, status = passing_inputs()
    del streams["/mid360/imu/data_raw"]
    report = summarize(streams, status, ACCEPTANCE, PROFILE)
    assert report["passed"] is False
    assert any(check["name"] == "/mid360/imu/data_raw present"
               for check in report["checks"])


def test_evidence_coverage_allows_only_profile_rounding():
    stats = {"skip_fraction": 0.000222, "duplicate_fraction": 6.7e-05,
             "receive_gap_ms": {"max": 345.34}}
    report = {"streams": {"/wheel_ticks": stats}}
    assert all(check["pass"] for check in evidence_coverage(
        report, PROFILE, "/wheel_ticks", 0.05))
    stats["receive_gap_ms"]["max"] = 400.0
    failed = [check["name"] for check in evidence_coverage(
        report, PROFILE, "/wheel_ticks", 0.05) if not check["pass"]]
    assert failed == ["encoder receive_gap_max_ms"]


def test_recorder_never_subscribes_commands_or_point_clouds():
    script = ROOT / "scripts/record_observation_parity.sh"
    subprocess.run(["bash", "-n", str(script)], check=True)
    source = script.read_text(encoding="utf-8")
    topics = re.findall(r"^(?:REQUIRED|OPTIONAL)=\((.*)\)$", source, re.M)
    recorded = " ".join(topics).split()
    assert recorded == ["/wheel_ticks", "/carbot/status",
                        "/mid360/imu/data_raw", "/odom", "/fast_lio/imu_odom"]
    assert "/cmd_vel" not in " ".join(recorded)
    assert "reliability: best_effort" in source
    assert "systemd-run --user" in source
    assert "cmd_vel_command" not in source.split("set -o pipefail", 1)[1]
    assert set(ACCEPTANCE["streams"]) <= set(recorded)
