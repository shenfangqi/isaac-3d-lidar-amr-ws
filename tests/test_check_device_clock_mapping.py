from pathlib import Path

import yaml

from scripts.analyze_observation_parity import Sample
from scripts.check_device_clock_mapping import evaluate


ROOT = Path(__file__).resolve().parents[1]
LAG_NS = int(21.4 * 3600e9)


def wheel(count, header_lag_ns=0, latency_ns=12_000_000):
    samples = []
    for index in range(count):
        device_us = 5_000_000 + index * 20_000
        true_ns = 1_792_065_000 * 10**9 + device_us * 1000
        samples.append(Sample(true_ns + latency_ns, true_ns - header_lag_ns,
                              index + 1, device_us, 7, (index, index)))
    return samples


def test_cold_boot_header_lag_is_removed():
    report = evaluate(wheel(3000, header_lag_ns=LAG_NS))
    assert report["passed"] is True
    assert report["future_stamps"] == 0
    assert report["resets"] == 0
    assert report["lag_ms"]["p50"] == 0.0


def test_correct_header_gives_a_steady_offset():
    report = evaluate(wheel(4000), expect_header_correct=True)
    assert report["passed"] is True
    assert report["header_minus_mapped_spread_ms"] == 0.0
    assert report["header_minus_mapped_minute_median_ms"][0] == -12.0


def test_drifting_header_fails_the_header_check_only():
    samples = [Sample(s.receive_ns, s.header_ns + index * 10_000,
                      s.sequence, s.device_us, s.boot_id, s.payload)
               for index, s in enumerate(wheel(6000))]
    report = evaluate(samples, expect_header_correct=True)
    failed = [check["name"] for check in report["checks"] if not check["pass"]]
    assert failed == ["header-minus-mapped spread <= 2.0 ms"]


def test_deployed_wheel_odometry_uses_device_clock_stamps():
    parameters = yaml.safe_load((
        ROOT / "src/carbot_hardware/config/wheel_odometry_fused.yaml"
    ).read_text(encoding="utf-8"))["carbot_wheel_odometry"]["ros__parameters"]
    assert parameters["stamp_mode"] == "device_clock"
    service = (ROOT / "src/carbot_hardware/systemd/"
               "carbot-wheel-odometry.service").read_text(encoding="utf-8")
    assert "wheel_odometry_fused.yaml" in service
