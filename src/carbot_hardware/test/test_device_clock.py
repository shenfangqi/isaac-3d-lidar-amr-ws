import random

import pytest

from carbot_hardware.device_clock import (
    DeviceClockMapper,
    resolve_stamp_mode,
)


PERIOD_US = 20_000          # 50 Hz wheel ticks
BASE_LATENCY_NS = 12_000_000
# Jetson time minus device time; any epoch works.
OFFSET_NS = 1_792_065_543_123_403_036


def stream(count, start_us=10_000_000, offset_ns=OFFSET_NS, jitter_ns=None,
           drift_ppm=0.0, seed=7):
    """Yield (device_us, true_sample_ns, receive_ns) for a 50 Hz stream."""
    rng = random.Random(seed)
    for index in range(count):
        device_us = start_us + index * PERIOD_US
        elapsed_ns = (device_us - start_us) * 1000
        true_ns = (device_us * 1000 + offset_ns
                   + int(elapsed_ns * drift_ppm * 1e-6))
        extra = rng.uniform(0, jitter_ns) if jitter_ns else 0.0
        yield device_us, true_ns, true_ns + BASE_LATENCY_NS + int(extra)


def run(mapper, samples, boot_id=1):
    return [(true_ns, receive_ns, mapper.update(boot_id, device_us, receive_ns))
            for device_us, true_ns, receive_ns in samples]


def test_steady_stream_maps_to_sample_time_plus_minimum_latency():
    mapper = DeviceClockMapper()
    results = run(mapper, stream(1000))
    for true_ns, receive_ns, stamp_ns in results:
        assert stamp_ns - true_ns == BASE_LATENCY_NS
        assert stamp_ns <= receive_ns
    assert mapper.resets == 0


def test_jitter_is_removed_down_to_the_lowest_latency_seen():
    mapper = DeviceClockMapper()
    results = run(mapper, stream(3000, jitter_ns=15_000_000))
    # After the first window the envelope has seen near-minimum latency.
    errors = [stamp - true - BASE_LATENCY_NS for true, _, stamp in results[600:]]
    assert max(errors) < 1_000_000
    assert min(errors) >= 0
    assert all(stamp <= receive for _, receive, stamp in results)
    stamps = [stamp for _, _, stamp in results]
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    assert mapper.resets == 0


def test_delayed_batch_is_not_mistaken_for_a_clock_step():
    mapper = DeviceClockMapper()
    samples = list(stream(1000))
    batched = []
    for device_us, true_ns, receive_ns in samples:
        # A 345 ms outage: samples 500..517 all arrive together at its end.
        if 500 <= (device_us - 10_000_000) // PERIOD_US < 518:
            receive_ns = samples[517][2] + 1_000_000
        batched.append((device_us, true_ns, receive_ns))
    results = run(mapper, batched)
    assert mapper.resets == 0
    for true_ns, receive_ns, stamp_ns in results:
        assert stamp_ns - true_ns == BASE_LATENCY_NS
        assert stamp_ns <= receive_ns


def test_long_outage_flush_is_not_a_forward_step():
    mapper = DeviceClockMapper()
    samples = list(stream(1000))
    flushed = []
    for device_us, true_ns, receive_ns in samples:
        # A 1.5 s outage (75 samples) flushed within a few milliseconds.
        index = (device_us - 10_000_000) // PERIOD_US
        if 400 <= index < 475:
            receive_ns = samples[475][2] + (index - 400) * 20_000
        flushed.append((device_us, true_ns, receive_ns))
    run(mapper, flushed)
    assert mapper.resets == 0


def test_forward_jetson_clock_step_is_followed_within_the_hold():
    mapper = DeviceClockMapper()
    step_ns = int(21.4 * 3600e9)
    samples = list(stream(1000))
    stepped = [(d, t + step_ns, r + step_ns) if i >= 300 else (d, t, r)
               for i, (d, t, r) in enumerate(samples)]
    results = run(mapper, stepped)
    assert mapper.resets == 1
    assert mapper.last_reset_reason == "jetson_clock_stepped_forward"
    # 0.5 s hold at 50 Hz: about 25 samples keep the old offset.
    late = [i for i, (true, _, stamp) in enumerate(results)
            if abs(stamp - true - BASE_LATENCY_NS) > 1_000_000]
    assert late and late[0] == 300 and late[-1] <= 330
    assert all(stamp <= receive for _, receive, stamp in results)


def test_backward_jetson_clock_step_resets_at_once():
    mapper = DeviceClockMapper()
    samples = list(stream(600))
    stepped = [(d, t - 2_000_000_000, r - 2_000_000_000) if i >= 300
               else (d, t, r) for i, (d, t, r) in enumerate(samples)]
    results = run(mapper, stepped)
    assert mapper.resets == 1
    assert mapper.last_reset_reason == "jetson_clock_stepped_back"
    for true_ns, receive_ns, stamp_ns in results[300:]:
        assert stamp_ns - true_ns == BASE_LATENCY_NS
        assert stamp_ns <= receive_ns


def test_esp32_reboot_and_device_clock_restart_reset():
    mapper = DeviceClockMapper()
    run(mapper, stream(100))
    # Same boot, device clock restarted near zero: a different offset.
    restarted = list(stream(100, start_us=1_000, offset_ns=OFFSET_NS + 7 * 10**9))
    run(mapper, restarted)
    assert mapper.resets == 1
    assert mapper.last_reset_reason == "device_time_regressed"
    rebooted = list(stream(100, start_us=1_000, offset_ns=OFFSET_NS + 9 * 10**9))
    results = run(mapper, rebooted, boot_id=2)
    assert mapper.resets == 2
    assert mapper.last_reset_reason == "boot_id_changed"
    for true_ns, _, stamp_ns in results:
        assert stamp_ns - true_ns == BASE_LATENCY_NS


def test_duplicates_and_reordered_packets_do_not_disturb_the_offset():
    mapper = DeviceClockMapper()
    samples = list(stream(200))
    run(mapper, samples[:100])
    offset = mapper.offset_ns
    device_us, _, receive_ns = samples[99]
    duplicate = mapper.update(1, device_us, receive_ns + 5_000_000)
    older = mapper.update(1, samples[95][0], receive_ns + 6_000_000)
    assert mapper.offset_ns == offset
    assert duplicate == device_us * 1000 + offset
    assert older == samples[95][0] * 1000 + offset
    assert mapper.resets == 0


def test_hundred_ppm_drift_is_tracked_within_a_millisecond():
    for drift in (100.0, -100.0):
        mapper = DeviceClockMapper()
        results = run(mapper, stream(6000, drift_ppm=drift))
        errors = [stamp - true - BASE_LATENCY_NS for true, _, stamp in results]
        assert max(abs(error) for error in errors) <= 1_000_000
        assert all(stamp <= receive for _, receive, stamp in results)
        assert mapper.resets == 0


def test_stamp_mode_resolution_keeps_the_legacy_flag():
    assert resolve_stamp_mode("", True) == "arrival"
    assert resolve_stamp_mode("", False) == "esp32_header"
    assert resolve_stamp_mode("device_clock", True) == "device_clock"
    assert resolve_stamp_mode("esp32_header", True) == "esp32_header"
    with pytest.raises(ValueError):
        resolve_stamp_mode("jetson", False)
