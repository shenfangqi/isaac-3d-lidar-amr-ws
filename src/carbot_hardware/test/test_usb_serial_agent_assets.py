"""Static checks for the Jetson USB serial micro-ROS deployment assets."""

import os
from pathlib import Path
import re
import subprocess
import threading
import time
from unittest import SkipTest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
DEVICE = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)


def test_serial_agent_service_uses_stable_device_and_expected_baudrate():
    service = (
        PACKAGE_DIR / "systemd/micro-ros-agent.service"
    ).read_text(encoding="utf-8")

    assert f"CARBOT_MICROROS_SERIAL_DEVICE={DEVICE}" in service
    assert "CARBOT_MICROROS_BAUDRATE=921600" in service
    assert "start_micro_ros_agent_serial.sh" in service
    assert "Restart=always" in service
    assert "udp4" not in service
    assert "network-online.target" not in service


def test_serial_agent_launcher_never_falls_back_to_udp():
    launcher = (
        PACKAGE_DIR / "scripts/start_micro_ros_agent_serial.sh"
    ).read_text(encoding="utf-8")

    assert "micro_ros_agent micro_ros_agent serial" in launcher
    assert '--dev "${device}"' in launcher
    assert '--baudrate "${baudrate}"' in launcher
    assert "readlink -f" in launcher
    assert "-r \"${device}\" && -w \"${device}\"" in launcher
    assert "udp4" not in launcher


def test_preflight_rejects_wrong_usb_identity_and_legacy_udp_agent():
    preflight_path = WORKSPACE_DIR / "scripts/jetson_nav_preflight.sh"
    if not preflight_path.is_file():
        raise SkipTest("partial deployment has no repository-level preflight")
    preflight = preflight_path.read_text(encoding="utf-8")

    assert DEVICE in preflight
    assert "ID_VENDOR_ID=10c4" in preflight
    assert "ID_MODEL_ID=ea60" in preflight
    assert "ID_SERIAL_SHORT=0001" in preflight
    assert "legacy UDP micro-ROS Agent" in preflight


def test_setup_installs_serial_agent_launcher():
    setup_source = (PACKAGE_DIR / "setup.py").read_text(encoding="utf-8")
    assert '"share/" + package_name + "/scripts"' in setup_source
    assert 'glob("scripts/*.sh")' in setup_source


WAIT_SCRIPT = PACKAGE_DIR / "scripts/wait_for_time_sync.sh"


def _run_wait(tmp_path, timeout, ntp="no", flag_after=None):
    """Run the NTP wait with a temporary flag path and a stub timedatectl."""
    stub_dir = tmp_path / "bin"
    stub_dir.mkdir(exist_ok=True)
    stub = stub_dir / "timedatectl"
    stub.write_text(f"#!/bin/sh\necho {ntp}\n", encoding="utf-8")
    stub.chmod(0o755)
    flag = tmp_path / "synchronized"
    environment = dict(
        os.environ,
        PATH=f"{stub_dir}:{os.environ['PATH']}",
        CARBOT_TIME_SYNC_FLAG=str(flag),
        CARBOT_MICROROS_TIME_SYNC_WAIT_SECONDS=str(timeout),
    )
    timer = None
    if flag_after is not None:
        timer = threading.Timer(flag_after, flag.touch)
        timer.start()
    started = time.monotonic()
    try:
        result = subprocess.run(
            ["bash", str(WAIT_SCRIPT)], env=environment,
            capture_output=True, text=True, timeout=30, check=False)
    finally:
        if timer is not None:
            timer.cancel()
    return result, time.monotonic() - started


def test_time_sync_wait_returns_at_once_when_this_boot_is_synchronized(
        tmp_path):
    (tmp_path / "synchronized").touch()
    result, elapsed = _run_wait(tmp_path, 90)
    assert result.returncode == 0
    assert "Clock synchronized after 0 s" in result.stdout
    assert elapsed < 2.0


def test_time_sync_wait_accepts_timedatectl_without_the_flag(tmp_path):
    result, elapsed = _run_wait(tmp_path, 90, ntp="yes")
    assert result.returncode == 0
    assert "Clock synchronized after 0 s" in result.stdout
    assert elapsed < 2.0


def test_time_sync_wait_continues_when_sync_arrives(tmp_path):
    result, elapsed = _run_wait(tmp_path, 20, flag_after=1.5)
    assert result.returncode == 0
    assert "Waiting for NTP synchronization" in result.stderr
    assert re.search(r"Clock synchronized after [1-4] s", result.stdout)
    assert elapsed < 6.0


def test_time_sync_wait_warns_and_starts_anyway_on_timeout(tmp_path):
    result, elapsed = _run_wait(tmp_path, 2)
    assert result.returncode == 0
    assert "WARNING: NTP not synchronized after 2 s" in result.stderr
    assert "issue #95" in result.stderr
    assert 2.0 <= elapsed < 6.0


def test_time_sync_wait_can_be_skipped_and_rejects_bad_values(tmp_path):
    result, elapsed = _run_wait(tmp_path, 0)
    assert result.returncode == 0
    assert "Skipping the NTP wait" in result.stdout
    assert elapsed < 2.0
    for value in ("abc", "-1", "1.5"):
        result, _ = _run_wait(tmp_path, value)
        assert result.returncode == 64, value


def test_launcher_waits_for_time_sync_before_the_esp32_session():
    launcher = (
        PACKAGE_DIR / "scripts/start_micro_ros_agent_serial.sh"
    ).read_text(encoding="utf-8")
    wait_call = launcher.index("wait_for_time_sync.sh")
    assert wait_call < launcher.index("Waiting for readable/writable")
    assert wait_call < launcher.index("exec ros2 run micro_ros_agent")
    assert os.access(WAIT_SCRIPT, os.X_OK)
    assert "cmd_vel" not in WAIT_SCRIPT.read_text(encoding="utf-8")


def test_time_sync_wait_counts_with_the_monotonic_boot_clock():
    # NTP steps the wall clock during the wait; SECONDS would follow it.
    source = WAIT_SCRIPT.read_text(encoding="utf-8")
    assert "/proc/uptime" in source
    assert not re.search(r"\$\{SECONDS\}|\$\(\(\s*SECONDS", source)
