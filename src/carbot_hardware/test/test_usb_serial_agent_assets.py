"""Static checks for the Jetson USB serial micro-ROS deployment assets."""

from pathlib import Path
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
