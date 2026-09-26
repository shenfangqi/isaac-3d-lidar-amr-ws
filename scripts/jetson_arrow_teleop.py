#!/usr/bin/env python3
"""Safety-bounded arrow-key teleoperation for the physical Carbot."""

import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy


LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.40
KEY_TIMEOUT = 0.25
PUBLISH_PERIOD = 0.05

ARROWS = {
    b"\x1b[A": (LINEAR_SPEED, 0.0, "forward"),
    b"\x1b[B": (-LINEAR_SPEED, 0.0, "reverse"),
    b"\x1b[D": (0.0, ANGULAR_SPEED, "left"),
    b"\x1b[C": (0.0, -ANGULAR_SPEED, "right"),
}


def publish_command(publisher, linear: float, angular: float) -> None:
    message = Twist()
    message.linear.x = linear
    message.angular.z = angular
    publisher.publish(message)


def main() -> None:
    if not sys.stdin.isatty():
        raise SystemExit("arrow teleop requires an interactive terminal")

    rclpy.init()
    node = rclpy.create_node("carbot_arrow_teleop")
    publisher = node.create_publisher(Twist, "/cmd_vel_command", 10)

    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (publisher.get_subscription_count() == 1
                and node.count_publishers("/cmd_vel_command") == 1):
            break
    else:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            "control preflight failed: expected one command compensator "
            "subscriber and no other teleop publisher"
        )

    print(
        "Carbot Wi-Fi arrow teleop ready\n"
        "  Up/Down    : forward/reverse at 0.10 m/s\n"
        "  Left/Right : turn left/right at 0.40 rad/s\n"
        "  Space      : immediate stop\n"
        "  Q or Ctrl-C: stop and quit\n"
        "Commands automatically stop 0.25 s after key release.\n",
        flush=True,
    )

    fd = sys.stdin.fileno()
    previous_settings = termios.tcgetattr(fd)
    command = (0.0, 0.0)
    command_deadline = 0.0
    last_label = "stop"
    pending = b""

    try:
        tty.setraw(fd)
        while rclpy.ok():
            readable, _, _ = select.select([fd], [], [], PUBLISH_PERIOD)
            if readable:
                pending += os.read(fd, 32)

            quit_requested = b"q" in pending.lower() or b"\x03" in pending
            if quit_requested:
                break

            if b" " in pending:
                command = (0.0, 0.0)
                command_deadline = 0.0
                if last_label != "stop":
                    os.write(fd, b"\rSTOP                         \r")
                    last_label = "stop"
                pending = pending.replace(b" ", b"")

            matched = False
            for sequence, (linear, angular, label) in ARROWS.items():
                if sequence in pending:
                    command = (linear, angular)
                    command_deadline = time.monotonic() + KEY_TIMEOUT
                    if label != last_label:
                        text = f"\r{label.upper():<12} v={linear:+.2f} w={angular:+.2f}"
                        os.write(fd, text.encode())
                        last_label = label
                    pending = pending.replace(sequence, b"")
                    matched = True

            # Drop incomplete/non-arrow input without ever converting it to
            # motion. Preserve a trailing ESC prefix for the next read.
            if not matched and pending not in (b"", b"\x1b", b"\x1b["):
                pending = b""
            elif len(pending) > 2:
                pending = pending[-2:]

            if time.monotonic() >= command_deadline:
                command = (0.0, 0.0)
                if last_label != "stop":
                    os.write(fd, b"\rSTOP                         \r")
                    last_label = "stop"

            publish_command(publisher, *command)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        for _ in range(20):
            publish_command(publisher, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.01)
        termios.tcsetattr(fd, termios.TCSADRAIN, previous_settings)
        print("\nStopped. Zero command sent.", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
