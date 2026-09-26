#!/usr/bin/env python3
"""Serve a phone control pad on Jetson and publish leased ROS 2 commands."""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from carbot_hardware.web_teleop_core import CommandLease


class WebState:
    def __init__(self, lease):
        self.lease = lease
        self._lock = threading.Lock()
        self._arm_request = None
        self.error = ""

    def request_arm(self, enabled):
        with self._lock:
            self._arm_request = bool(enabled)

    def take_arm_request(self):
        with self._lock:
            request = self._arm_request
            self._arm_request = None
            return request

    def status(self):
        with self._lock:
            return {"armed": self.lease.armed, "error": self.error}


def make_handler(state, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            return

        def _json(self, status, payload):
            data = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            length = int(self.headers.get("Content-Length", "0"))
            if length < 1 or length > 1024:
                raise ValueError("invalid request size")
            return json.loads(self.rfile.read(length))

        def do_GET(self):
            if self.path == "/":
                data = page
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            elif self.path == "/api/status":
                self._json(HTTPStatus.OK, state.status())
            else:
                self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

        def do_POST(self):
            try:
                body = self._body()
                if self.path == "/api/arm":
                    if not isinstance(body.get("enabled"), bool):
                        raise ValueError("enabled must be boolean")
                    state.request_arm(body["enabled"])
                elif self.path == "/api/command":
                    state.lease.submit(body["linear_x"], body["angular_z"])
                else:
                    self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
                    return
                self._json(HTTPStatus.OK, {"ok": True})
            except (KeyError, TypeError, ValueError) as error:
                self._json(HTTPStatus.BAD_REQUEST, {"error": str(error)})
            except RuntimeError as error:
                self._json(HTTPStatus.CONFLICT, {"error": str(error)})

    return Handler


class WebTeleop(Node):
    def __init__(self):
        super().__init__("carbot_web_teleop")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("output_topic", "/cmd_vel_command")
        self.declare_parameter("linear_speed_mps", 0.10)
        self.declare_parameter("angular_speed_rps", 0.40)
        self.declare_parameter("command_timeout_s", 0.30)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._topic = self.get_parameter("output_topic").value
        linear = self.get_parameter("linear_speed_mps").value
        angular = self.get_parameter("angular_speed_rps").value
        timeout = self.get_parameter("command_timeout_s").value
        rate = self.get_parameter("publish_rate_hz").value
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._lease = CommandLease(linear, angular, timeout)
        self._state = WebState(self._lease)
        self._publisher = self.create_publisher(Twist, self._topic, 10)
        self._zero_pending = False
        self._timer = self.create_timer(1.0 / rate, self._tick)

        address = self.get_parameter("bind_address").value
        port = self.get_parameter("port").value
        page_path = os.path.join(
            get_package_share_directory("carbot_hardware"),
            "config",
            "web_teleop.html",
        )
        with open(page_path, "rb") as stream:
            page = stream.read()
        self._server = ThreadingHTTPServer(
            (address, port), make_handler(self._state, page)
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._thread.start()
        self.get_logger().info(
            f"phone controller: http://{address}:{port} -> {self._topic}"
        )

    def _safe_topology(self):
        return (
            self.count_publishers(self._topic) == 1
            and self._publisher.get_subscription_count() == 1
        )

    def _publish(self, linear_x=0.0, angular_z=0.0):
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self._publisher.publish(message)

    def _tick(self):
        request = self._state.take_arm_request()
        if request is False:
            self._lease.set_armed(False)
            self._state.error = ""
            self._zero_pending = True
        elif request is True:
            if self._safe_topology():
                self._lease.set_armed(True)
                self._state.error = ""
            else:
                self._lease.set_armed(False)
                self._state.error = "使能失败：要求唯一 Web 发布者和唯一速度补偿订阅者"
                self._zero_pending = True

        if self._lease.armed and not self._safe_topology():
            self._lease.set_armed(False)
            self._state.error = "已自动停用：ROS 2 控制拓扑发生变化"
            self._zero_pending = True

        if self._lease.armed:
            command = self._lease.sample()
            self._publish(command.linear_x, command.angular_z)
        elif self._zero_pending:
            self._publish()
            self._zero_pending = False

    def close(self):
        self._lease.set_armed(False)
        for _ in range(10):
            self._publish()
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
