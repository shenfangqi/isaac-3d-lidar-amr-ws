#!/usr/bin/env python3
"""Apply the calibrated right-turn correction before commands reach ESP32."""

from copy import deepcopy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import yaml

from carbot_hardware.command_compensation import compensate_angular_z


class CmdVelCompensator(Node):
    def __init__(self):
        super().__init__("cmd_vel_compensator")
        default_path = (
            get_package_share_directory("carbot_description")
            + "/config/carbot_parameters.yaml"
        )
        self.declare_parameter("canonical_parameters_path", default_path)
        self.declare_parameter("input_topic", "/cmd_vel_command")
        self.declare_parameter("output_topic", "/cmd_vel")

        parameter_path = self.get_parameter(
            "canonical_parameters_path"
        ).value
        with open(parameter_path, encoding="utf-8") as stream:
            parameters = yaml.safe_load(stream)
        self.right_turn_scale = float(
            parameters["control"]["right_turn_command_scale"]
        )
        compensate_angular_z(0.0, self.right_turn_scale)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        if input_topic == output_topic:
            raise ValueError("input_topic and output_topic must differ")
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, input_topic, self.command_callback, 10
        )
        self.get_logger().info(
            f"right-turn scale={self.right_turn_scale:.3f}: "
            f"{input_topic} -> {output_topic}"
        )

    def command_callback(self, message):
        output = deepcopy(message)
        try:
            output.angular.z = compensate_angular_z(
                message.angular.z, self.right_turn_scale
            )
        except ValueError as error:
            self.get_logger().error(
                f"rejecting invalid velocity command: {error}"
            )
            output = Twist()
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelCompensator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
