"""Publish Livox MID-360 acceleration using ROS SI units."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .imu_units import (
    STANDARD_GRAVITY_MPS2,
    acceleration_g_to_mps2,
    scale_covariance,
)


class Mid360ImuAdapter(Node):
    """Convert the Livox driver's g-valued acceleration to m/s^2."""

    def __init__(self):
        super().__init__("mid360_imu_adapter")
        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/mid360/imu/data_raw")
        self.declare_parameter("gravity_mps2", STANDARD_GRAVITY_MPS2)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._gravity_mps2 = float(self.get_parameter("gravity_mps2").value)
        if self._gravity_mps2 <= 0.0:
            raise ValueError("gravity_mps2 must be positive")

        self._publisher = self.create_publisher(
            Imu, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            Imu, input_topic, self._on_imu, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"converting MID-360 acceleration from g to m/s^2: "
            f"{input_topic} -> {output_topic}"
        )

    def _on_imu(self, source):
        converted = Imu()
        converted.header = source.header

        # The Livox ROS driver does not publish an orientation estimate. Marking
        # it unavailable prevents consumers from interpreting its identity
        # quaternion and all-zero covariance as a measured orientation.
        converted.orientation.w = 1.0
        converted.orientation_covariance[0] = -1.0

        converted.angular_velocity = source.angular_velocity
        converted.angular_velocity_covariance = source.angular_velocity_covariance

        acceleration = acceleration_g_to_mps2(
            (
                source.linear_acceleration.x,
                source.linear_acceleration.y,
                source.linear_acceleration.z,
            ),
            self._gravity_mps2,
        )
        converted.linear_acceleration.x = acceleration[0]
        converted.linear_acceleration.y = acceleration[1]
        converted.linear_acceleration.z = acceleration[2]
        converted.linear_acceleration_covariance = scale_covariance(
            source.linear_acceleration_covariance, self._gravity_mps2
        )
        self._publisher.publish(converted)


def main(args=None):
    rclpy.init(args=args)
    node = Mid360ImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
