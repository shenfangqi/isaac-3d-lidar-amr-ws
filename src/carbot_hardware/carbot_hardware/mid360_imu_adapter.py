"""Publish Livox MID-360 acceleration using ROS SI units."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .imu_units import (
    STANDARD_GRAVITY_MPS2,
    StationaryBiasEstimator,
    acceleration_g_to_mps2,
    covariance_with_fallback_diagonal,
    scale_covariance,
    shift_ros_stamp,
)


class Mid360ImuAdapter(Node):
    """Convert the Livox driver's g-valued acceleration to m/s^2."""

    def __init__(self):
        super().__init__("mid360_imu_adapter")
        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/mid360/imu/data_raw")
        self.declare_parameter("output_frame_id", "imu_link")
        self.declare_parameter("gravity_mps2", STANDARD_GRAVITY_MPS2)
        self.declare_parameter("timestamp_correction_s", 0.0)
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("stationary_linear_threshold_mps", 0.002)
        self.declare_parameter("stationary_angular_threshold_rad_s", 0.01)
        self.declare_parameter("gyro_z_bias_min_samples", 400)
        self.declare_parameter("gyro_z_bias_window_samples", 2000)
        self.declare_parameter("gyro_z_bias_max_abs_rad_s", 0.03)
        self.declare_parameter(
            "angular_velocity_covariance_diagonal",
            [1.0e-4, 1.0e-4, 4.0e-6],
        )
        self.declare_parameter("gyro_z_uncalibrated_variance", 1.0e-2)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._output_frame_id = str(
            self.get_parameter("output_frame_id").value
        )
        self._gravity_mps2 = float(self.get_parameter("gravity_mps2").value)
        self._timestamp_correction_s = float(
            self.get_parameter("timestamp_correction_s").value
        )
        self._stationary_linear_threshold = float(
            self.get_parameter("stationary_linear_threshold_mps").value
        )
        self._stationary_angular_threshold = float(
            self.get_parameter("stationary_angular_threshold_rad_s").value
        )
        self._angular_velocity_covariance_diagonal = tuple(
            float(value)
            for value in self.get_parameter(
                "angular_velocity_covariance_diagonal"
            ).value
        )
        self._gyro_z_uncalibrated_variance = float(
            self.get_parameter("gyro_z_uncalibrated_variance").value
        )
        self._gyro_z_bias = StationaryBiasEstimator(
            int(self.get_parameter("gyro_z_bias_min_samples").value),
            int(self.get_parameter("gyro_z_bias_window_samples").value),
            float(self.get_parameter("gyro_z_bias_max_abs_rad_s").value),
        )
        self._wheel_stationary = False
        self._bias_ready_logged = False
        if self._gravity_mps2 <= 0.0:
            raise ValueError("gravity_mps2 must be positive")
        if not self._output_frame_id:
            raise ValueError("output_frame_id must not be empty")
        # A one-second bound catches unit/sign configuration mistakes while
        # leaving ample room for sensor timestamp calibration.
        if not math.isfinite(self._timestamp_correction_s):
            raise ValueError("timestamp_correction_s must be finite")
        if abs(self._timestamp_correction_s) > 1.0:
            raise ValueError("abs(timestamp_correction_s) must not exceed 1 s")
        if self._stationary_linear_threshold <= 0.0:
            raise ValueError("stationary_linear_threshold_mps must be positive")
        if self._stationary_angular_threshold <= 0.0:
            raise ValueError("stationary_angular_threshold_rad_s must be positive")
        if self._gyro_z_uncalibrated_variance <= 0.0:
            raise ValueError("gyro_z_uncalibrated_variance must be positive")
        covariance_with_fallback_diagonal(
            [0.0] * 9, self._angular_velocity_covariance_diagonal
        )

        self._publisher = self.create_publisher(
            Imu, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            Imu, input_topic, self._on_imu, qos_profile_sensor_data
        )
        self._wheel_odom_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_odom_topic").value),
            self._on_wheel_odom,
            10,
        )
        self.get_logger().info(
            f"converting MID-360 acceleration from g to m/s^2: "
            f"{input_topic} -> {output_topic}; frame={self._output_frame_id}; "
            f"timestamp correction={self._timestamp_correction_s:+.9f} s"
        )

    def _on_wheel_odom(self, message):
        self._wheel_stationary = (
            abs(message.twist.twist.linear.x)
            <= self._stationary_linear_threshold
            and abs(message.twist.twist.angular.z)
            <= self._stationary_angular_threshold
        )

    def _on_imu(self, source):
        converted = Imu()
        corrected_sec, corrected_nanosec = shift_ros_stamp(
            source.header.stamp.sec,
            source.header.stamp.nanosec,
            self._timestamp_correction_s,
        )
        converted.header.stamp.sec = corrected_sec
        converted.header.stamp.nanosec = corrected_nanosec
        converted.header.frame_id = self._output_frame_id

        # The Livox ROS driver does not publish an orientation estimate. Marking
        # it unavailable prevents consumers from interpreting its identity
        # quaternion and all-zero covariance as a measured orientation.
        converted.orientation.w = 1.0
        converted.orientation_covariance[0] = -1.0

        converted.angular_velocity.x = source.angular_velocity.x
        converted.angular_velocity.y = source.angular_velocity.y
        converted.angular_velocity.z = self._gyro_z_bias.update(
            source.angular_velocity.z, self._wheel_stationary
        )
        angular_covariance = list(
            covariance_with_fallback_diagonal(
                source.angular_velocity_covariance,
                self._angular_velocity_covariance_diagonal,
            )
        )
        if not self._gyro_z_bias.ready:
            angular_covariance[8] = max(
                angular_covariance[8], self._gyro_z_uncalibrated_variance
            )
        converted.angular_velocity_covariance = angular_covariance
        if self._gyro_z_bias.ready and not self._bias_ready_logged:
            self.get_logger().info(
                "MID-360 gyro z bias ready after %d stationary samples: %+.9f rad/s"
                % (self._gyro_z_bias.sample_count, self._gyro_z_bias.bias)
            )
            self._bias_ready_logged = True

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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
