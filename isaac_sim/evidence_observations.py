"""ROS observations that mirror the physically measured Carbot interfaces."""

from pathlib import Path
import sys


WORKSPACE = Path("/workspace/ros-humble/isaac_3d_lidar_amr_ws")
MESSAGE_PATH = (
    WORKSPACE / "install/carbot_msgs/local/lib/python3.10/dist-packages"
)
if MESSAGE_PATH.exists():
    sys.path.insert(0, str(MESSAGE_PATH))

from carbot_msgs.msg import CarbotStatus, WheelTicks  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from isaac_sim.evidence_models import (  # noqa: E402
    EncoderObservationModel,
    ImuObservationModel,
)


class EvidenceObservationPublisher:
    """Publish encoder/status/MID-360 IMU messages for the real EKF path."""

    def __init__(self, node, profile, time_message):
        self._time_message = time_message
        self._encoder = EncoderObservationModel(profile["encoder"])
        self._imu = ImuObservationModel(profile["imu"])
        self._imu_period_s = 1.0 / profile["imu"]["publish_rate_hz"]
        self._next_imu_s = self._imu_period_s
        self._status_period_s = 0.5
        self._next_status_s = 0.0
        self._boot_id = 20260923
        self._wheel_publisher = node.create_publisher(
            WheelTicks, "/wheel_ticks", 10
        )
        self._status_publisher = node.create_publisher(
            CarbotStatus, "/carbot/status", 10
        )
        self._imu_publisher = node.create_publisher(
            Imu, profile["state_estimation"]["imu_topic"], 50
        )
        self._imu_covariance = profile["imu"][
            "gyro_z_covariance_rad2_s2"
        ]

    def update(self, simulation_time, command, dt_s):
        sample = self._encoder.update(
            command.left_wheel_rad_s, command.right_wheel_rad_s, dt_s
        )
        if sample is not None:
            message = WheelTicks()
            message.header.stamp = self._time_message(
                sample.device_stamp_us / 1.0e6
            )
            message.header.frame_id = "base_footprint"
            message.sequence = sample.sequence
            message.boot_id = self._boot_id
            message.device_stamp_us = sample.device_stamp_us
            message.left_ticks = sample.left_ticks
            message.right_ticks = sample.right_ticks
            self._wheel_publisher.publish(message)

        while self._next_imu_s <= simulation_time + 1.0e-12:
            message = Imu()
            message.header.stamp = self._time_message(self._next_imu_s)
            message.header.frame_id = "imu_link"
            message.orientation_covariance[0] = -1.0
            message.angular_velocity.z = self._imu.yaw_rate(
                command.applied_angular_rad_s
            )
            message.angular_velocity_covariance[8] = self._imu_covariance
            message.linear_acceleration.z = 9.80665
            self._imu_publisher.publish(message)
            self._next_imu_s += self._imu_period_s

        if simulation_time + 1.0e-12 >= self._next_status_s:
            message = CarbotStatus()
            message.header.stamp = self._time_message(simulation_time)
            message.header.frame_id = "base_footprint"
            message.sequence = round(
                simulation_time / self._status_period_s
            )
            message.boot_id = self._boot_id
            message.device_stamp_us = round(simulation_time * 1.0e6)
            message.agent_connected = True
            message.time_synchronized = True
            message.imu_ready = False
            message.session_uptime_ms = round(simulation_time * 1.0e3)
            self._status_publisher.publish(message)
            self._next_status_s += self._status_period_s
