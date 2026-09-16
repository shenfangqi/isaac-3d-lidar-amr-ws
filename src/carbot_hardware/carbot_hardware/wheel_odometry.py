"""Convert cumulative Carbot wheel ticks into the sole hardware odometry TF."""

import math

from carbot_msgs.msg import CarbotStatus, WheelTicks
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .odometry_math import Pose2D, integrate_tick_delta


def _yaw_quaternion(yaw_rad):
    half_yaw = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def _covariance(x_variance, y_variance, yaw_variance):
    covariance = [0.0] * 36
    covariance[0] = x_variance
    covariance[7] = y_variance
    covariance[14] = 1.0e6
    covariance[21] = 1.0e6
    covariance[28] = 1.0e6
    covariance[35] = yaw_variance
    return covariance


class WheelOdometry(Node):
    """Integrate cumulative ticks while handling reconnects and duplicate frames."""

    def __init__(self):
        super().__init__("carbot_wheel_odometry")

        self.declare_parameter("wheel_ticks_topic", "/wheel_ticks")
        self.declare_parameter("status_topic", "/carbot/status")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("counts_per_revolution", 1560.0)
        self.declare_parameter("effective_sprocket_radius_m", 0.02175)
        self.declare_parameter("effective_track_separation_m", 0.254)
        self.declare_parameter("max_wheel_rpm", 250.0)
        self.declare_parameter("tick_rate_margin", 1.25)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("require_time_synchronized", True)
        self.declare_parameter("pose_x_variance", 0.0025)
        self.declare_parameter("pose_y_variance", 0.01)
        self.declare_parameter("pose_yaw_variance", 0.01)
        self.declare_parameter("twist_x_variance", 0.01)
        self.declare_parameter("twist_y_variance", 0.04)
        self.declare_parameter("twist_yaw_variance", 0.04)

        counts_per_revolution = float(
            self.get_parameter("counts_per_revolution").value
        )
        sprocket_radius_m = float(
            self.get_parameter("effective_sprocket_radius_m").value
        )
        if counts_per_revolution <= 0.0 or sprocket_radius_m <= 0.0:
            raise ValueError("wheel radius and encoder resolution must be positive")

        self._meters_per_tick = (
            2.0 * math.pi * sprocket_radius_m / counts_per_revolution
        )
        self._counts_per_revolution = counts_per_revolution
        self._track_separation_m = float(
            self.get_parameter("effective_track_separation_m").value
        )
        self._max_wheel_rpm = float(self.get_parameter("max_wheel_rpm").value)
        self._tick_rate_margin = float(
            self.get_parameter("tick_rate_margin").value
        )
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._require_time_synchronized = bool(
            self.get_parameter("require_time_synchronized").value
        )

        self._pose_covariance = _covariance(
            float(self.get_parameter("pose_x_variance").value),
            float(self.get_parameter("pose_y_variance").value),
            float(self.get_parameter("pose_yaw_variance").value),
        )
        self._twist_covariance = _covariance(
            float(self.get_parameter("twist_x_variance").value),
            float(self.get_parameter("twist_y_variance").value),
            float(self.get_parameter("twist_yaw_variance").value),
        )

        self._pose = Pose2D()
        self._previous_ticks = None
        self._time_synchronized = not self._require_time_synchronized
        self._duplicate_count = 0
        self._rejected_count = 0

        self._odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._status_subscription = self.create_subscription(
            CarbotStatus,
            str(self.get_parameter("status_topic").value),
            self._status_callback,
            qos_profile_sensor_data,
        )
        self._ticks_subscription = self.create_subscription(
            WheelTicks,
            str(self.get_parameter("wheel_ticks_topic").value),
            self._ticks_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "wheel odometry ready: %.9f m/tick, track %.3f m"
            % (self._meters_per_tick, self._track_separation_m)
        )

    def _status_callback(self, msg):
        was_synchronized = self._time_synchronized
        self._time_synchronized = bool(msg.time_synchronized)
        if was_synchronized and not self._time_synchronized:
            self._previous_ticks = None
            self.get_logger().error(
                "ESP32 time synchronization lost; odometry publication paused"
            )
        elif not was_synchronized and self._time_synchronized:
            self._previous_ticks = None
            self.get_logger().info(
                "ESP32 time synchronized; resetting the tick delta baseline"
            )

    def _ticks_callback(self, msg):
        if not self._time_synchronized:
            return

        if self._previous_ticks is None:
            self._previous_ticks = msg
            self._publish(msg, 0.0, 0.0)
            return

        previous = self._previous_ticks
        if msg.boot_id != previous.boot_id:
            self._previous_ticks = msg
            self.get_logger().warning(
                "ESP32 boot_id changed; preserving pose and resetting tick baseline"
            )
            self._publish(msg, 0.0, 0.0)
            return

        if msg.sequence <= previous.sequence or msg.device_stamp_us <= previous.device_stamp_us:
            self._duplicate_count += 1
            return

        dt_s = (msg.device_stamp_us - previous.device_stamp_us) * 1.0e-6
        left_delta = msg.left_ticks - previous.left_ticks
        right_delta = msg.right_ticks - previous.right_ticks
        maximum_ticks = (
            self._max_wheel_rpm
            * self._counts_per_revolution
            * dt_s
            / 60.0
            * self._tick_rate_margin
            + 2.0
        )
        if abs(left_delta) > maximum_ticks or abs(right_delta) > maximum_ticks:
            self._rejected_count += 1
            self._previous_ticks = msg
            self.get_logger().error(
                "rejected implausible tick jump: left=%d right=%d dt=%.3f s"
                % (left_delta, right_delta, dt_s)
            )
            return

        motion = integrate_tick_delta(
            self._pose,
            left_delta,
            right_delta,
            self._meters_per_tick,
            self._track_separation_m,
        )
        self._pose = motion.pose
        self._previous_ticks = msg
        self._publish(msg, motion.distance_m / dt_s, motion.yaw_rad / dt_s)

    def _publish(self, ticks, linear_mps, angular_rps):
        quaternion = _yaw_quaternion(self._pose.yaw)
        odom = Odometry()
        odom.header.stamp = ticks.header.stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._pose.x
        odom.pose.pose.position.y = self._pose.y
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.pose.covariance = self._pose_covariance
        odom.twist.twist.linear.x = linear_mps
        odom.twist.twist.angular.z = angular_rps
        odom.twist.covariance = self._twist_covariance
        self._odom_publisher.publish(odom)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = self._pose.x
            transform.transform.translation.y = self._pose.y
            transform.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
