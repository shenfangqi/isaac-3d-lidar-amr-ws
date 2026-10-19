// C++ port of carbot_hardware/mid360_imu_adapter.py: the same node name,
// parameters, topics and output.  On the Jetson the Python node used about
// half a core for 200 Hz IMU messages (2026-10-19), nearly all of it rclpy
// per-message overhead.
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "carbot_hardware_cpp/imu_units.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace carbot_hardware_cpp
{

// Convert the Livox driver's g-valued acceleration to m/s^2.
class Mid360ImuAdapter : public rclcpp::Node
{
public:
  Mid360ImuAdapter()
  : rclcpp::Node("mid360_imu_adapter"),
    gyro_z_bias_(
      declare_parameter<int64_t>("gyro_z_bias_min_samples", 400),
      declare_parameter<int64_t>("gyro_z_bias_window_samples", 2000),
      declare_parameter<double>("gyro_z_bias_max_abs_rad_s", 0.03))
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox/imu");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/mid360/imu/data_raw");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "imu_link");
    gravity_mps2_ = declare_parameter<double>("gravity_mps2", kStandardGravityMps2);
    timestamp_correction_s_ = declare_parameter<double>("timestamp_correction_s", 0.0);
    const auto wheel_odom_topic = declare_parameter<std::string>(
      "wheel_odom_topic", "/wheel/odom");
    stationary_linear_threshold_ = declare_parameter<double>(
      "stationary_linear_threshold_mps", 0.002);
    stationary_angular_threshold_ = declare_parameter<double>(
      "stationary_angular_threshold_rad_s", 0.01);
    const auto diagonal = declare_parameter<std::vector<double>>(
      "angular_velocity_covariance_diagonal", {1.0e-4, 1.0e-4, 4.0e-6});
    gyro_z_uncalibrated_variance_ = declare_parameter<double>(
      "gyro_z_uncalibrated_variance", 1.0e-2);

    if (!(gravity_mps2_ > 0.0)) {
      throw std::invalid_argument("gravity_mps2 must be positive");
    }
    if (output_frame_id_.empty()) {
      throw std::invalid_argument("output_frame_id must not be empty");
    }
    // A one-second bound catches unit/sign configuration mistakes while
    // leaving ample room for sensor timestamp calibration.
    if (!std::isfinite(timestamp_correction_s_)) {
      throw std::invalid_argument("timestamp_correction_s must be finite");
    }
    if (std::abs(timestamp_correction_s_) > 1.0) {
      throw std::invalid_argument("abs(timestamp_correction_s) must not exceed 1 s");
    }
    if (!(stationary_linear_threshold_ > 0.0)) {
      throw std::invalid_argument("stationary_linear_threshold_mps must be positive");
    }
    if (!(stationary_angular_threshold_ > 0.0)) {
      throw std::invalid_argument("stationary_angular_threshold_rad_s must be positive");
    }
    if (!(gyro_z_uncalibrated_variance_ > 0.0)) {
      throw std::invalid_argument("gyro_z_uncalibrated_variance must be positive");
    }
    if (diagonal.size() != 3) {
      throw std::invalid_argument(
        "covariance must be 3x3 and diagonal must have 3 values");
    }
    angular_velocity_covariance_diagonal_ = {diagonal[0], diagonal[1], diagonal[2]};
    covariance_with_fallback_diagonal({}, angular_velocity_covariance_diagonal_);

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(*message);});
    wheel_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic, 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_wheel_odom(*message);});
    RCLCPP_INFO(
      get_logger(),
      "converting MID-360 acceleration from g to m/s^2: %s -> %s; frame=%s; "
      "timestamp correction=%+.9f s",
      input_topic.c_str(), output_topic.c_str(), output_frame_id_.c_str(),
      timestamp_correction_s_);
  }

private:
  void on_wheel_odom(const nav_msgs::msg::Odometry & message)
  {
    wheel_stationary_ =
      std::abs(message.twist.twist.linear.x) <= stationary_linear_threshold_ &&
      std::abs(message.twist.twist.angular.z) <= stationary_angular_threshold_;
  }

  void on_imu(const sensor_msgs::msg::Imu & source)
  {
    auto converted = std::make_unique<sensor_msgs::msg::Imu>();
    const auto stamp = shift_ros_stamp(
      source.header.stamp.sec, source.header.stamp.nanosec, timestamp_correction_s_);
    converted->header.stamp.sec = static_cast<int32_t>(stamp.first);
    converted->header.stamp.nanosec = static_cast<uint32_t>(stamp.second);
    converted->header.frame_id = output_frame_id_;

    // The Livox ROS driver does not publish an orientation estimate. Marking
    // it unavailable prevents consumers from interpreting its identity
    // quaternion and all-zero covariance as a measured orientation.
    converted->orientation.w = 1.0;
    converted->orientation_covariance[0] = -1.0;

    converted->angular_velocity.x = source.angular_velocity.x;
    converted->angular_velocity.y = source.angular_velocity.y;
    converted->angular_velocity.z = gyro_z_bias_.update(
      source.angular_velocity.z, wheel_stationary_);
    auto angular_covariance = covariance_with_fallback_diagonal(
      source.angular_velocity_covariance, angular_velocity_covariance_diagonal_);
    if (!gyro_z_bias_.ready()) {
      angular_covariance[8] = std::max(angular_covariance[8], gyro_z_uncalibrated_variance_);
    }
    converted->angular_velocity_covariance = angular_covariance;
    if (gyro_z_bias_.ready() && !bias_ready_logged_) {
      RCLCPP_INFO(
        get_logger(), "MID-360 gyro z bias ready after %zu stationary samples: %+.9f rad/s",
        gyro_z_bias_.sample_count(), gyro_z_bias_.bias());
      bias_ready_logged_ = true;
    }

    converted->linear_acceleration.x = source.linear_acceleration.x * gravity_mps2_;
    converted->linear_acceleration.y = source.linear_acceleration.y * gravity_mps2_;
    converted->linear_acceleration.z = source.linear_acceleration.z * gravity_mps2_;
    converted->linear_acceleration_covariance = scale_covariance(
      source.linear_acceleration_covariance, gravity_mps2_);
    publisher_->publish(std::move(converted));
  }

  StationaryBiasEstimator gyro_z_bias_;
  std::string output_frame_id_;
  double gravity_mps2_{};
  double timestamp_correction_s_{};
  double stationary_linear_threshold_{};
  double stationary_angular_threshold_{};
  double gyro_z_uncalibrated_variance_{};
  std::array<double, 3> angular_velocity_covariance_diagonal_{};
  bool wheel_stationary_{false};
  bool bias_ready_logged_{false};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
};

}  // namespace carbot_hardware_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<carbot_hardware_cpp::Mid360ImuAdapter>());
  rclcpp::shutdown();
  return 0;
}
