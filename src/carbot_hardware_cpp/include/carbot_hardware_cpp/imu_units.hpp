// C++ port of carbot_hardware/imu_units.py; keep both identical.
#ifndef CARBOT_HARDWARE_CPP__IMU_UNITS_HPP_
#define CARBOT_HARDWARE_CPP__IMU_UNITS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <utility>
#include <vector>

namespace carbot_hardware_cpp
{

constexpr double kStandardGravityMps2 = 9.80665;

// Estimate a scalar sensor bias from bounded stationary samples.
class StationaryBiasEstimator
{
public:
  StationaryBiasEstimator(int64_t min_samples, int64_t window_samples, double max_abs_value)
  : min_samples_(min_samples), window_(window_samples), max_abs_value_(max_abs_value)
  {
    if (min_samples <= 0) {
      throw std::invalid_argument("min_samples must be positive");
    }
    if (window_samples < min_samples) {
      throw std::invalid_argument("window_samples must be at least min_samples");
    }
    if (!(max_abs_value > 0.0) || !std::isfinite(max_abs_value)) {
      throw std::invalid_argument("max_abs_value must be finite and positive");
    }
  }

  std::size_t sample_count() const {return samples_.size();}
  bool ready() const {return static_cast<int64_t>(samples_.size()) >= min_samples_;}

  double bias() const
  {
    if (ordered_.empty()) {
      return 0.0;
    }
    const std::size_t middle = ordered_.size() / 2;
    if (ordered_.size() % 2) {
      return ordered_[middle];
    }
    return 0.5 * (ordered_[middle - 1] + ordered_[middle]);
  }

  double update(double value, bool stationary)
  {
    if (stationary && std::isfinite(value) && std::abs(value) <= max_abs_value_) {
      if (static_cast<int64_t>(samples_.size()) == window_) {
        const double oldest = samples_.front();
        samples_.pop_front();
        ordered_.erase(std::lower_bound(ordered_.begin(), ordered_.end(), oldest));
      }
      samples_.push_back(value);
      ordered_.insert(std::upper_bound(ordered_.begin(), ordered_.end(), value), value);
    }
    return value - bias();
  }

private:
  int64_t min_samples_;
  int64_t window_;
  double max_abs_value_;
  std::deque<double> samples_;
  std::vector<double> ordered_;
};

// Use an explicit diagonal when a sensor reports unknown covariance.
inline std::array<double, 9> covariance_with_fallback_diagonal(
  const std::array<double, 9> & covariance, const std::array<double, 3> & diagonal)
{
  for (double value : covariance) {
    if (value != 0.0) {
      return covariance;
    }
  }
  std::array<double, 9> result{};
  for (std::size_t index = 0; index < 3; ++index) {
    if (!(diagonal[index] > 0.0) || !std::isfinite(diagonal[index])) {
      throw std::invalid_argument("fallback covariance diagonal must be finite and positive");
    }
    result[index * 4] = diagonal[index];
  }
  return result;
}

// Scale a 3x3 covariance when each underlying value is scaled.
inline std::array<double, 9> scale_covariance(const std::array<double, 9> & covariance, double scale)
{
  const double scale_squared = std::pow(scale, 2);
  std::array<double, 9> result{};
  for (std::size_t index = 0; index < 9; ++index) {
    result[index] = covariance[index] * scale_squared;
  }
  return result;
}

// Add a signed correction to a ROS timestamp and normalize the result.
inline std::pair<int64_t, int64_t> shift_ros_stamp(int64_t sec, int64_t nanosec, double correction_s)
{
  if (!std::isfinite(correction_s)) {
    throw std::invalid_argument("timestamp correction must be finite");
  }
  if (nanosec < 0 || nanosec >= 1000000000) {
    throw std::invalid_argument("nanosec must be in [0, 1e9)");
  }
  // Python round() rounds half to even; std::nearbyint does too in the
  // default rounding mode.
  const int64_t corrected = sec * 1000000000 + nanosec +
    static_cast<int64_t>(std::nearbyint(correction_s * 1e9));
  if (corrected < 0) {
    throw std::invalid_argument("corrected ROS timestamp must be non-negative");
  }
  return {corrected / 1000000000, corrected % 1000000000};
}

}  // namespace carbot_hardware_cpp

#endif  // CARBOT_HARDWARE_CPP__IMU_UNITS_HPP_
