// Mirrors carbot_hardware/test/test_imu_units.py for the C++ port.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "carbot_hardware_cpp/imu_units.hpp"

using carbot_hardware_cpp::StationaryBiasEstimator;
using carbot_hardware_cpp::covariance_with_fallback_diagonal;
using carbot_hardware_cpp::scale_covariance;
using carbot_hardware_cpp::shift_ros_stamp;

TEST(ImuUnits, CovarianceIsScaledByGravitySquared)
{
  const auto converted = scale_covariance({1.0, 0, 0, 0, 2.0, 0, 0, 0, 3.0}, 2.0);
  EXPECT_DOUBLE_EQ(converted[0], 4.0);
  EXPECT_DOUBLE_EQ(converted[4], 8.0);
  EXPECT_DOUBLE_EQ(converted[8], 12.0);
}

TEST(ImuUnits, TimestampCorrectionNormalizesAcrossSecondBoundary)
{
  const auto stamp = shift_ros_stamp(100, 5000000, -0.009782937);
  EXPECT_EQ(stamp.first, 99);
  EXPECT_EQ(stamp.second, 995217063);
}

TEST(ImuUnits, TimestampCorrectionRejectsInvalidInputs)
{
  EXPECT_THROW(shift_ros_stamp(0, 0, -0.1), std::invalid_argument);
  EXPECT_THROW(shift_ros_stamp(1, 1000000000, 0.0), std::invalid_argument);
  EXPECT_THROW(
    shift_ros_stamp(1, 0, std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(ImuUnits, UnknownCovarianceUsesTheFallbackDiagonal)
{
  const auto result = covariance_with_fallback_diagonal({}, {1e-4, 2e-4, 4e-6});
  EXPECT_DOUBLE_EQ(result[0], 1e-4);
  EXPECT_DOUBLE_EQ(result[4], 2e-4);
  EXPECT_DOUBLE_EQ(result[8], 4e-6);
  const std::array<double, 9> reported{0, 0, 0, 0, 0, 0, 0, 0, 7.0};
  EXPECT_EQ(covariance_with_fallback_diagonal(reported, {1e-4, 2e-4, 4e-6}), reported);
  EXPECT_THROW(covariance_with_fallback_diagonal({}, {1e-4, 0.0, 4e-6}), std::invalid_argument);
}

TEST(ImuUnits, BiasIsTheMedianOfStationarySamplesInTheWindow)
{
  StationaryBiasEstimator estimator(3, 4, 0.03);
  EXPECT_DOUBLE_EQ(estimator.update(0.010, true), 0.0);
  EXPECT_FALSE(estimator.ready());
  estimator.update(0.020, true);
  // Moving and out-of-range samples are not used.
  estimator.update(0.5, false);
  estimator.update(0.1, true);
  estimator.update(std::nan(""), true);
  EXPECT_DOUBLE_EQ(estimator.update(0.012, true), 0.0);
  EXPECT_TRUE(estimator.ready());
  EXPECT_DOUBLE_EQ(estimator.bias(), 0.012);
  estimator.update(0.011, true);
  EXPECT_DOUBLE_EQ(estimator.bias(), 0.0115);
  // The window drops the oldest sample (0.010).
  estimator.update(0.030, true);
  EXPECT_EQ(estimator.sample_count(), 4u);
  EXPECT_DOUBLE_EQ(estimator.bias(), 0.016);
}

TEST(ImuUnits, BiasEstimatorRejectsInvalidSettings)
{
  EXPECT_THROW(StationaryBiasEstimator(0, 4, 0.03), std::invalid_argument);
  EXPECT_THROW(StationaryBiasEstimator(5, 4, 0.03), std::invalid_argument);
  EXPECT_THROW(StationaryBiasEstimator(3, 4, 0.0), std::invalid_argument);
}
