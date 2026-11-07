#include "fusion/sensors/imu.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace std::chrono_literals;

namespace fusion {
namespace {

ImuSample sample(Duration t, double ax, double wz) {
  return {.stamp = Timestamp{t},
          .accel = Eigen::Vector3d{ax, 0.0, 9.81},
          .gyro = Eigen::Vector3d{0.0, 0.0, wz}};
}

// ---- interpolate -------------------------------------------------------------
TEST(Imu, InterpolateMidpoint) {
  const auto a = sample(0ms, 1.0, 0.2);
  const auto b = sample(10ms, 3.0, 0.4);
  const ImuSample m = interpolate(a, b, Timestamp{5ms});
  EXPECT_EQ(m.stamp, Timestamp{5ms});
  EXPECT_NEAR(m.accel.x(), 2.0, 1e-12);
  EXPECT_NEAR(m.accel.z(), 9.81, 1e-12);
  EXPECT_NEAR(m.gyro.z(), 0.3, 1e-12);
}

TEST(Imu, InterpolateAtEndpointsReturnsEndpoints) {
  const auto a = sample(0ms, 1.0, 0.2);
  const auto b = sample(10ms, 3.0, 0.4);
  EXPECT_TRUE(interpolate(a, b, a.stamp).accel.isApprox(a.accel));
  EXPECT_TRUE(interpolate(a, b, b.stamp).gyro.isApprox(b.gyro));
}

TEST(Imu, InterpolateRejectsBadInput) {
  const auto a = sample(0ms, 1.0, 0.2);
  const auto b = sample(10ms, 3.0, 0.4);
  EXPECT_THROW((void)interpolate(a, b, Timestamp{11ms}), std::invalid_argument);  // t outside
  EXPECT_THROW((void)interpolate(b, a, Timestamp{5ms}), std::invalid_argument);   // a after b
  EXPECT_THROW((void)interpolate(a, a, a.stamp), std::invalid_argument);          // zero span
}

}  // namespace
}  // namespace fusion
