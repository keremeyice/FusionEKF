#pragma once
// ---------------------------------------------------------------------------
// fusion/sensors/imu.hpp -- IMU sample type.
//
// Plain aggregate; construct with designated initializers:
//   ImuSample s{.stamp = t, .accel = a, .gyro = w};
// ---------------------------------------------------------------------------
#include <Eigen/Core>

#include "fusion/core/time.hpp"

namespace fusion {

/// One IMU reading, expressed in the IMU (body) frame.
struct ImuSample {
  Timestamp stamp{};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};  ///< specific force [m/s^2]
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};   ///< angular rate   [rad/s]
};

/// Linearly interpolates accel and gyro to time t. The result has stamp == t.
/// Preconditions: a.stamp < b.stamp and a.stamp <= t <= b.stamp.
/// Throws std::invalid_argument if they are violated.
[[nodiscard]] ImuSample interpolate(const ImuSample& a, const ImuSample& b, Timestamp t);

}  // namespace fusion
