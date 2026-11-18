#pragma once
// ---------------------------------------------------------------------------
// fusion/filter/kalman_filter.hpp -- a linear Kalman filter.
//
// The filter keeps two things:
//   x : the state estimate          (e.g. [position, velocity])
//   P : the covariance of x         (how uncertain we are about x)
//
// and does two things:
//   predict(): move the state forward in time with a motion model
//   update():  correct the state with a measurement
// ---------------------------------------------------------------------------
#include <Eigen/Dense>

namespace fusion {

class KalmanFilter {
 public:
  /// Starts the filter at state x0 with covariance P0.
  KalmanFilter(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

  /// Prediction step with motion model F and process noise Q:
  ///   x = F x
  ///   P = F P Fᵀ + Q
  void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

  /// Update step with measurement z, measurement model H and noise R:
  ///   y = z − H x              (innovation: measured minus predicted)
  ///   S = H P Hᵀ + R           (innovation covariance)
  ///   K = P Hᵀ S⁻¹             (Kalman gain)
  ///   x = x + K y
  ///   P = (I − K H) P
  void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

  /// Read-only access to the current estimate.
  const Eigen::VectorXd& state() const { return x_; }
  const Eigen::MatrixXd& covariance() const { return P_; }

 private:
  Eigen::VectorXd x_;  // state estimate
  Eigen::MatrixXd P_;  // state covariance
};

// ---- Motion model: constant velocity in 1D --------------------------------
// State: [position, velocity]

/// Transition matrix for a time step dt:
///   | 1  dt |
///   | 0  1  |
Eigen::MatrixXd constant_velocity_F(double dt);

/// Process noise for a time step dt, assuming random accelerations with
/// standard deviation accel_std [m/s²]:
///   q = accel_std²
///   | dt⁴/4  dt³/2 | · q
///   | dt³/2  dt²   |
Eigen::MatrixXd constant_velocity_Q(double dt, double accel_std);

}  // namespace fusion
