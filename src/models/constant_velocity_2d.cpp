#include "fusion/models/constant_velocity_2d.hpp"

namespace fusion {

Eigen::MatrixXd constant_velocity_2d_F(double dt) {
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

Eigen::MatrixXd constant_velocity_2d_Q(double dt, double accel_std) {
  const double q = accel_std * accel_std;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  // Same 2x2 pattern as the 1D model, once for x (indices 0, 2)
  // and once for y (indices 1, 3).
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
  for (int axis = 0; axis < 2; ++axis) {
    const int pos = axis;      // 0 for x, 1 for y
    const int vel = axis + 2;  // 2 for vx, 3 for vy
    Q(pos, pos) = dt4 / 4.0 * q;
    Q(pos, vel) = dt3 / 2.0 * q;
    Q(vel, pos) = dt3 / 2.0 * q;
    Q(vel, vel) = dt2 * q;
  }
  return Q;
}

Eigen::MatrixXd gnss_position_H() {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 4);
  H(0, 0) = 1;
  H(1, 1) = 1;
  return H;
}

}  // namespace fusion
