#pragma once
// ---------------------------------------------------------------------------
// fusion/models/constant_velocity_2d.hpp -- constant-velocity motion model in
// the plane, and the GNSS position measurement model.
//
// State order (4 numbers):
//   index:   0   1   2    3
//   state: [ x,  y,  vx,  vy ]     positions [m], velocities [m/s]
// ---------------------------------------------------------------------------
#include <Eigen/Dense>

namespace fusion {

/// Transition matrix for a time step dt:
///   x  = x + vx·dt
///   y  = y + vy·dt
///   vx = vx
///   vy = vy
Eigen::MatrixXd constant_velocity_2d_F(double dt);

/// Process noise for a time step dt with random accelerations of standard
/// deviation accel_std [m/s²], independently in x and y.
Eigen::MatrixXd constant_velocity_2d_Q(double dt, double accel_std);

/// GNSS measures position (x, y) but not velocity: a 2x4 matrix.
Eigen::MatrixXd gnss_position_H();

}  // namespace fusion
