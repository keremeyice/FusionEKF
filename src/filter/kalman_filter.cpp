#include "fusion/filter/kalman_filter.hpp"

namespace fusion {

KalmanFilter::KalmanFilter(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
    : x_{x0}, P_{P0} {}

void KalmanFilter::predict(const Eigen::MatrixXd& F,
                           const Eigen::MatrixXd& Q) {
  P_ = F * P_ * F.transpose() + Q;
  x_ = F * x_;
}

void KalmanFilter::update(const Eigen::VectorXd& z,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R) {
  Eigen::VectorXd y(1);
  Eigen::MatrixXd S(1,1);
  Eigen::MatrixXd K(2,1);
  y = z - H * x_;
  S = H * P_ * H.transpose() + R;
  K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K* y;
  P_ = P_ - K * H * P_;
}

Eigen::MatrixXd constant_velocity_F(double dt) {
  Eigen::MatrixXd F(2, 2);
  F << 1.0, dt,
       0.0, 1.0;
  return F;
}

Eigen::MatrixXd constant_velocity_Q(double dt, double accel_std) {
  const double q = accel_std * accel_std;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  Eigen::MatrixXd Q(2, 2);
  Q << dt4 / 4.0, dt3 / 2.0,
       dt3 / 2.0, dt2;
  return Q * q;
}

}  // namespace fusion
