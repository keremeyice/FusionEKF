// car_1d_demo: a car drives along a straight road at constant speed.
// A GNSS receiver measures its position with noise. The Kalman filter
// estimates position AND speed from those noisy positions.
//
// Run: ./build/debug/apps/car_1d_demo
#include <cmath>
#include <cstdio>
#include <random>

#include "fusion/filter/kalman_filter.hpp"

int main() {
  const double dt = 0.1;           // time step [s]
  const double true_speed = 15.0;  // [m/s], about 54 km/h
  const double gnss_std = 3.0;     // GNSS position noise [m]
  const double accel_std = 0.1;    // how much we expect the speed to change [m/s²]

  std::mt19937 rng(7);
  std::normal_distribution<double> gnss_noise(0.0, gnss_std);

  // Initial guess: at 0 m, standing still, but very unsure about both.
  fusion::KalmanFilter kf(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(100.0, 100.0).asDiagonal());

  Eigen::MatrixXd H(1, 2);
  H << 1.0, 0.0;  // GNSS measures position only
  const Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, gnss_std * gnss_std);

  std::printf("%6s %10s %10s %10s %8s %10s %8s\n", "t[s]", "true[m]", "gnss[m]", "est[m]",
              "err[m]", "speed[m/s]", "σpos[m]");

  double true_position = 0.0;
  for (int step = 1; step <= 200; ++step) {
    const double t = step * dt;
    true_position += true_speed * dt;
    const double gnss = true_position + gnss_noise(rng);

    kf.predict(fusion::constant_velocity_F(dt), fusion::constant_velocity_Q(dt, accel_std));
    kf.update(Eigen::VectorXd::Constant(1, gnss), H, R);

    if (step % 10 == 0) {  // print once per second
      const double est = kf.state()(0);
      std::printf("%6.1f %10.2f %10.2f %10.2f %8.2f %10.2f %8.2f\n", t, true_position, gnss, est,
                  est - true_position, kf.state()(1), std::sqrt(kf.covariance()(0, 0)));
    }
  }
  return 0;
}
