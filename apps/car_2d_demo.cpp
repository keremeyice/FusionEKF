// car_2d_demo: a car drives in a circle; GNSS measures its position at 5 Hz.
// A 2D constant-velocity Kalman filter runs at 10 Hz: it predicts every step
// and updates whenever a GNSS fix arrives.
//
// Run:   ./build/debug/apps/car_2d_demo [output.csv] [accel_std]
// Plot:  python3 tools/plot_car_2d.py car_2d.csv
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "fusion/filter/kalman_filter.hpp"
#include "fusion/io/results_csv.hpp"
#include "fusion/models/constant_velocity_2d.hpp"
#include "fusion/sim/circle_simulator.hpp"

int main(int argc, char** argv) {
  const std::string output_path = (argc > 1) ? argv[1] : "car_2d.csv";
  const double accel_std = (argc > 2) ? std::stod(argv[2]) : 3.0;  // filter tuning [m/s²]

  const fusion::CircleConfig config;  // all defaults: R = 100 m, 10 m/s, GNSS 5 Hz, 3 m noise
  const std::vector<fusion::SimStep> steps = fusion::simulate_circle(config);

  // Start from the first GNSS fix, with zero velocity and large uncertainty.
  const fusion::GnssFix& first_fix = *steps.front().gnss;
  const Eigen::Vector4d x0(first_fix.x, first_fix.y, 0.0, 0.0);
  const Eigen::Vector4d p0_diagonal(9.0, 9.0, 100.0, 100.0);
  fusion::KalmanFilter kf(x0, p0_diagonal.asDiagonal());

  const Eigen::MatrixXd F = fusion::constant_velocity_2d_F(config.dt);
  const Eigen::MatrixXd Q = fusion::constant_velocity_2d_Q(config.dt, accel_std);
  const Eigen::MatrixXd H = fusion::gnss_position_H();
  const Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2) * (config.gnss_std * config.gnss_std);

  std::vector<fusion::ResultRow> rows;
  rows.reserve(steps.size());

  double gnss_sq_error = 0.0;
  double filter_sq_error = 0.0;
  int gnss_count = 0;

  for (const fusion::SimStep& step : steps) {
    if (step.t > 0.0) {
      kf.predict(F, Q);
    }

    fusion::ResultRow row;
    if (step.gnss) {
      kf.update(Eigen::Vector2d(step.gnss->x, step.gnss->y), H, R);
      row.has_gnss = true;
      row.gnss_x = step.gnss->x;
      row.gnss_y = step.gnss->y;
      gnss_sq_error += std::pow(step.gnss->x - step.true_x, 2) + std::pow(step.gnss->y - step.true_y, 2);
      ++gnss_count;
    }

    const Eigen::VectorXd& x = kf.state();
    const Eigen::MatrixXd& P = kf.covariance();
    row.t = step.t;
    row.true_x = step.true_x;
    row.true_y = step.true_y;
    row.est_x = x(0);
    row.est_y = x(1);
    row.est_vx = x(2);
    row.est_vy = x(3);
    row.sigma_x = std::sqrt(P(0, 0));
    row.sigma_y = std::sqrt(P(1, 1));
    rows.push_back(row);

    filter_sq_error += std::pow(x(0) - step.true_x, 2) + std::pow(x(1) - step.true_y, 2);
  }

  if (!fusion::write_results_csv(output_path, rows)) {
    std::fprintf(stderr, "Could not write %s\n", output_path.c_str());
    return 1;
  }

  const double gnss_rmse = std::sqrt(gnss_sq_error / gnss_count);
  const double filter_rmse = std::sqrt(filter_sq_error / static_cast<double>(rows.size()));
  std::printf("accel_std        : %.2f m/s²\n", accel_std);
  std::printf("GNSS   RMSE      : %.2f m  (%d fixes)\n", gnss_rmse, gnss_count);
  std::printf("Filter RMSE      : %.2f m  (%zu steps)\n", filter_rmse, rows.size());
  std::printf("Wrote %s\n", output_path.c_str());
  return 0;
}
