#include "fusion/filter/kalman_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

namespace fusion {
namespace {

// Helper: makes a 1x1 matrix or vector holding one number.
Eigen::MatrixXd mat1(double value) { return Eigen::MatrixXd::Constant(1, 1, value); }
Eigen::VectorXd vec1(double value) { return Eigen::VectorXd::Constant(1, value); }

// ---- constant velocity model ------------------------------------------------

TEST(ConstantVelocityModel, TransitionMatrix) {
  const Eigen::MatrixXd F = constant_velocity_F(0.5);
  ASSERT_EQ(F.rows(), 2);
  ASSERT_EQ(F.cols(), 2);
  EXPECT_DOUBLE_EQ(F(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(F(0, 1), 0.5);
  EXPECT_DOUBLE_EQ(F(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(F(1, 1), 1.0);
}

TEST(ConstantVelocityModel, ProcessNoise) {
  const Eigen::MatrixXd Q = constant_velocity_Q(2.0, 3.0);  // dt = 2, q = 9
  ASSERT_EQ(Q.rows(), 2);
  ASSERT_EQ(Q.cols(), 2);
  EXPECT_DOUBLE_EQ(Q(0, 0), 16.0 / 4.0 * 9.0);
  EXPECT_DOUBLE_EQ(Q(0, 1), 8.0 / 2.0 * 9.0);
  EXPECT_DOUBLE_EQ(Q(1, 0), 8.0 / 2.0 * 9.0);
  EXPECT_DOUBLE_EQ(Q(1, 1), 4.0 * 9.0);
}

// ---- construction -----------------------------------------------------------

TEST(KalmanFilter, StartsAtInitialState) {
  const Eigen::Vector2d x0(1.0, 2.0);
  const Eigen::Matrix2d P0 = Eigen::Matrix2d::Identity() * 4.0;
  const KalmanFilter kf(x0, P0);
  EXPECT_TRUE(kf.state().isApprox(x0));
  EXPECT_TRUE(kf.covariance().isApprox(P0));
}

// ---- predict ----------------------------------------------------------------

TEST(KalmanFilter, PredictMovesPositionByVelocityTimesDt) {
  KalmanFilter kf(Eigen::Vector2d(10.0, 2.0), Eigen::Matrix2d::Identity());
  kf.predict(constant_velocity_F(0.5), Eigen::Matrix2d::Zero());
  EXPECT_DOUBLE_EQ(kf.state()(0), 11.0);  // 10 + 2 * 0.5
  EXPECT_DOUBLE_EQ(kf.state()(1), 2.0);   // velocity unchanged
}

TEST(KalmanFilter, PredictGrowsUncertainty) {
  KalmanFilter kf(vec1(0.0), mat1(1.0));
  kf.predict(mat1(1.0), mat1(0.5));
  EXPECT_DOUBLE_EQ(kf.covariance()(0, 0), 1.5);  // F P Fᵀ + Q = 1 + 0.5
}

TEST(KalmanFilter, PredictPropagatesCovarianceThroughF) {
  KalmanFilter kf(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());
  kf.predict(constant_velocity_F(1.0), Eigen::Matrix2d::Zero());
  // F I Fᵀ = F Fᵀ = [[2, 1], [1, 1]]: position uncertainty grows because
  // velocity is uncertain, and position and velocity become correlated.
  EXPECT_DOUBLE_EQ(kf.covariance()(0, 0), 2.0);
  EXPECT_DOUBLE_EQ(kf.covariance()(0, 1), 1.0);
  EXPECT_DOUBLE_EQ(kf.covariance()(1, 0), 1.0);
  EXPECT_DOUBLE_EQ(kf.covariance()(1, 1), 1.0);
}

// ---- update -----------------------------------------------------------------

TEST(KalmanFilter, UpdateWithEqualUncertaintyLandsHalfway) {
  // Prior: 0 with variance 1. Measurement: 10 with variance 1.
  // Both are equally trusted, so the answer is exactly in the middle.
  KalmanFilter kf(vec1(0.0), mat1(1.0));
  kf.update(vec1(10.0), mat1(1.0), mat1(1.0));
  EXPECT_DOUBLE_EQ(kf.state()(0), 5.0);
  EXPECT_DOUBLE_EQ(kf.covariance()(0, 0), 0.5);  // more certain than either one
}

TEST(KalmanFilter, UpdateTrustsTheMoreCertainSource) {
  // Prior variance 4, measurement variance 1 -> gain K = 4 / (4 + 1) = 0.8
  KalmanFilter kf(vec1(0.0), mat1(4.0));
  kf.update(vec1(10.0), mat1(1.0), mat1(1.0));
  EXPECT_NEAR(kf.state()(0), 8.0, 1e-12);
  EXPECT_NEAR(kf.covariance()(0, 0), 0.8, 1e-12);  // (1 - K) P = 0.2 * 4
}

TEST(KalmanFilter, PositionMeasurementAlsoCorrectsVelocity) {
  // Start with position and velocity correlated (as after a predict).
  Eigen::Matrix2d P0;
  P0 << 2.0, 1.0,
        1.0, 1.0;
  KalmanFilter kf(Eigen::Vector2d::Zero(), P0);

  Eigen::MatrixXd H(1, 2);
  H << 1.0, 0.0;  // we only measure position
  kf.update(vec1(3.0), H, mat1(1.0));

  // The filter never saw velocity, but the correlation in P moves it anyway.
  EXPECT_NEAR(kf.state()(0), 2.0, 1e-12);  // K = [2/3, 1/3]
  EXPECT_NEAR(kf.state()(1), 1.0, 1e-12);
}

TEST(KalmanFilter, CovarianceStaysSymmetric) {
  KalmanFilter kf(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity() * 10.0);
  Eigen::MatrixXd H(1, 2);
  H << 1.0, 0.0;
  for (int i = 0; i < 100; ++i) {
    kf.predict(constant_velocity_F(0.1), constant_velocity_Q(0.1, 0.5));
    kf.update(vec1(1.0), H, mat1(4.0));
  }
  const Eigen::MatrixXd& P = kf.covariance();
  EXPECT_NEAR(P(0, 1), P(1, 0), 1e-9);
}

// ---- the whole filter on simulated data ---------------------------------------

TEST(KalmanFilter, TracksACarMovingAtConstantSpeed) {
  // Truth: a car starts at 0 m and drives at 15 m/s.
  // GNSS: measures position every 0.1 s with 3 m noise.
  const double dt = 0.1;
  const double gnss_std = 3.0;
  const double true_speed = 15.0;

  std::mt19937 rng(42);  // fixed seed, so the test gives the same result every run
  std::normal_distribution<double> gnss_noise(0.0, gnss_std);

  // We start with a bad guess: position 0, speed 0, and large uncertainty.
  KalmanFilter kf(Eigen::Vector2d::Zero(), Eigen::Vector2d(100.0, 100.0).asDiagonal());
  Eigen::MatrixXd H(1, 2);
  H << 1.0, 0.0;

  double true_position = 0.0;
  for (int step = 0; step < 300; ++step) {  // 30 seconds
    true_position += true_speed * dt;
    kf.predict(constant_velocity_F(dt), constant_velocity_Q(dt, 0.1));
    kf.update(vec1(true_position + gnss_noise(rng)), H, mat1(gnss_std * gnss_std));
  }

  // The filter estimates speed even though it is never measured.
  EXPECT_NEAR(kf.state()(1), true_speed, 0.5);
  // And its position is better than a single 3 m GNSS fix.
  EXPECT_NEAR(kf.state()(0), true_position, 1.5);
}

}  // namespace
}  // namespace fusion
