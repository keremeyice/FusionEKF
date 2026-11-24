#include "fusion/models/constant_velocity_2d.hpp"

#include <gtest/gtest.h>

#include "fusion/filter/kalman_filter.hpp"

namespace fusion {
namespace {

TEST(ConstantVelocity2d, TransitionMatrix) {
  const Eigen::MatrixXd F = constant_velocity_2d_F(0.5);
  Eigen::MatrixXd expected(4, 4);
  expected << 1.0, 0.0, 0.5, 0.0,
              0.0, 1.0, 0.0, 0.5,
              0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 1.0;
  ASSERT_EQ(F.rows(), 4);
  ASSERT_EQ(F.cols(), 4);
  EXPECT_TRUE(F.isApprox(expected)) << "F =\n" << F;
}

TEST(ConstantVelocity2d, PredictMovesBothAxes) {
  KalmanFilter kf(Eigen::Vector4d(10.0, 20.0, 2.0, -4.0), Eigen::Matrix4d::Identity());
  kf.predict(constant_velocity_2d_F(0.5), Eigen::Matrix4d::Zero());
  EXPECT_DOUBLE_EQ(kf.state()(0), 11.0);  // x + vx·dt
  EXPECT_DOUBLE_EQ(kf.state()(1), 18.0);  // y + vy·dt
  EXPECT_DOUBLE_EQ(kf.state()(2), 2.0);
  EXPECT_DOUBLE_EQ(kf.state()(3), -4.0);
}

TEST(ConstantVelocity2d, ProcessNoiseMatches1dModelPerAxis) {
  const Eigen::MatrixXd Q2 = constant_velocity_2d_Q(0.2, 1.5);
  const Eigen::MatrixXd Q1 = constant_velocity_Q(0.2, 1.5);
  // x axis uses indices (0, 2), y axis uses (1, 3).
  EXPECT_DOUBLE_EQ(Q2(0, 0), Q1(0, 0));
  EXPECT_DOUBLE_EQ(Q2(0, 2), Q1(0, 1));
  EXPECT_DOUBLE_EQ(Q2(2, 2), Q1(1, 1));
  EXPECT_DOUBLE_EQ(Q2(1, 1), Q1(0, 0));
  EXPECT_DOUBLE_EQ(Q2(3, 1), Q1(1, 0));
  EXPECT_DOUBLE_EQ(Q2(0, 1), 0.0);  // x and y are independent
}

TEST(GnssPositionModel, MeasurementMatrix) {
  const Eigen::MatrixXd H = gnss_position_H();
  Eigen::MatrixXd expected(2, 4);
  expected << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
  ASSERT_EQ(H.rows(), 2);
  ASSERT_EQ(H.cols(), 4);
  EXPECT_TRUE(H.isApprox(expected)) << "H =\n" << H;
}

TEST(GnssPositionModel, PicksPositionOutOfTheState) {
  const Eigen::Vector4d x(3.0, 7.0, 100.0, 200.0);
  const Eigen::VectorXd predicted_measurement = gnss_position_H() * x;
  ASSERT_EQ(predicted_measurement.size(), 2);
  EXPECT_DOUBLE_EQ(predicted_measurement(0), 3.0);
  EXPECT_DOUBLE_EQ(predicted_measurement(1), 7.0);
}

}  // namespace
}  // namespace fusion
