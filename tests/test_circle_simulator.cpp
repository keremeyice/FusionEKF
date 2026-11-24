#include "fusion/sim/circle_simulator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace fusion {
namespace {

TEST(CircleSimulator, ProducesOneStepPerTimeIncludingBothEnds) {
  const std::vector<SimStep> steps = simulate_circle(CircleConfig{.dt = 0.1, .duration = 60.0});
  ASSERT_EQ(steps.size(), 601u);  // t = 0.0, 0.1, ..., 60.0
  EXPECT_DOUBLE_EQ(steps.front().t, 0.0);
  EXPECT_NEAR(steps.back().t, 60.0, 1e-9);
  EXPECT_NEAR(steps[1].t - steps[0].t, 0.1, 1e-12);
}

TEST(CircleSimulator, StartsAtRadiusZeroMovingInPlusY) {
  const auto steps = simulate_circle(CircleConfig{.radius = 50.0, .speed = 10.0});
  EXPECT_DOUBLE_EQ(steps[0].true_x, 50.0);
  EXPECT_DOUBLE_EQ(steps[0].true_y, 0.0);
  EXPECT_NEAR(steps[0].true_vx, 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(steps[0].true_vy, 10.0);
}

TEST(CircleSimulator, TruthStaysOnTheCircleAtConstantSpeed) {
  const CircleConfig config{.radius = 30.0, .speed = 12.0};
  for (const SimStep& step : simulate_circle(config)) {
    EXPECT_NEAR(std::hypot(step.true_x, step.true_y), 30.0, 1e-9) << "at t = " << step.t;
    EXPECT_NEAR(std::hypot(step.true_vx, step.true_vy), 12.0, 1e-9) << "at t = " << step.t;
  }
}

TEST(CircleSimulator, QuarterLapAfterQuarterPeriod) {
  // One full lap takes 2π·R / speed. After a quarter of that, the car is at (0, R).
  const double radius = 50.0;
  const double speed = 10.0;
  const double quarter_period = 2.0 * std::numbers::pi * radius / speed / 4.0;
  const auto steps = simulate_circle(
      CircleConfig{.radius = radius, .speed = speed, .dt = quarter_period, .duration = quarter_period});
  ASSERT_EQ(steps.size(), 2u);
  EXPECT_NEAR(steps[1].true_x, 0.0, 1e-9);
  EXPECT_NEAR(steps[1].true_y, radius, 1e-9);
  EXPECT_NEAR(steps[1].true_vx, -speed, 1e-9);
}

TEST(CircleSimulator, GnssOnlyEveryNthStep) {
  const auto steps = simulate_circle(CircleConfig{.gnss_every_n_steps = 10});
  int fixes = 0;
  for (std::size_t i = 0; i < steps.size(); ++i) {
    const bool should_have_fix = (i % 10 == 0);
    EXPECT_EQ(steps[i].gnss.has_value(), should_have_fix) << "at step " << i;
    if (steps[i].gnss) ++fixes;
  }
  EXPECT_EQ(fixes, 61);  // steps 0, 10, ..., 600
}

TEST(CircleSimulator, ZeroNoiseGnssEqualsTruth) {
  for (const SimStep& step : simulate_circle(CircleConfig{.gnss_std = 0.0})) {
    if (step.gnss) {
      EXPECT_DOUBLE_EQ(step.gnss->x, step.true_x);
      EXPECT_DOUBLE_EQ(step.gnss->y, step.true_y);
    }
  }
}

TEST(CircleSimulator, GnssNoiseHasTheRequestedSize) {
  // Long run with GNSS on every step, so we get many samples.
  const auto steps = simulate_circle(
      CircleConfig{.duration = 1000.0, .gnss_every_n_steps = 1, .gnss_std = 3.0});
  double sum = 0.0;
  double sum_sq = 0.0;
  int n = 0;
  for (const SimStep& step : steps) {
    const double error = step.gnss->x - step.true_x;
    sum += error;
    sum_sq += error * error;
    ++n;
  }
  const double mean = sum / n;
  const double std_dev = std::sqrt(sum_sq / n - mean * mean);
  EXPECT_NEAR(mean, 0.0, 0.1);
  EXPECT_NEAR(std_dev, 3.0, 0.1);
}

TEST(CircleSimulator, SameSeedGivesSameNoise) {
  const auto a = simulate_circle(CircleConfig{.seed = 1});
  const auto b = simulate_circle(CircleConfig{.seed = 1});
  const auto c = simulate_circle(CircleConfig{.seed = 2});
  EXPECT_DOUBLE_EQ(a[10].gnss->x, b[10].gnss->x);
  EXPECT_NE(a[10].gnss->x, c[10].gnss->x);
}

}  // namespace
}  // namespace fusion
