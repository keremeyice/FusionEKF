#include "fusion/sim/circle_simulator.hpp"

#include <cmath>
#include <random>

namespace fusion {

std::vector<SimStep> simulate_circle([[maybe_unused]] const CircleConfig& config) {
  const double omega = config.speed / config.radius;
  const int num_steps = static_cast<int>(std::round(config.duration / config.dt)) + 1;
  std::mt19937 rng(config.seed);
  std::normal_distribution<double> noise(0.0, config.gnss_std);
  std::vector<SimStep> steps;
  double t = 0.0;
  steps.reserve(static_cast<std::size_t>(num_steps));
  for (int i = 0; i < num_steps; i++)
  {
    SimStep temp_step;
    t = i * config.dt;
    temp_step.t = t;
    temp_step.true_x = config.radius * cos(omega * t);
    temp_step.true_y = config.radius * sin(omega * t);
    temp_step.true_vx = -config.speed * sin(omega * t);
    temp_step.true_vy = config.speed * cos(omega * t);
    if(i%config.gnss_every_n_steps == 0)
    {
      double n1 = noise(rng);
      double n2 = noise(rng);
      temp_step.gnss = GnssFix{temp_step.true_x + n1, temp_step.true_y + n2};
    }
    steps.push_back(temp_step);
  }
  return steps;
}

}  // namespace fusion
