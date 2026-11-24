#pragma once
// ---------------------------------------------------------------------------
// fusion/sim/circle_simulator.hpp -- simulates a car driving in a circle at
// constant speed, and a GNSS receiver that measures its position with noise.
//
// Because we generate the data ourselves, we know the TRUE position at every
// moment, so we can measure exactly how good the filter is.
// ---------------------------------------------------------------------------
#include <optional>
#include <vector>

namespace fusion {

/// Settings for one simulation run. Every field has a sensible default, so
/// you only set what you want to change:
///   CircleConfig config{.speed = 20.0, .gnss_std = 5.0};
struct CircleConfig {
  double radius = 100.0;        ///< circle radius [m]
  double speed = 10.0;          ///< car speed [m/s]
  double dt = 0.1;              ///< simulation time step [s]
  double duration = 60.0;       ///< total time [s]
  int gnss_every_n_steps = 2;   ///< GNSS rate: every 2 steps of 0.1 s = 5 Hz
  double gnss_std = 3.0;        ///< GNSS noise, per axis [m]
  unsigned int seed = 42;       ///< random seed: same seed -> same noise
};

/// A GNSS position fix [m].
struct GnssFix {
  double x = 0.0;
  double y = 0.0;
};

/// Everything that is true at one time step, plus the GNSS fix if one
/// arrived at this step.
struct SimStep {
  double t = 0.0;  ///< time [s]
  double true_x = 0.0;
  double true_y = 0.0;
  double true_vx = 0.0;
  double true_vy = 0.0;
  std::optional<GnssFix> gnss;  ///< empty on steps without a GNSS fix
};

/// Runs the simulation. Returns one SimStep for each time
///   t = 0, dt, 2·dt, ..., duration      (both ends included)
///
/// The car starts at (radius, 0) and drives counter-clockwise:
///   ω  = speed / radius            (angular rate [rad/s])
///   x  =  radius · cos(ω t)        vx = −speed · sin(ω t)
///   y  =  radius · sin(ω t)        vy =  speed · cos(ω t)
///
/// A GNSS fix is added on steps 0, n, 2n, ... (n = gnss_every_n_steps):
/// the true position plus normal noise with standard deviation gnss_std.
std::vector<SimStep> simulate_circle(const CircleConfig& config);

}  // namespace fusion
