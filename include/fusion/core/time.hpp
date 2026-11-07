#pragma once
// ---------------------------------------------------------------------------
// fusion/core/time.hpp -- the single representation of time in the project.
//
// Time is a std::chrono type, so unit mix-ups (s vs. ns) are compile errors.
// SensorClock is a distinct clock from the system clock, so sensor stamps
// cannot be mixed with wall-clock time.
// ---------------------------------------------------------------------------
#include <chrono>
#include <cstdint>

namespace fusion {

/// Clock tag for time as reported by the sensors (e.g. GPS time or a shared
/// hardware clock). It deliberately has no now(): in a fusion system, time comes
/// FROM THE DATA, never from the computer that happens to process it. That
/// makes log replay deterministic.
struct SensorClock {
  using rep = std::int64_t;
  using period = std::nano;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<SensorClock>;
  static constexpr bool is_steady = true;
};

/// Integer nanoseconds: exact, totally ordered, and no floating-point drift
/// over hours of logging. Convert to double seconds only at the math boundary.
using Duration = SensorClock::duration;
using Timestamp = SensorClock::time_point;

/// Duration -> floating-point seconds (for use in dt-based filter math).
[[nodiscard]] constexpr double to_seconds(Duration d) noexcept {
  return std::chrono::duration<double>(d).count();
}

/// Timestamp -> seconds since the sensor-clock epoch.
[[nodiscard]] constexpr double to_seconds(Timestamp t) noexcept {
  return to_seconds(t.time_since_epoch());
}

/// Floating-point seconds -> Duration, rounded to the nearest nanosecond
/// (not truncated: 0.9999999999 s becomes exactly 1 s).
[[nodiscard]] constexpr Duration from_seconds([[maybe_unused]] double seconds) noexcept {
  // TODO: implement
  return Duration{0};
}

/// Convenience: seconds since epoch -> Timestamp.
[[nodiscard]] constexpr Timestamp timestamp_from_seconds(double seconds) noexcept {
  return Timestamp{from_seconds(seconds)};
}

}  // namespace fusion
