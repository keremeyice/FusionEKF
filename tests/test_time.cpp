#include "fusion/core/time.hpp"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

namespace fusion {
namespace {

// Compile-time checks (the conversions are constexpr).
static_assert(to_seconds(Duration{1'500'000'000}) == 1.5);
static_assert(std::is_same_v<Duration, std::chrono::nanoseconds>);

TEST(Time, ToSeconds) {
  EXPECT_DOUBLE_EQ(to_seconds(Duration{250ms}), 0.25);
  EXPECT_DOUBLE_EQ(to_seconds(Timestamp{2s}), 2.0);
}

TEST(Time, FromSecondsExact) {
  EXPECT_EQ(from_seconds(1.5), Duration{1500ms});
  EXPECT_EQ(from_seconds(0.0), Duration{0});
  EXPECT_EQ(from_seconds(-2.0), Duration{-2s});
}

TEST(Time, FromSecondsRoundsToNearestNanosecond) {
  // Truncation would give 999'999'999 ns here.
  EXPECT_EQ(from_seconds(0.9999999999), Duration{1s});
  EXPECT_EQ(from_seconds(1e-9 * 0.6), Duration{1ns});
}

TEST(Time, RoundTrip) {
  const Timestamp t = timestamp_from_seconds(1'700'000'000.123456789);
  // A double holds ~15-16 significant digits: at 1.7e9 s, that's ~0.2 us.
  EXPECT_NEAR(to_seconds(t), 1'700'000'000.123456789, 1e-6);
}

}  // namespace
}  // namespace fusion
