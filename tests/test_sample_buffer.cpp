#include "fusion/core/sample_buffer.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "fusion/sensors/imu.hpp"

using namespace std::chrono_literals;

namespace fusion {
namespace {

// A minimal type that satisfies the Timestamped concept.
struct Tick {
  Timestamp stamp;
  int id{0};
};

static_assert(Timestamped<Tick>);
static_assert(Timestamped<ImuSample>);
static_assert(!Timestamped<int>);

Timestamp at_ms(int ms) { return Timestamp{std::chrono::milliseconds{ms}}; }

SampleBuffer<Tick> make_buffer(std::vector<int> stamps_ms, std::size_t capacity = 10) {
  SampleBuffer<Tick> buf{capacity};
  for (int id = 0; const int ms : stamps_ms) {
    EXPECT_TRUE(buf.push({.stamp = at_ms(ms), .id = id++}));
  }
  return buf;
}

// ---- push ------------------------------------------------------------------
TEST(SampleBuffer, PushKeepsOrder) {
  const auto buf = make_buffer({0, 10, 20});
  ASSERT_EQ(buf.size(), 3u);
  EXPECT_EQ(buf.front().stamp, at_ms(0));
  EXPECT_EQ(buf.back().stamp, at_ms(20));
}

TEST(SampleBuffer, PushRejectsDuplicateAndOutOfOrder) {
  auto buf = make_buffer({0, 10});
  EXPECT_FALSE(buf.push({.stamp = at_ms(10)}));  // duplicate
  EXPECT_FALSE(buf.push({.stamp = at_ms(5)}));   // older
  EXPECT_EQ(buf.size(), 2u);
  EXPECT_TRUE(buf.push({.stamp = at_ms(11)}));
}

TEST(SampleBuffer, PushEvictsOldestWhenFull) {
  const auto buf = make_buffer({0, 10, 20, 30}, /*capacity=*/3);
  ASSERT_EQ(buf.size(), 3u);
  EXPECT_EQ(buf.front().stamp, at_ms(10));
  EXPECT_EQ(buf.back().stamp, at_ms(30));
}

TEST(SampleBuffer, ZeroCapacityStoresNothing) {
  SampleBuffer<Tick> buf{0};
  EXPECT_FALSE(buf.push({.stamp = at_ms(0)}));
  EXPECT_TRUE(buf.empty());
}

// ---- bracket ---------------------------------------------------------------
TEST(SampleBuffer, BracketInterior) {
  const auto buf = make_buffer({0, 10, 20, 30});
  const auto br = buf.bracket(at_ms(15));
  ASSERT_TRUE(br.has_value());
  EXPECT_EQ(br->first.stamp, at_ms(10));
  EXPECT_EQ(br->second.stamp, at_ms(20));
}

TEST(SampleBuffer, BracketOnExactSample) {
  const auto buf = make_buffer({0, 10, 20, 30});
  const auto br = buf.bracket(at_ms(20));
  ASSERT_TRUE(br.has_value());
  EXPECT_LE(br->first.stamp, at_ms(20));
  EXPECT_GE(br->second.stamp, at_ms(20));
  EXPECT_LT(br->first.stamp, br->second.stamp);
}

TEST(SampleBuffer, BracketAtBothEnds) {
  const auto buf = make_buffer({0, 10, 20});
  const auto first = buf.bracket(at_ms(0));
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->first.stamp, at_ms(0));
  EXPECT_EQ(first->second.stamp, at_ms(10));

  const auto last = buf.bracket(at_ms(20));
  ASSERT_TRUE(last.has_value());
  EXPECT_EQ(last->first.stamp, at_ms(10));
  EXPECT_EQ(last->second.stamp, at_ms(20));
}

TEST(SampleBuffer, BracketOutsideRangeIsEmpty) {
  const auto buf = make_buffer({10, 20});
  EXPECT_FALSE(buf.bracket(at_ms(9)).has_value());
  EXPECT_FALSE(buf.bracket(at_ms(21)).has_value());
}

TEST(SampleBuffer, BracketNeedsTwoSamples) {
  const auto buf = make_buffer({10});
  EXPECT_FALSE(buf.bracket(at_ms(10)).has_value());
}

// ---- drop_before -----------------------------------------------------------
TEST(SampleBuffer, DropBeforeKeepsOneOlderSample) {
  auto buf = make_buffer({0, 10, 20, 30, 40});
  buf.drop_before(at_ms(25));
  ASSERT_EQ(buf.size(), 3u);
  EXPECT_EQ(buf.front().stamp, at_ms(20));
  EXPECT_TRUE(buf.bracket(at_ms(25)).has_value());
}

TEST(SampleBuffer, DropBeforeOnExactStamp) {
  auto buf = make_buffer({0, 10, 20, 30});
  buf.drop_before(at_ms(20));
  EXPECT_EQ(buf.front().stamp, at_ms(10));
}

TEST(SampleBuffer, DropBeforeOlderThanAllDoesNothing) {
  auto buf = make_buffer({10, 20});
  buf.drop_before(at_ms(5));
  EXPECT_EQ(buf.size(), 2u);
}

TEST(SampleBuffer, WorksWithRangeFor) {
  const auto buf = make_buffer({0, 10, 20});
  int expected_id = 0;
  for (const Tick& tick : buf) EXPECT_EQ(tick.id, expected_id++);
}

}  // namespace
}  // namespace fusion
