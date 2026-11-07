#pragma once
// ---------------------------------------------------------------------------
// fusion/core/sample_buffer.hpp -- a bounded, time-ordered buffer of sensor
// samples. The filter uses it to find the two IMU samples around a GNSS or
// camera timestamp, so it can propagate exactly to the measurement time.
//
// Backed by std::deque: O(1) eviction at the front while keeping
// random-access iterators for O(log n) lookups.
// ---------------------------------------------------------------------------
#include <algorithm>
#include <concepts>
#include <cstddef>
#include <deque>
#include <iterator>
#include <optional>
#include <utility>

#include "fusion/core/time.hpp"

namespace fusion {

/// Any type with a public member `stamp` convertible to Timestamp.
template <typename T>
concept Timestamped = requires(const T& sample) {
  { sample.stamp } -> std::convertible_to<Timestamp>;
};

template <Timestamped T>
class SampleBuffer {
 public:
  using value_type = T;
  using const_iterator = typename std::deque<T>::const_iterator;

  explicit SampleBuffer(std::size_t capacity) : capacity_{capacity} {}

  /// Appends a sample. Returns false (buffer unchanged) if its stamp is not
  /// strictly newer than the newest stored sample: sensor streams must be
  /// monotonic, and duplicates / out-of-order stamps indicate a driver fault.
  /// When full, the oldest sample is evicted.
  [[nodiscard]] bool push([[maybe_unused]] T sample) {
    // TODO: implement
    return false;
  }

  /// Returns two consecutive samples (a, b) with
  ///     a.stamp <= t <= b.stamp   and   a.stamp < b.stamp,
  /// or std::nullopt if t is outside [front().stamp, back().stamp] or fewer
  /// than two samples are stored. O(log n).
  /// Returned by value: references would dangle once push() evicts.
  [[nodiscard]] std::optional<std::pair<T, T>> bracket([[maybe_unused]] Timestamp t) const {
    // TODO: implement
    return std::nullopt;
  }

  /// Drops samples older than `t`, except the newest one with stamp < t, so
  /// that bracket(t) still succeeds afterwards.
  void drop_before([[maybe_unused]] Timestamp t) {
    // TODO: implement
  }

  [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }
  [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }
  [[nodiscard]] bool empty() const noexcept { return samples_.empty(); }
  /// Precondition: !empty().
  [[nodiscard]] const T& front() const { return samples_.front(); }
  [[nodiscard]] const T& back() const { return samples_.back(); }

  [[nodiscard]] const_iterator begin() const noexcept { return samples_.cbegin(); }
  [[nodiscard]] const_iterator end() const noexcept { return samples_.cend(); }

 private:
  std::deque<T> samples_;
  std::size_t capacity_;
};

}  // namespace fusion
