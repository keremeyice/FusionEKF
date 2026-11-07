#include "fusion/sensors/imu.hpp"

#include <stdexcept>

namespace fusion {

ImuSample interpolate(const ImuSample& a, [[maybe_unused]] const ImuSample& b,
                      [[maybe_unused]] Timestamp t) {
  // TODO: implement
  return a;
}

}  // namespace fusion
