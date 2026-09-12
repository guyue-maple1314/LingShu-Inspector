#pragma once

#include <array>
#include <cstdint>

namespace inspection_execution {

struct FootForceSample {
  std::array<double, 4> normal_forces{};
  std::array<double, 4> contact_flags{};
  std::uint64_t timestamp_ns{0};
};

class FootForceAdapter {
 public:
  virtual ~FootForceAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool ReadSample(FootForceSample* sample) = 0;
};

}  // namespace inspection_execution
