#pragma once

#include <cstdint>

namespace inspection_execution {

struct ImuSample {
  double ax{0.0};
  double ay{0.0};
  double az{0.0};
  double gx{0.0};
  double gy{0.0};
  double gz{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
  std::uint64_t timestamp_ns{0};
};

class ImuAdapter {
 public:
  virtual ~ImuAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool ReadSample(ImuSample* sample) = 0;
};

}  // namespace inspection_execution
