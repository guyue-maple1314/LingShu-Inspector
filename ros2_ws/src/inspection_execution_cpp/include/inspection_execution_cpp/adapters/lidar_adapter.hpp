#pragma once

#include <cstdint>
#include <vector>

namespace inspection_execution {

struct LidarScan {
  double range_min{0.0};
  double range_max{0.0};
  double angle_min{0.0};
  double angle_increment{0.0};
  std::vector<float> ranges;
  std::vector<float> intensities;
  std::uint64_t timestamp_ns{0};
};

class LidarAdapter {
 public:
  virtual ~LidarAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool ReadScan(LidarScan* scan) = 0;
};

}  // namespace inspection_execution
