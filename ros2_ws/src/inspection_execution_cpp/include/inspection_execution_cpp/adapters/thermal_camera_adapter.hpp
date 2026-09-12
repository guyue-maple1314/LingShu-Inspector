#pragma once

#include <cstdint>
#include <vector>

namespace inspection_execution {

struct ThermalFrame {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<std::uint16_t> temperature_counts;
  double angle_deg{0.0};
  double distance_m{0.0};
  double emissivity{0.95};
  std::uint64_t timestamp_ns{0};
};

class ThermalCameraAdapter {
 public:
  virtual ~ThermalCameraAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool ReadFrame(ThermalFrame* frame) = 0;
};

}  // namespace inspection_execution
