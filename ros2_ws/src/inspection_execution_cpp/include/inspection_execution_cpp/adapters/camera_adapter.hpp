#pragma once

#include <cstdint>
#include <vector>

namespace inspection_execution {

struct ImageFrame {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t channels{0};
  std::vector<std::uint8_t> data;
  std::uint64_t timestamp_ns{0};
};

class CameraAdapter {
 public:
  virtual ~CameraAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool ReadFrame(ImageFrame* frame) = 0;
};

}  // namespace inspection_execution
