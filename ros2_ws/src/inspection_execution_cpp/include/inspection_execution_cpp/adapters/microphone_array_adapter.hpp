#pragma once

#include <cstdint>
#include <vector>

namespace inspection_execution {

struct AudioChunk {
  std::uint32_t sample_rate{0};
  std::uint32_t channels{0};
  std::vector<float> samples;
  std::uint64_t timestamp_ns{0};
};

class MicrophoneArrayAdapter {
 public:
  virtual ~MicrophoneArrayAdapter() = default;
  virtual bool Connect() = 0;
  virtual void Disconnect() = 0;
  virtual bool ReadChunk(AudioChunk* chunk) = 0;
};

}  // namespace inspection_execution
