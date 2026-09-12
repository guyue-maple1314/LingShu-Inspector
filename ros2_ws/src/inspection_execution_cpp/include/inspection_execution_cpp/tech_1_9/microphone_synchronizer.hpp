#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace inspection_execution {
namespace tech_1_9 {

constexpr std::size_t kNumMicrophones = 8;

struct MultiChannelAudio {
  std::uint32_t sample_rate{0};
  std::array<std::vector<float>, kNumMicrophones> channels;
  std::uint64_t timestamp_ns{0};
};

struct MicrophoneSyncResult {
  bool valid{false};
  std::string reason;
};

class MicrophoneSynchronizer {
 public:
  MicrophoneSyncResult Synchronize(
      const MultiChannelAudio& audio,
      std::uint64_t now_ns,
      std::uint64_t max_age_ns = 20000000ULL) const;
};

}  // namespace tech_1_9
}  // namespace inspection_execution
