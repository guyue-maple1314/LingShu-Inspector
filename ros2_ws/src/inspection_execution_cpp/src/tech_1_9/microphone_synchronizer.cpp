#include "inspection_execution_cpp/tech_1_9/microphone_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_9 {

MicrophoneSyncResult MicrophoneSynchronizer::Synchronize(
    const MultiChannelAudio& audio,
    std::uint64_t now_ns,
    std::uint64_t max_age_ns) const {
  MicrophoneSyncResult result;
  if (audio.sample_rate == 0) {
    result.reason = "sample_rate is zero";
    return result;
  }
  const std::size_t frames = audio.channels[0].size();
  for (std::size_t i = 0; i < kNumMicrophones; ++i) {
    if (audio.channels[i].empty()) {
      result.reason = "channel " + std::to_string(i) + " is empty";
      return result;
    }
    if (audio.channels[i].size() != frames) {
      result.reason = "channel frame count mismatch";
      return result;
    }
  }
  if (now_ns < audio.timestamp_ns || now_ns - audio.timestamp_ns > max_age_ns) {
    result.reason = "audio timestamp stale or invalid";
    return result;
  }
  result.valid = true;
  result.reason = "ok";
  return result;
}

}  // namespace tech_1_9
}  // namespace inspection_execution
