#include "inspection_execution_cpp/tech_1_9/spatial_filter.hpp"

namespace inspection_execution {
namespace tech_1_9 {

bool SpatialFilter::Filter(MultiChannelAudio* audio) const {
  if (audio == nullptr || audio->channels[0].empty()) {
    return false;
  }
  const std::size_t frames = audio->channels[0].size();
  for (std::size_t n = 0; n < frames; ++n) {
    float mean = 0.0f;
    for (std::size_t i = 0; i < kNumMicrophones; ++i) {
      mean += audio->channels[i][n];
    }
    mean /= static_cast<float>(kNumMicrophones);
    for (std::size_t i = 0; i < kNumMicrophones; ++i) {
      audio->channels[i][n] -= mean;
    }
  }
  return true;
}

}  // namespace tech_1_9
}  // namespace inspection_execution
