#include "inspection_execution_cpp/tech_1_9/directional_beamformer.hpp"

#include <array>
#include <cmath>
#include <cstdint>

namespace inspection_execution {
namespace tech_1_9 {

DirectionalBeamformer::DirectionalBeamformer(double mic_spacing_m, double speed_of_sound_mps)
    : mic_spacing_m_(mic_spacing_m), speed_of_sound_mps_(speed_of_sound_mps) {}

double DirectionalBeamformer::MaxAliasFreeFrequencyHz() const {
  if (mic_spacing_m_ <= 0.0) return 0.0;
  return speed_of_sound_mps_ / (2.0 * mic_spacing_m_);
}

BeamResult DirectionalBeamformer::Beamform(const MultiChannelAudio& audio,
                                           double azimuth_rad) const {
  BeamResult result;
  if (audio.sample_rate == 0 || audio.channels[0].empty()) {
    return result;
  }
  const std::size_t frames = audio.channels[0].size();

  std::array<std::int64_t, kNumMicrophones> delays{};
  for (std::size_t i = 0; i < kNumMicrophones; ++i) {
    const double tau_sec =
        static_cast<double>(i) * mic_spacing_m_ * std::sin(azimuth_rad) / speed_of_sound_mps_;
    delays[i] = static_cast<std::int64_t>(std::llround(tau_sec * audio.sample_rate));
  }

  result.mono.assign(frames, 0.0f);
  for (std::size_t i = 0; i < kNumMicrophones; ++i) {
    for (std::size_t n = 0; n < frames; ++n) {
      const std::int64_t src = static_cast<std::int64_t>(n) - delays[i];
      if (src >= 0 && src < static_cast<std::int64_t>(frames)) {
        result.mono[n] += audio.channels[i][static_cast<std::size_t>(src)];
      }
    }
  }
  for (auto& v : result.mono) {
    v /= static_cast<float>(kNumMicrophones);
  }
  result.valid = true;
  return result;
}

}  // namespace tech_1_9
}  // namespace inspection_execution
