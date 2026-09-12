#pragma once

#include <vector>

#include "inspection_execution_cpp/tech_1_9/microphone_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_9 {

struct BeamResult {
  bool valid{false};
  std::vector<float> mono;
};

class DirectionalBeamformer {
 public:
  explicit DirectionalBeamformer(double mic_spacing_m = 0.03,
                                 double speed_of_sound_mps = 343.0);
  BeamResult Beamform(const MultiChannelAudio& audio, double azimuth_rad) const;

 private:
  double mic_spacing_m_;
  double speed_of_sound_mps_;
};

}  // namespace tech_1_9
}  // namespace inspection_execution
