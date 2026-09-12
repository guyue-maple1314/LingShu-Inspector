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

  /// 阵列无混叠上限频率 f_max = c / (2d)。
  /// 默认参数下约 5.7 kHz：监测频段高于该值时波束形成会发生空间混叠，
  /// 需先做低通或子带处理，不能直接采信方向估计。
  double MaxAliasFreeFrequencyHz() const;

 private:
  double mic_spacing_m_;
  double speed_of_sound_mps_;
};

}  // namespace tech_1_9
}  // namespace inspection_execution
