#include "inspection_execution_cpp/tech_1_9/snr_estimator.hpp"

#include <cmath>
#include <limits>

namespace inspection_execution {
namespace tech_1_9 {

double SnrEstimator::EstimateDb(const std::vector<float>& signal,
                                const std::vector<float>& noise) const {
  if (signal.empty() || noise.empty()) {
    return 0.0;
  }
  double signal_power = 0.0;
  for (float v : signal) {
    signal_power += static_cast<double>(v) * v;
  }
  signal_power /= static_cast<double>(signal.size());

  double noise_power = 0.0;
  for (float v : noise) {
    noise_power += static_cast<double>(v) * v;
  }
  noise_power /= static_cast<double>(noise.size());

  if (noise_power <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return 10.0 * std::log10(signal_power / noise_power);
}

}  // namespace tech_1_9
}  // namespace inspection_execution
