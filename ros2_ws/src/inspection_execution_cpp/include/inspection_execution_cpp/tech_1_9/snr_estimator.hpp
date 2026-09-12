#pragma once

#include <vector>

namespace inspection_execution {
namespace tech_1_9 {

class SnrEstimator {
 public:
  double EstimateDb(const std::vector<float>& signal, const std::vector<float>& noise) const;
};

}  // namespace tech_1_9
}  // namespace inspection_execution
