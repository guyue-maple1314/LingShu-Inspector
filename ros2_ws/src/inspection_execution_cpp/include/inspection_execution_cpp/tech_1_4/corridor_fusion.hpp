#pragma once

#include "inspection_execution_cpp/tech_1_4/dynamic_pointcloud_segmenter.hpp"
#include "inspection_execution_cpp/tech_1_4/visual_texture_validator.hpp"

namespace inspection_execution {
namespace tech_1_4 {

struct FusedCorridor {
  bool valid{false};
  double width{0.0};
  double center_x{0.0};
  double left_x{0.0};
  double right_x{0.0};
  bool visual_validation_passed{false};
  double confidence{0.0};
};

class CorridorFusion {
 public:
  FusedCorridor Fuse(const CorridorSegmentation& seg, const VisualValidation& vis) const;
};

}  // namespace tech_1_4
}  // namespace inspection_execution
