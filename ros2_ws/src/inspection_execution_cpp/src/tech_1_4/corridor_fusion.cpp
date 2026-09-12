#include "inspection_execution_cpp/tech_1_4/corridor_fusion.hpp"

namespace inspection_execution {
namespace tech_1_4 {

FusedCorridor CorridorFusion::Fuse(const CorridorSegmentation& seg,
                                   const VisualValidation& vis) const {
  FusedCorridor result;
  result.width = seg.width;
  result.center_x = seg.center_x;
  result.left_x = seg.left_x;
  result.right_x = seg.right_x;
  result.visual_validation_passed = vis.passed;
  result.confidence = vis.confidence;
  result.valid = seg.valid && vis.passed;
  return result;
}

}  // namespace tech_1_4
}  // namespace inspection_execution
