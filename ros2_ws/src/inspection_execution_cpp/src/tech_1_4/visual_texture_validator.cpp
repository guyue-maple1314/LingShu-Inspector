#include "inspection_execution_cpp/tech_1_4/visual_texture_validator.hpp"

namespace inspection_execution {
namespace tech_1_4 {

VisualTextureValidator::VisualTextureValidator(double min_confidence)
    : min_confidence_(min_confidence) {}

VisualValidation VisualTextureValidator::Validate(double texture_confidence) const {
  VisualValidation result;
  result.confidence = texture_confidence;
  result.passed = texture_confidence >= min_confidence_;
  return result;
}

}  // namespace tech_1_4
}  // namespace inspection_execution
