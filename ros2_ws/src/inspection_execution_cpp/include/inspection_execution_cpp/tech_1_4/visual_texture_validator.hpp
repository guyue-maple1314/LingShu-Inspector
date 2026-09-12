#pragma once

namespace inspection_execution {
namespace tech_1_4 {

struct VisualValidation {
  bool passed{false};
  double confidence{0.0};
};

class VisualTextureValidator {
 public:
  explicit VisualTextureValidator(double min_confidence = 0.6);
  VisualValidation Validate(double texture_confidence) const;

 private:
  double min_confidence_;
};

}  // namespace tech_1_4
}  // namespace inspection_execution
