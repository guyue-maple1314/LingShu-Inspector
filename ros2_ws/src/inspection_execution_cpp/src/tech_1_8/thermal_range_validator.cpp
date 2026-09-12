#include "inspection_execution_cpp/tech_1_8/thermal_range_validator.hpp"

namespace inspection_execution {
namespace tech_1_8 {

RangeValidationResult ThermalRangeValidator::Validate(
    const AngleDistanceCompensated& input, double emissivity) {
  RangeValidationResult out;
  out.timestamp_ns = input.timestamp_ns;
  out.angle = input.angle;
  out.distance = input.distance;
  out.emissivity = emissivity;
  out.correction_factor = input.correction_factor;
  out.final_temperature = input.compensated_temperature;

  // 红线：输入无效时不虚构结果
  if (!input.valid) {
    out.error_state = "input_invalid";
    out.in_range = false;
    return out;
  }

  // 逐项范围检查（红线：超范围不按范围内精度发布）
  if (!EmissivityValid(emissivity)) {
    out.error_state = "emissivity_out_of_range";
    out.in_range = false;
    return out;
  }
  if (!AngleInRange(input.angle)) {
    out.error_state = "angle_out_of_range";
    out.in_range = false;
    return out;
  }
  if (!DistanceInRange(input.distance)) {
    out.error_state = "distance_out_of_range";
    out.in_range = false;
    return out;
  }
  if (!CorrectionFactorInRange(input.correction_factor)) {
    out.error_state = "coefficient_out_of_range";
    out.in_range = false;
    return out;
  }

  // 全部在范围内
  out.in_range = true;
  out.error_state = "ok";
  return out;
}

bool ThermalRangeValidator::AngleInRange(double angle_deg) const {
  return angle_deg >= kAngleMinDeg && angle_deg <= kAngleMaxDeg;
}

bool ThermalRangeValidator::DistanceInRange(double distance_m) const {
  return distance_m >= kDistanceMinM && distance_m <= kDistanceMaxM;
}

bool ThermalRangeValidator::CorrectionFactorInRange(double cf) const {
  return cf >= kCorrectionFactorMin && cf <= kCorrectionFactorMax;
}

bool ThermalRangeValidator::EmissivityValid(double em) const {
  return em >= kEmissivityMin && em <= kEmissivityMax;
}

}  // namespace tech_1_8
}  // namespace inspection_execution
