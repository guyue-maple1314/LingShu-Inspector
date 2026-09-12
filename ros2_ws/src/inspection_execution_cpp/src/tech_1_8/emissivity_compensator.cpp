#include <cmath>

#include "inspection_execution_cpp/tech_1_8/emissivity_compensator.hpp"

namespace inspection_execution {
namespace tech_1_8 {

double EmissivityCompensator::PowQuarter(double x) {
  // x^0.25 = sqrt(sqrt(x))，避免直接 pow 的精度/告警问题
  return std::sqrt(std::sqrt(x));
}

EmissivityCompensated EmissivityCompensator::Compensate(
    const StabilizedThermal& input) {
  EmissivityCompensated out;
  out.timestamp_ns = input.timestamp_ns;
  out.angle = input.angle;
  out.distance = input.distance;
  out.correction_factor = input.correction_factor;

  // 红线：输入无效时不虚构补偿结果
  if (!input.valid) {
    out.error_state = "input_invalid";
    return out;
  }

  const double em = input.emissivity;
  // 辐射率非物理值（≤0 或 >1）时标记 invalid
  if (em <= 0.0 || em > 1.0) {
    out.emissivity = em;
    out.error_state = "invalid_emissivity";
    return out;
  }

  // 辐射率补偿：compensated = raw / ε^0.25
  out.emissivity = em;
  out.compensated_temperature = input.stabilized_temperature / PowQuarter(em);
  out.valid = true;
  out.error_state = "ok";
  return out;
}

}  // namespace tech_1_8
}  // namespace inspection_execution
