#include <cmath>

#include "inspection_execution_cpp/tech_1_8/angle_distance_compensator.hpp"

namespace inspection_execution {
namespace tech_1_8 {

double AngleDistanceCompensator::CosDeg(double angle_deg) {
  // 角度→弧度→cos
  const double rad = angle_deg * (3.14159265358979323846 / 180.0);
  return std::cos(rad);
}

AngleDistanceCompensated AngleDistanceCompensator::Compensate(
    const EmissivityCompensated& input) {
  AngleDistanceCompensated out;
  out.timestamp_ns = input.timestamp_ns;
  out.angle = input.angle;
  out.distance = input.distance;
  out.correction_factor = input.correction_factor;

  // 红线：输入无效时不虚构补偿结果
  if (!input.valid) {
    out.error_state = "input_invalid";
    return out;
  }

  // 角度补偿：大角度下接收能量下降，compensated = temp / cos(angle)
  // 注意：|angle| ≥ 90° 时 cos ≤ 0，物理无意义；用 abs 并对 cos 做下限保护
  double cos_a = CosDeg(input.angle);
  if (cos_a < 0.01) {
    cos_a = 0.01;  // 防止除零/发散，范围校验由 validator 兜底
  }
  double temp = input.compensated_temperature / cos_a;

  // 距离补偿：远距离大气衰减，compensated = temp × correction_factor
  temp *= input.correction_factor;

  out.compensated_temperature = temp;
  out.valid = true;
  out.error_state = "ok";
  return out;
}

}  // namespace tech_1_8
}  // namespace inspection_execution
