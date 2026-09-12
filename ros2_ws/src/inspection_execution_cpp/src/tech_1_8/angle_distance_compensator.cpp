#include <cmath>

#include "inspection_execution_cpp/tech_1_8/angle_distance_compensator.hpp"

namespace inspection_execution {
namespace tech_1_8 {

double AngleDistanceCompensator::CosDeg(double angle_deg) {
  // 角度→弧度→cos
  const double rad = angle_deg * (3.14159265358979323846 / 180.0);
  return std::cos(rad);
}

void AngleDistanceCompensator::SetMaxAngleCorrectionFactor(double factor) {
  // 只接受 ≥1.0 的上限：小于 1.0 等于把角度修正变成衰减，非本项目意图
  if (factor >= 1.0) {
    max_angle_factor_ = factor;
  }
}

double AngleDistanceCompensator::MaxAngleCorrectionFactor() const {
  return max_angle_factor_;
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

  // 角度补偿：大角度下接收能量下降，采用有上限的 1/cos 近似
  // 注意：|angle| ≥ 90° 时 cos ≤ 0，物理无意义；对 cos 做下限保护，
  //       并对修正系数设上限，避免未标定阶段把读数放大到失真。
  double cos_a = CosDeg(input.angle);
  if (cos_a < 0.01) {
    cos_a = 0.01;  // 防止除零/发散，范围校验由 validator 兜底
  }
  double angle_factor = 1.0 / cos_a;
  if (angle_factor > max_angle_factor_) {
    angle_factor = max_angle_factor_;
  }
  double temp = input.compensated_temperature * angle_factor;

  // 距离补偿：远距离大气衰减，compensated = temp × correction_factor
  temp *= input.correction_factor;

  out.compensated_temperature = temp;
  out.valid = true;
  out.error_state = "ok";
  return out;
}

}  // namespace tech_1_8
}  // namespace inspection_execution
