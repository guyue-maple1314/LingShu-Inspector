#pragma once

#include <cstdint>
#include <string>

#include "inspection_execution_cpp/tech_1_8/emissivity_compensator.hpp"

namespace inspection_execution {
namespace tech_1_8 {

/// 角度 / 距离补偿结果
struct AngleDistanceCompensated {
  double compensated_temperature{0.0};  // 角度+距离补偿后温度（℃）
  double angle{0.0};                    // 测量角度
  double distance{0.0};                 // 测量距离
  double correction_factor{1.0};        // 修正系数
  std::uint64_t timestamp_ns{0};
  bool valid{false};
  std::string error_state;  // "ok" / "input_invalid"
};

/// 角度 / 距离补偿器（技术 1.8）
///
/// 执行 ±60°、1–5m 范围内的角度与距离补偿：
///   - 角度补偿：大角度下红外接收能量下降，需补偿
///     未标定阶段采用**有上限**的朗伯体近似：
///       angle_factor = min(1 / cos(angle), max_angle_correction_factor)
///     默认上限 1.10（角度修正最多 +10%）。直接取 1/cos 在 ±60° 时会把读数
///     放大一倍，量纲上不成立且会掩盖真实误差，本项目实机标定完成前不上放；
///     标定后可用 SetMaxAngleCorrectionFactor 调整上限。
///   - 距离补偿：远距离大气衰减，compensated = temp × correction_factor
///   - 修正系数 ∈ [0.95, 1.05] 由标定给出
///
/// 注意：本组件只做"范围内"补偿，范围校验由 ThermalRangeValidator 负责
///       （红线：超范围不按范围内精度发布，由 validator 标记）
///
/// 红线：
///   - 输入无效时不虚构补偿结果
///   - 补偿在 C++ 执行（Python 不重复实现）
class AngleDistanceCompensator {
 public:
  /// 角度修正系数上限默认值（实机标定前不得放大）
  static constexpr double kDefaultMaxAngleCorrectionFactor = 1.10;

  /// 角度 + 距离补偿
  AngleDistanceCompensated Compensate(const EmissivityCompensated& input);

  /// 设置角度修正系数上限（必须 ≥ 1.0；标定完成后按实测调整）
  void SetMaxAngleCorrectionFactor(double factor);

  /// 当前角度修正系数上限
  double MaxAngleCorrectionFactor() const;

 private:
  static double CosDeg(double angle_deg);  // 角度→cos

  double max_angle_factor_{kDefaultMaxAngleCorrectionFactor};
};

}  // namespace tech_1_8
}  // namespace inspection_execution
