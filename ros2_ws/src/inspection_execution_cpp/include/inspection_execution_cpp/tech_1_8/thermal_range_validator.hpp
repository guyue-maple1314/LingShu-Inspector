#pragma once

#include <cstdint>
#include <string>

#include "inspection_execution_cpp/tech_1_8/angle_distance_compensator.hpp"

namespace inspection_execution {
namespace tech_1_8 {

/// 范围校验结果
struct RangeValidationResult {
  double final_temperature{0.0};   // 最终补偿温度
  double angle{0.0};
  double distance{0.0};
  double emissivity{1.0};
  double correction_factor{1.0};
  bool in_range{false};            // 是否在 PPT 范围内
  std::string error_state;         // "ok" / "angle_out_of_range" /
                                   // "distance_out_of_range" /
                                   // "coefficient_out_of_range" /
                                   // "emissivity_out_of_range" / "input_invalid"
  std::uint64_t timestamp_ns{0};
};

/// 范围校验器（技术 1.8）
///
/// 检查测量条件是否落在 PPT 范围内：
///   - 角度：±60°
///   - 距离：1–5 m
///   - 修正系数：0.95–1.05
///   - 辐射率：(0, 1]
///
/// 红线：
///   - 超 PPT 范围时标记 "xxx_out_of_range"，in_range=false
///   - 不按范围内精度发布（即不宣称满足 ±0.2℃ 或 ±0.5℃ 指标）
///   - 输入无效时标记 "input_invalid"，in_range=false
///   - 范围内 → in_range=true，error_state="ok"
class ThermalRangeValidator {
 public:
  static constexpr double kAngleMinDeg = -60.0;
  static constexpr double kAngleMaxDeg = 60.0;
  static constexpr double kDistanceMinM = 1.0;
  static constexpr double kDistanceMaxM = 5.0;
  static constexpr double kCorrectionFactorMin = 0.95;
  static constexpr double kCorrectionFactorMax = 1.05;
  static constexpr double kEmissivityMin = 0.01;  // 物理下限（>0）
  static constexpr double kEmissivityMax = 1.0;

  /// 范围校验 + 最终结果封装
  RangeValidationResult Validate(const AngleDistanceCompensated& input,
                                double emissivity);

  /// 单项范围检查（供测试）
  bool AngleInRange(double angle_deg) const;
  bool DistanceInRange(double distance_m) const;
  bool CorrectionFactorInRange(double cf) const;
  bool EmissivityValid(double em) const;
};

}  // namespace tech_1_8
}  // namespace inspection_execution
