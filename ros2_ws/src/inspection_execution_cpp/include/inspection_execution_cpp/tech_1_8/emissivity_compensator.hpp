#pragma once

#include <cstdint>
#include <string>

#include "inspection_execution_cpp/tech_1_8/thermal_image_stabilizer.hpp"

namespace inspection_execution {
namespace tech_1_8 {

/// 辐射率补偿结果
struct EmissivityCompensated {
  double compensated_temperature{0.0};  // 辐射率补偿后温度（℃）
  double emissivity{1.0};              // 使用的辐射率
  double angle{0.0};                   // 透传
  double distance{0.0};                // 透传
  double correction_factor{1.0};       // 透传
  std::uint64_t timestamp_ns{0};
  bool valid{false};
  std::string error_state;  // "ok" / "invalid_emissivity" / "input_invalid"
};

/// 辐射率补偿器（技术 1.8）
///
/// 根据目标辐射率补偿原始测温值：
///   - 简化模型：compensated = raw / pow(emissivity, 0.25)
///     （红外测温的辐射率补偿通常为 ε^0.25 量级，STEEL-BALL 黑体近似）
///   - 辐射率 ∈ (0, 1]，超出物理意义时标记 invalid
///
/// 红线：
///   - 输入无效时不虚构补偿结果（valid=false）
///   - 辐射率非物理值（≤0 或 >1）时标记 "invalid_emissivity"
///   - 补偿在 C++ 执行（Python 不重复实现）
class EmissivityCompensator {
 public:
  /// 辐射率补偿
  EmissivityCompensated Compensate(const StabilizedThermal& input);

 private:
  static double PowQuarter(double x);  // x^0.25
};

}  // namespace tech_1_8
}  // namespace inspection_execution
