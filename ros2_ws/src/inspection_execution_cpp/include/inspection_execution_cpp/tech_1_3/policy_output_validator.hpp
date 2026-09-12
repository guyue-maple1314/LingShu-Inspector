#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/common/error_codes.hpp"
#include "inspection_execution_cpp/common/execution_result.hpp"

namespace inspection_execution {
namespace tech_1_3 {

// 策略输出：关节位置增量（默认 12 维四足，每条腿 3 关节），或速度/扭矩；
// 由 locomotion_command_adapter 决定最终映射到 JointCommand 的哪个字段。
inline constexpr std::uint32_t kDefaultActionDim = 12;

class PolicyOutputValidator {
 public:
  PolicyOutputValidator();

  void SetExpectedDimension(std::uint32_t dim);
  std::uint32_t ExpectedDimension() const { return expected_dim_; }

  // 每条维度允许的对称范围（±range_per_dim）；默认 ±1，适合 tanh 策略输出。
  void SetActionRangePerDim(float range_per_dim);
  float ActionRangePerDim() const { return range_per_dim_; }

  // 最大推理新鲜度（毫秒），超时视为 stale；默认 500ms。
  void SetMaxAgeMs(std::uint32_t ms);
  std::uint32_t MaxAgeMs() const { return max_age_ms_; }

  // 校验 action：维度 / 范围 / 时效。时间戳单位 ns。
  ExecutionResult Validate(const std::vector<float>& action,
                           std::uint64_t produced_timestamp_ns,
                           std::uint64_t now_timestamp_ns) const;

  // 将 action 裁剪到合法范围（不做推理，只做安全夹紧）；
  // 直接修改传入的 action；返回 ErrorCode::kOk 或 kExecutionFailed。
  ErrorCode Clamp(std::vector<float>* action) const;

 private:
  std::uint32_t expected_dim_;
  float range_per_dim_;
  std::uint32_t max_age_ms_;
};

}  // namespace tech_1_3
}  // namespace inspection_execution
