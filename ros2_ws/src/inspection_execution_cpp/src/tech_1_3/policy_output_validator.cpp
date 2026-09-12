#include "inspection_execution_cpp/tech_1_3/policy_output_validator.hpp"

#include <algorithm>
#include <cstdint>

namespace inspection_execution {
namespace tech_1_3 {

PolicyOutputValidator::PolicyOutputValidator()
    : expected_dim_(kDefaultActionDim), range_per_dim_(1.0f), max_age_ms_(500) {}

void PolicyOutputValidator::SetExpectedDimension(std::uint32_t dim) {
  expected_dim_ = dim == 0 ? kDefaultActionDim : dim;
}

void PolicyOutputValidator::SetActionRangePerDim(float range_per_dim) {
  range_per_dim_ = range_per_dim > 0.0f ? range_per_dim : 1.0f;
}

void PolicyOutputValidator::SetMaxAgeMs(std::uint32_t ms) { max_age_ms_ = ms; }

ExecutionResult PolicyOutputValidator::Validate(const std::vector<float>& action,
                                                 std::uint64_t produced_timestamp_ns,
                                                 std::uint64_t now_timestamp_ns) const {
  if (action.size() != expected_dim_) {
    return ExecutionResult::Failure(
        static_cast<std::int32_t>(ErrorCode::kExecutionFailed),
        "policy action dim mismatch: got " + std::to_string(action.size()) + " expected " +
            std::to_string(expected_dim_));
  }
  for (float v : action) {
    if (v != v /*NaN*/) {
      return ExecutionResult::Failure(
          static_cast<std::int32_t>(ErrorCode::kExecutionFailed),
          "policy action contains NaN");
    }
    if (v < -range_per_dim_ || v > range_per_dim_) {
      // 越界不直接 Fail，提醒调用方应先走 Clamp。
      return ExecutionResult::Failure(
          static_cast<std::int32_t>(ErrorCode::kExecutionFailed),
          "policy action value out of range; call Clamp before ConvertAndSend");
    }
  }
  if (now_timestamp_ns >= produced_timestamp_ns) {
    const std::uint64_t age_ns = now_timestamp_ns - produced_timestamp_ns;
    const std::uint64_t limit_ns = static_cast<std::uint64_t>(max_age_ms_) * 1000000ULL;
    if (age_ns > limit_ns) {
      return ExecutionResult::Failure(
          static_cast<std::int32_t>(ErrorCode::kStaleSensorData),
          "policy action is stale");
    }
  }
  return ExecutionResult::Ok("policy action valid");
}

ErrorCode PolicyOutputValidator::Clamp(std::vector<float>* action) const {
  if (!action) return ErrorCode::kExecutionFailed;
  if (action->size() != expected_dim_) return ErrorCode::kExecutionFailed;
  for (auto& v : *action) {
    if (v != v) v = 0.0f;  // NaN → 0
    v = std::max(-range_per_dim_, std::min(range_per_dim_, v));
  }
  return ErrorCode::kOk;
}

}  // namespace tech_1_3
}  // namespace inspection_execution
