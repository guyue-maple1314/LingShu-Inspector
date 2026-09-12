#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <vector>

#include "inspection_execution_cpp/common/error_codes.hpp"

namespace inspection_execution {
namespace tech_1_1 {

namespace {

// 禁止性约束关键词：出现即拒绝（英文小写匹配 + 中文原文匹配）
const std::vector<std::string>& ForbiddenKeywords() {
  static const std::vector<std::string> kKeywords = {
      "disable_safety", "ignore_geofence", "disable_geofence", "no_limit",
      "bypass", "关闭安全", "忽略围栏", "无视限速", "取消限速",
  };
  return kKeywords;
}

std::string ToLower(const std::string& text) {
  std::string out = text;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

}  // namespace

bool GoalSafetyValidator::HasForbiddenConstraint(const std::string& constraints) {
  const std::string lowered = ToLower(constraints);
  for (const auto& keyword : ForbiddenKeywords()) {
    if (lowered.find(keyword) != std::string::npos) {
      return true;
    }
  }
  return false;
}

double GoalSafetyValidator::ParseMaxSpeedMps(const std::string& constraints) {
  const std::string lowered = ToLower(constraints);
  const std::string key = "max_speed";
  const std::size_t pos = lowered.find(key);
  if (pos == std::string::npos) {
    return 0.0;
  }
  std::size_t cursor = pos + key.size();
  while (cursor < lowered.size() &&
         (lowered[cursor] == ' ' || lowered[cursor] == '=' ||
          lowered[cursor] == ':' || lowered[cursor] == '"')) {
    ++cursor;
  }
  const char* begin = constraints.c_str() + cursor;
  char* end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin || value <= 0.0) {
    return 0.0;
  }
  return value;
}

GoalSafetyResult GoalSafetyValidator::Validate(const GoalSafetyInput& input) const {
  GoalSafetyResult result;

  if (input.goal_id.empty()) {
    result.error_code = static_cast<std::int32_t>(ErrorCode::kInvalidGoal);
    result.reason = "goal_id is empty";
    return result;
  }

  if (input.task_type != "inspection" && input.task_type != "abnormal" &&
      input.task_type != "collection") {
    result.error_code = static_cast<std::int32_t>(ErrorCode::kInvalidGoal);
    result.reason = "unknown task_type";
    return result;
  }

  if (input.valid_until_sec <= input.now_sec) {
    result.error_code = static_cast<std::int32_t>(ErrorCode::kGoalExpired);
    result.reason = "goal expired";
    return result;
  }

  if (!input.robot_ready) {
    result.error_code = static_cast<std::int32_t>(ErrorCode::kSafetyRejected);
    result.reason = "robot not ready";
    return result;
  }

  // constraints 校验：禁止性约束一律拒绝
  if (HasForbiddenConstraint(input.constraints)) {
    result.error_code = static_cast<std::int32_t>(ErrorCode::kSafetyRejected);
    result.reason = "constraint attempts to disable safety limits";
    return result;
  }

  // constraints 校验：max_speed 超过平台上限一律拒绝
  const double max_speed = ParseMaxSpeedMps(input.constraints);
  if (max_speed > kMaxAllowedSpeedMps) {
    result.error_code = static_cast<std::int32_t>(ErrorCode::kSafetyRejected);
    result.reason = "constraint max_speed exceeds platform limit";
    return result;
  }

  result.allowed = true;
  result.error_code = static_cast<std::int32_t>(ErrorCode::kOk);
  result.reason = "ok";
  return result;
}

}  // namespace tech_1_1
}  // namespace inspection_execution
