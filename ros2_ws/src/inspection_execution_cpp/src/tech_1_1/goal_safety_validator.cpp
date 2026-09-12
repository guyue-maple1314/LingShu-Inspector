#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"

#include "inspection_execution_cpp/common/error_codes.hpp"

namespace inspection_execution {
namespace tech_1_1 {

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

  result.allowed = true;
  result.error_code = static_cast<std::int32_t>(ErrorCode::kOk);
  result.reason = "ok";
  return result;
}

}  // namespace tech_1_1
}  // namespace inspection_execution
