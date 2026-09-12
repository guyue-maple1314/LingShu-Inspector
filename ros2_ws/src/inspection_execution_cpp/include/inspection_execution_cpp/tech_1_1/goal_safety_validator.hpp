#pragma once

#include <cstdint>
#include <string>

namespace inspection_execution {
namespace tech_1_1 {

struct GoalSafetyInput {
  std::string goal_id;
  std::string task_type;
  double valid_until_sec{0.0};
  double now_sec{0.0};
  bool robot_ready{false};
  std::string constraints;
};

struct GoalSafetyResult {
  bool allowed{false};
  std::int32_t error_code{0};
  std::string reason;
};

class GoalSafetyValidator {
 public:
  GoalSafetyResult Validate(const GoalSafetyInput& input) const;
};

}  // namespace tech_1_1
}  // namespace inspection_execution
