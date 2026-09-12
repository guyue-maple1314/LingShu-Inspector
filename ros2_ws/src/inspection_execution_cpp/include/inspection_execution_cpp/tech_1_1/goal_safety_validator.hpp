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
  /// 本平台允许的最大行进速度（m/s）：constraints 中若给出更大的 max_speed 则拒绝
  static constexpr double kMaxAllowedSpeedMps = 1.5;

  GoalSafetyResult Validate(const GoalSafetyInput& input) const;

  /// constraints 文本是否包含禁止性约束（试图关闭安全/限速/围栏等）
  static bool HasForbiddenConstraint(const std::string& constraints);

  /// 从 constraints 文本中解析 max_speed=<数值>；无该字段返回 0
  static double ParseMaxSpeedMps(const std::string& constraints);
};

}  // namespace tech_1_1
}  // namespace inspection_execution
