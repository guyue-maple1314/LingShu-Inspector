#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"
#include "inspection_execution_cpp/common/execution_result.hpp"

namespace inspection_execution {
namespace tech_1_3 {

// 每条腿的关节数：四足每条腿 3 关节 → 共 12 关节
inline constexpr std::size_t kJointsPerLeg = 3;
inline constexpr std::size_t kLegCount = 4;
inline constexpr std::size_t kTotalJoints = kJointsPerLeg * kLegCount;

// LocomotionCommandAdapter：把策略 action 向量 → RobotSdkAdapter 的 JointCommand
//
// 关键约束：不直接调用厂商 SDK，只通过 RobotSdkAdapter 抽象。
// 本类只做维度映射和安全缩放，不做 PPT 1.5 的 MPC，也不碰 1.1 的 PPO runtime。
class LocomotionCommandAdapter {
 public:
  explicit LocomotionCommandAdapter(std::shared_ptr<RobotSdkAdapter> robot);
  ~LocomotionCommandAdapter() = default;

  // 把每条 action 维度 [−1,+1] 映射到关节位置增量 rad。默认 ±0.04rad。
  void SetJointDeltaRangeRad(double range_per_dim);
  double JointDeltaRangeRad() const { return joint_delta_range_rad_; }

  // 映射模式：action → positions（默认）或 velocities 或 torques。
  enum class TargetField { kPositions, kVelocities, kTorques };
  void SetTargetField(TargetField f);
  TargetField CurrentTargetField() const { return target_; }

  // 执行转换；失败时不会 SendJointCommand 到适配器。
  ExecutionResult ConvertAndSend(const std::vector<float>& action);

  // 仅转换，不发送 — 便于 unittest 和 smoke test。
  ExecutionResult ConvertOnly(const std::vector<float>& action, JointCommand* out) const;

 private:
  std::shared_ptr<RobotSdkAdapter> robot_;
  double joint_delta_range_rad_;
  TargetField target_;
};

}  // namespace tech_1_3
}  // namespace inspection_execution
