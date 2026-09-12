#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"

namespace inspection_execution {
namespace tech_1_6 {

// 约束类型
enum class ConstraintType : int {
  kJointPositionLimit = 0,  // 关节角度限位
  kJointVelocityLimit = 1,  // 关节速度限位
  kFootForceLimit = 2,      // 足端力 ≤50N 约束
  kContactConstraint = 3,   // 接触约束
};

// 单条约束
struct KinematicConstraint {
  ConstraintType type{ConstraintType::kJointPositionLimit};
  int joint_index{-1};
  double min_value{0.0};
  double max_value{0.0};
  double current_value{0.0};
  bool satisfied{true};
};

// 运动学约束集合
struct ConstraintSet {
  std::vector<KinematicConstraint> constraints;
  bool all_satisfied{true};
  std::uint64_t timestamp_ns{0};
};

/// 运动学约束构建器
/// 从机器人状态 + 足端力生成运动学及 50N 足端力约束
/// 复用 RobotStateRaw + FootForceSample，不重复造适配器
class KinematicConstraintBuilder {
 public:
  static constexpr double kFootForceLimit = 50.0;   // PPT: 50N 足端力
  static constexpr double kJointPosLimit = 3.14;     // ±π rad
  static constexpr double kJointVelLimit = 10.0;     // ±10 rad/s

  /// 从机器人状态构建关节限位约束
  ConstraintSet BuildFromRobotState(const RobotStateRaw& state,
                                     std::uint64_t ts_ns) const;

  /// 从足端力构建 50N 约束
  ConstraintSet BuildFromFootForce(const FootForceSample& sample) const;

  /// 合并约束集
  static ConstraintSet Merge(const ConstraintSet& a, const ConstraintSet& b);
};

}  // namespace tech_1_6
}  // namespace inspection_execution
