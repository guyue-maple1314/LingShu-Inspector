#include "inspection_execution_cpp/tech_1_6/kinematic_constraint_builder.hpp"

#include <cmath>

namespace inspection_execution {
namespace tech_1_6 {

ConstraintSet KinematicConstraintBuilder::BuildFromRobotState(
    const RobotStateRaw& state, std::uint64_t ts_ns) const {
  ConstraintSet set;
  set.timestamp_ns = ts_ns;

  // 关节位置限位约束
  const std::size_t n = state.joint_positions.size();
  for (std::size_t i = 0; i < n; ++i) {
    KinematicConstraint c;
    c.type = ConstraintType::kJointPositionLimit;
    c.joint_index = static_cast<int>(i);
    c.min_value = -kJointPosLimit;
    c.max_value = kJointPosLimit;
    c.current_value = state.joint_positions[i];
    c.satisfied = (c.current_value >= c.min_value &&
                   c.current_value <= c.max_value);
    if (!c.satisfied) set.all_satisfied = false;
    set.constraints.push_back(c);
  }

  // 关节速度限位约束
  const std::size_t nv = state.joint_velocities.size();
  for (std::size_t i = 0; i < nv; ++i) {
    KinematicConstraint c;
    c.type = ConstraintType::kJointVelocityLimit;
    c.joint_index = static_cast<int>(i);
    c.min_value = -kJointVelLimit;
    c.max_value = kJointVelLimit;
    c.current_value = state.joint_velocities[i];
    c.satisfied = (std::abs(c.current_value) <= kJointVelLimit);
    if (!c.satisfied) set.all_satisfied = false;
    set.constraints.push_back(c);
  }

  return set;
}

ConstraintSet KinematicConstraintBuilder::BuildFromFootForce(
    const FootForceSample& sample) const {
  ConstraintSet set;
  set.timestamp_ns = sample.timestamp_ns;

  for (std::size_t i = 0; i < 4; ++i) {
    KinematicConstraint c;
    c.type = ConstraintType::kFootForceLimit;
    c.joint_index = static_cast<int>(i);  // 足索引
    c.min_value = 0.0;
    c.max_value = kFootForceLimit;  // 50N
    c.current_value = sample.normal_forces[i];
    c.satisfied = (c.current_value <= kFootForceLimit);
    if (!c.satisfied) set.all_satisfied = false;
    set.constraints.push_back(c);
  }

  return set;
}

ConstraintSet KinematicConstraintBuilder::Merge(const ConstraintSet& a,
                                                const ConstraintSet& b) {
  ConstraintSet merged;
  merged.timestamp_ns = std::max(a.timestamp_ns, b.timestamp_ns);
  merged.constraints.reserve(a.constraints.size() + b.constraints.size());
  merged.constraints.insert(merged.constraints.end(),
                            a.constraints.begin(), a.constraints.end());
  merged.constraints.insert(merged.constraints.end(),
                            b.constraints.begin(), b.constraints.end());
  merged.all_satisfied = a.all_satisfied && b.all_satisfied;
  return merged;
}

}  // namespace tech_1_6
}  // namespace inspection_execution
