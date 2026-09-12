#include "inspection_execution_cpp/tech_1_3/locomotion_command_adapter.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace inspection_execution {
namespace tech_1_3 {

LocomotionCommandAdapter::LocomotionCommandAdapter(std::shared_ptr<RobotSdkAdapter> robot)
    : robot_(std::move(robot)), joint_delta_range_rad_(0.04), target_(TargetField::kPositions) {}

void LocomotionCommandAdapter::SetJointDeltaRangeRad(double range_per_dim) {
  joint_delta_range_rad_ = range_per_dim > 0.0 ? range_per_dim : 0.0;
}

void LocomotionCommandAdapter::SetTargetField(TargetField f) { target_ = f; }

ExecutionResult LocomotionCommandAdapter::ConvertOnly(const std::vector<float>& action,
                                                      JointCommand* out) const {
  if (!out) return ExecutionResult::Failure(4001, "null JointCommand output");
  if (action.size() != kTotalJoints) {
    return ExecutionResult::Failure(
        4001, "action size must be " + std::to_string(kTotalJoints) + "; got " +
                  std::to_string(action.size()));
  }
  if (!robot_) {
    return ExecutionResult::Failure(4001, "RobotSdkAdapter is null (adapter not bound)");
  }

  out->positions.assign(kTotalJoints, 0.0);
  out->velocities.assign(kTotalJoints, 0.0);
  out->torques.assign(kTotalJoints, 0.0);

  const double scale = joint_delta_range_rad_;
  std::vector<double>* target_vec = nullptr;
  switch (target_) {
    case TargetField::kPositions:
      target_vec = &out->positions;
      break;
    case TargetField::kVelocities:
      target_vec = &out->velocities;
      break;
    case TargetField::kTorques:
      target_vec = &out->torques;
      break;
  }
  if (!target_vec) return ExecutionResult::Failure(4001, "unknown TargetField");
  for (std::size_t i = 0; i < kTotalJoints; ++i) {
    const double v = static_cast<double>(action[i]) * scale;
    (*target_vec)[i] = v;
  }
  return ExecutionResult::Ok();
}

ExecutionResult LocomotionCommandAdapter::ConvertAndSend(const std::vector<float>& action) {
  JointCommand cmd;
  const auto r = ConvertOnly(action, &cmd);
  if (!r.success) return r;
  if (!robot_ || !robot_->IsConnected()) {
    return ExecutionResult::Failure(4001, "robot SDK not connected (send skipped)");
  }
  const bool ok = robot_->SendJointCommand(cmd);
  return ok ? ExecutionResult::Ok("joint command sent via RobotSdkAdapter")
            : ExecutionResult::Failure(4001, "RobotSdkAdapter::SendJointCommand returned false");
}

}  // namespace tech_1_3
}  // namespace inspection_execution
