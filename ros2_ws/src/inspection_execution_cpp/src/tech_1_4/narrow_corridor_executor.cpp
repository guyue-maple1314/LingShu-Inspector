#include "inspection_execution_cpp/tech_1_4/narrow_corridor_executor.hpp"

namespace inspection_execution {
namespace tech_1_4 {

NarrowCorridorExecutor::NarrowCorridorExecutor(double safety_margin)
    : safety_margin_(safety_margin) {}

bool NarrowCorridorExecutor::Start(double robot_width, double corridor_width) {
  if (corridor_width < robot_width + safety_margin_) {
    state_ = CorridorExecutionState::kFailed;
    return false;
  }
  robot_width_ = robot_width;
  progress_ = 0.0;
  state_ = CorridorExecutionState::kTracking;
  return true;
}

bool NarrowCorridorExecutor::Update(double corridor_width, bool visual_valid) {
  if (state_ != CorridorExecutionState::kTracking) {
    return state_ == CorridorExecutionState::kCompleted;
  }
  if (!visual_valid || corridor_width < robot_width_ + safety_margin_) {
    state_ = CorridorExecutionState::kBlocked;
    return false;
  }
  progress_ += 0.1;
  if (progress_ >= 1.0) {
    progress_ = 1.0;
    state_ = CorridorExecutionState::kCompleted;
  }
  return state_ == CorridorExecutionState::kCompleted;
}

void NarrowCorridorExecutor::Complete() {
  state_ = CorridorExecutionState::kCompleted;
  progress_ = 1.0;
}

CorridorExecutionState NarrowCorridorExecutor::State() const {
  return state_;
}

double NarrowCorridorExecutor::Progress() const {
  return progress_;
}

}  // namespace tech_1_4
}  // namespace inspection_execution
