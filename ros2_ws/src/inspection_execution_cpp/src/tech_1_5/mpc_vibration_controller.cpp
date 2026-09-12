#include "inspection_execution_cpp/tech_1_5/mpc_vibration_controller.hpp"

#include <algorithm>
#include <cmath>

namespace inspection_execution {
namespace tech_1_5 {

MpcVibrationController::MpcVibrationController(
    std::shared_ptr<AbstractMpcSolver> solver)
    : solver_(std::move(solver)) {}

bool MpcVibrationController::ComputeCorrection(const MpcSolveInput& input,
                                               MpcCorrection* output) {
  if (!solver_ || !solver_->IsReady()) {
    return false;
  }

  // 振动低于激活阈值时不修正
  if (input.vibration.severity < activation_threshold_) {
    latest_correction_.active = false;
    latest_correction_.timestamp_ns = input.timestamp_ns;
    if (output) *output = latest_correction_;
    return false;
  }

  if (!solver_->Solve(input, output)) {
    return false;
  }

  output->active = true;
  output->timestamp_ns = input.timestamp_ns;
  latest_correction_ = *output;
  return true;
}

bool MpcVibrationController::IsActive() const {
  return latest_correction_.active;
}

const MpcCorrection& MpcVibrationController::LatestCorrection() const {
  return latest_correction_;
}

void MpcVibrationController::SetActivationSeverity(VibrationSeverity threshold) {
  activation_threshold_ = threshold;
}

// --- FakeMpcSolver ---

bool FakeMpcSolver::Solve(const MpcSolveInput& input,
                          MpcCorrection* output) {
  if (!initialized_ || !output) return false;
  if (input.vibration.severity == VibrationSeverity::kNone) return false;

  // 确定性阻尼修正：根据振动幅值线性生成阻尼系数
  const double rms = input.vibration.accel_rms;
  const double damping = std::min(1.0, rms / VibrationEstimator::kSevereThreshold);

  output->damping_factor = damping;

  // 扭矩修正：对支撑腿施加反向阻尼力矩
  for (std::size_t i = 0; i < 12; ++i) {
    // 交替正负，模拟对腿对的反向修正
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    output->torque_corrections[i] = sign * damping * 2.0;  // 最大 2 N·m
    output->position_offsets[i] = sign * damping * 0.01;   // 最大 0.01 rad
  }

  return true;
}

// --- ApplyCorrection ---

JointCommand ApplyCorrection(const JointCommand& base_cmd,
                             const MpcCorrection& correction,
                             double max_torque_correction,
                             double max_position_offset) {
  JointCommand result = base_cmd;

  // 扭矩修正
  if (result.torques.size() >= kTotalJoints) {
    for (std::size_t i = 0; i < kTotalJoints; ++i) {
      double t = result.torques[i] + correction.torque_corrections[i];
      t = std::clamp(t, -max_torque_correction * 10, max_torque_correction * 10);
      result.torques[i] = t;
    }
  }

  // 位置修正
  if (result.positions.size() >= kTotalJoints) {
    for (std::size_t i = 0; i < kTotalJoints; ++i) {
      double p = result.positions[i] + correction.position_offsets[i];
      p = std::clamp(p, -max_position_offset * 50, max_position_offset * 50);
      result.positions[i] = p;
    }
  }

  return result;
}

}  // namespace tech_1_5
}  // namespace inspection_execution
