#include "inspection_execution_cpp/tech_1_6/tightly_coupled_localizer.hpp"

namespace inspection_execution {
namespace tech_1_6 {

TightlyCoupledLocalizer::TightlyCoupledLocalizer(
    std::shared_ptr<AbstractLocalizerBackend> backend)
    : backend_(std::move(backend)) {}

bool TightlyCoupledLocalizer::Localize(const LocalizationSolveInput& input,
                                       FusionPoseOutput* output) {
  if (!backend_ || !backend_->IsReady() || !output) {
    return false;
  }

  if (!backend_->Solve(input, output)) {
    return false;
  }

  output->timestamp_ns = input.timestamp_ns;
  latest_pose_ = *output;
  valid_sources_ = output->valid_sources;
  localization_state_ = output->localization_state;
  return output->valid;
}

const FusionPoseOutput& TightlyCoupledLocalizer::LatestPose() const {
  return latest_pose_;
}

const std::vector<std::string>& TightlyCoupledLocalizer::ValidSources() const {
  return valid_sources_;
}

const std::string& TightlyCoupledLocalizer::LocalizationState() const {
  return localization_state_;
}

// --- FakeLocalizerBackend ---
// 确定性位姿输出，不虚构 SLAM 结果
// 仅在后端已 Initialize 且传感器数据非空时返回有效位姿
bool FakeLocalizerBackend::Solve(const LocalizationSolveInput& input,
                                  FusionPoseOutput* output) {
  if (!initialized_ || !output) return false;

  // 至少需要 IMU 有效（最低定位源）
  if (!input.sensors.imu_valid) {
    output->valid = false;
    output->localization_state = "lost";
    return false;
  }

  // 确定性位姿：基于上一帧位姿 + 零增量（不虚构运动）
  output->x = input.prev_pose.x;
  output->y = input.prev_pose.y;
  output->z = input.prev_pose.z;
  output->qw = input.prev_pose.qw;
  output->qx = input.prev_pose.qx;
  output->qy = input.prev_pose.qy;
  output->qz = input.prev_pose.qz;

  // 协方差：源越多越小（简化模型）
  std::size_t source_count = 0;
  if (input.sensors.lidar_valid) ++source_count;
  if (input.sensors.camera_left_valid) ++source_count;
  if (input.sensors.camera_right_valid) ++source_count;
  if (input.sensors.imu_valid) ++source_count;
  if (input.sensors.foot_force_valid) ++source_count;

  const double cov_scale = (source_count >= 3) ? 0.01 : 0.04;
  for (std::size_t i = 0; i < 6; ++i) {
    output->covariance[i] = cov_scale;
  }

  // 有效数据源列表（不伪造激光有效状态）
  output->valid_sources.clear();
  if (input.sensors.lidar_valid)        output->valid_sources.push_back("lidar");
  if (input.sensors.camera_left_valid)  output->valid_sources.push_back("camera_left");
  if (input.sensors.camera_right_valid) output->valid_sources.push_back("camera_right");
  if (input.sensors.imu_valid)         output->valid_sources.push_back("imu");
  if (input.sensors.foot_force_valid)  output->valid_sources.push_back("foot_force");

  // 定位状态判定
  if (input.sensors.lidar_valid && source_count >= 3) {
    output->localization_state = "tracking";
  } else if (!input.sensors.lidar_valid && source_count >= 2) {
    output->localization_state = "degraded";  // 激光失效，降级融合
  } else {
    output->localization_state = "degraded";
  }

  // 约束全部满足时定位有效
  output->valid = input.constraints.all_satisfied && source_count >= 2;
  return output->valid;
}

}  // namespace tech_1_6
}  // namespace inspection_execution
