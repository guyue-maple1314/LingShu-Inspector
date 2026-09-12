#include "inspection_execution_cpp/tech_1_6/localization_degradation_monitor.hpp"

namespace inspection_execution {
namespace tech_1_6 {

LocalizationDegradationMonitor::LocalizationDegradationMonitor() = default;

DegradationStatus LocalizationDegradationMonitor::Evaluate(
    const SyncedSensorPacket& packet) {
  DegradationStatus status;
  status.lidar_valid = packet.lidar_valid;
  status.visual_valid = packet.camera_left_valid || packet.camera_right_valid;
  status.imu_valid = packet.imu_valid;
  status.foot_force_valid = packet.foot_force_valid;
  status.timestamp_ns = packet.synced_timestamp_ns;

  // 有效数据源列表（不伪造激光有效状态，红线）
  status.valid_sources.clear();
  if (packet.lidar_valid)        status.valid_sources.push_back("lidar");
  if (packet.camera_left_valid)  status.valid_sources.push_back("camera_left");
  if (packet.camera_right_valid) status.valid_sources.push_back("camera_right");
  if (packet.imu_valid)          status.valid_sources.push_back("imu");
  if (packet.foot_force_valid)   status.valid_sources.push_back("foot_force");

  const std::size_t src_count = status.valid_sources.size();

  // 退化等级判定
  // healthy：激光有效且源数 >= 3
  // degraded：激光失效但视觉+惯导+运动学+足力维持（源数 >= 2）
  // lost：关键源不足（源数 < 2）
  if (status.lidar_valid && src_count >= 3) {
    status.level = DegradationLevel::kHealthy;
    status.localization_state = "healthy";
  } else if (!status.lidar_valid && src_count >= 2) {
    // 激光失效 → 降级，不伪造激光有效状态
    status.level = DegradationLevel::kDegraded;
    status.localization_state = "degraded";
  } else if (status.lidar_valid && src_count < 3) {
    // 激光有效但源不足 → 仍标记降级
    status.level = DegradationLevel::kDegraded;
    status.localization_state = "degraded";
  } else {
    // 源数 < 阈值 → 定位丢失
    status.level = DegradationLevel::kLost;
    status.localization_state = "lost";
  }

  latest_ = status;
  return status;
}

const DegradationStatus& LocalizationDegradationMonitor::Latest() const {
  return latest_;
}

DegradationLevel LocalizationDegradationMonitor::Level() const {
  return latest_.level;
}

const std::vector<std::string>& LocalizationDegradationMonitor::ValidSources()
    const {
  return latest_.valid_sources;
}

}  // namespace tech_1_6
}  // namespace inspection_execution
