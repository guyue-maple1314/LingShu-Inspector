#include "inspection_execution_cpp/tech_1_6/multi_sensor_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_6 {

MultiSensorSynchronizer::MultiSensorSynchronizer(double tolerance_ms)
    : lidar_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))),
      cam_left_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))),
      cam_right_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))),
      imu_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))),
      foot_force_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))) {}

void MultiSensorSynchronizer::UpdateLidar(std::uint64_t ts_ns, bool valid) {
  lidar_ts_ = ts_ns;
  lidar_valid_ = valid;
  if (valid) {
    lidar_freshness_.Update(DataFreshnessGuard::Clock::now());
  }
}

void MultiSensorSynchronizer::UpdateCamera(int cam_idx, std::uint64_t ts_ns,
                                             bool valid) {
  if (cam_idx == 0) {
    cam_left_ts_ = ts_ns;
    cam_left_valid_ = valid;
    if (valid) cam_left_freshness_.Update(DataFreshnessGuard::Clock::now());
  } else if (cam_idx == 1) {
    cam_right_ts_ = ts_ns;
    cam_right_valid_ = valid;
    if (valid) cam_right_freshness_.Update(DataFreshnessGuard::Clock::now());
  }
}

void MultiSensorSynchronizer::UpdateImu(const ImuSample& sample) {
  latest_imu_ = sample;
  imu_valid_ = true;
  imu_freshness_.Update(DataFreshnessGuard::Clock::now());
}

void MultiSensorSynchronizer::UpdateFootForce(const FootForceSample& sample) {
  latest_foot_force_ = sample;
  foot_force_valid_ = true;
  foot_force_freshness_.Update(DataFreshnessGuard::Clock::now());
}

std::optional<SyncedSensorPacket> MultiSensorSynchronizer::TrySync() {
  // 至少 IMU 需有效且新鲜作为同步基准（1000Hz 最高频）
  const auto now = DataFreshnessGuard::Clock::now();
  if (!imu_valid_ || !imu_freshness_.IsFresh(now)) {
    return std::nullopt;
  }

  SyncedSensorPacket packet;
  packet.imu = latest_imu_;
  packet.foot_force = latest_foot_force_;
  packet.lidar_timestamp_ns = lidar_ts_;
  packet.camera_left_timestamp_ns = cam_left_ts_;
  packet.camera_right_timestamp_ns = cam_right_ts_;
  // 数据新鲜度检查：超时传感器标记为 invalid（不伪造有效状态）
  packet.lidar_valid = lidar_valid_ && lidar_freshness_.IsFresh(now);
  packet.camera_left_valid =
      cam_left_valid_ && cam_left_freshness_.IsFresh(now);
  packet.camera_right_valid =
      cam_right_valid_ && cam_right_freshness_.IsFresh(now);
  packet.imu_valid = true;
  packet.foot_force_valid =
      foot_force_valid_ && foot_force_freshness_.IsFresh(now);

  // 同步时间戳取 IMU 时间戳（最高频基准）
  packet.synced_timestamp_ns = latest_imu_.timestamp_ns;
  return packet;
}

bool MultiSensorSynchronizer::IsSensorValid(SensorType type) const {
  const auto now = DataFreshnessGuard::Clock::now();
  switch (type) {
    case SensorType::kLidar:
      return lidar_valid_ && lidar_freshness_.IsFresh(now);
    case SensorType::kCameraLeft:
      return cam_left_valid_ && cam_left_freshness_.IsFresh(now);
    case SensorType::kCameraRight:
      return cam_right_valid_ && cam_right_freshness_.IsFresh(now);
    case SensorType::kImu:
      return imu_valid_ && imu_freshness_.IsFresh(now);
    case SensorType::kFootForce:
      return foot_force_valid_ && foot_force_freshness_.IsFresh(now);
  }
  return false;
}

}  // namespace tech_1_6
}  // namespace inspection_execution
