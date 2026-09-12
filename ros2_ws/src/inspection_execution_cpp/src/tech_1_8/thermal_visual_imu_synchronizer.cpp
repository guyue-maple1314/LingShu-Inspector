#include "inspection_execution_cpp/tech_1_8/thermal_visual_imu_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_8 {

ThermalVisualImuSynchronizer::ThermalVisualImuSynchronizer(double tolerance_ms)
    : thermal_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))),
      imu_freshness_(std::chrono::milliseconds(
          static_cast<std::int64_t>(tolerance_ms))),
      tolerance_ms_(tolerance_ms) {}

void ThermalVisualImuSynchronizer::UpdateThermal(const ThermalFrame& frame) {
  latest_thermal_ = frame;
  thermal_valid_ = frame.valid;
  if (frame.valid) {
    thermal_freshness_.Update(DataFreshnessGuard::Clock::now());
  }
}

void ThermalVisualImuSynchronizer::UpdateImu(const ImuSample& sample) {
  latest_imu_ = sample;
  imu_valid_ = true;
  imu_freshness_.Update(DataFreshnessGuard::Clock::now());
}

std::optional<SyncedThermalPacket> ThermalVisualImuSynchronizer::TrySync() {
  // IMU 为最高频基准，缺 IMU 时不同步（不虚构 IMU 数据，红线）
  const auto now = DataFreshnessGuard::Clock::now();
  if (!imu_valid_ || !imu_freshness_.IsFresh(now)) {
    return std::nullopt;
  }

  SyncedThermalPacket packet;
  packet.imu = latest_imu_;
  packet.imu_valid = true;
  // 红外超时 → thermal_valid=false，但仍输出包（IMU 有效时）
  packet.thermal = latest_thermal_;
  packet.thermal_valid =
      thermal_valid_ && thermal_freshness_.IsFresh(now);
  // 同步时间戳取 IMU 时间戳（最高频基准）
  packet.synced_timestamp_ns = latest_imu_.timestamp_ns;
  return packet;
}

bool ThermalVisualImuSynchronizer::IsImuValid() const {
  const auto now = DataFreshnessGuard::Clock::now();
  return imu_valid_ && imu_freshness_.IsFresh(now);
}

bool ThermalVisualImuSynchronizer::IsThermalValid() const {
  const auto now = DataFreshnessGuard::Clock::now();
  return thermal_valid_ && thermal_freshness_.IsFresh(now);
}

double ThermalVisualImuSynchronizer::ToleranceMs() const {
  return tolerance_ms_;
}

}  // namespace tech_1_8
}  // namespace inspection_execution
