#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/common/data_freshness_guard.hpp"

namespace inspection_execution {
namespace tech_1_6 {

// 传感器类型枚举
enum class SensorType : int {
  kLidar = 0,
  kCameraLeft = 1,
  kCameraRight = 2,
  kImu = 3,
  kFootForce = 4,
};

// 同步后的传感器数据包
struct SyncedSensorPacket {
  ImuSample imu{};
  FootForceSample foot_force{};
  std::uint64_t lidar_timestamp_ns{0};
  std::uint64_t camera_left_timestamp_ns{0};
  std::uint64_t camera_right_timestamp_ns{0};
  bool lidar_valid{false};
  bool camera_left_valid{false};
  bool camera_right_valid{false};
  bool imu_valid{false};
  bool foot_force_valid{false};
  std::uint64_t synced_timestamp_ns{0};
};

/// 多传感器时间同步器
/// 对齐激光、多目相机、1000Hz IMU、500Hz 足端力的时间戳
/// 滑窗匹配：在 tolerance 内找最近的传感器样本
class MultiSensorSynchronizer {
 public:
  static constexpr double kDefaultToleranceMs = 5.0;  // 5ms 容差

  explicit MultiSensorSynchronizer(double tolerance_ms = kDefaultToleranceMs);

  /// 推入激光雷达时间戳
  void UpdateLidar(std::uint64_t ts_ns, bool valid = true);

  /// 推入相机时间戳
  void UpdateCamera(int cam_idx, std::uint64_t ts_ns, bool valid = true);

  /// 推入 IMU 样本
  void UpdateImu(const ImuSample& sample);

  /// 推入足力样本
  void UpdateFootForce(const FootForceSample& sample);

  /// 尝试同步：返回最新对齐的数据包
  std::optional<SyncedSensorPacket> TrySync();

  /// 各传感器是否有效（最近 tolerance 内有数据）
  bool IsSensorValid(SensorType type) const;

 private:
  // 每个传感器一个新鲜度守卫：数据超时自动标记 invalid
  // 复用 common/DataFreshnessGuard，消除死代码
  DataFreshnessGuard lidar_freshness_;
  DataFreshnessGuard cam_left_freshness_;
  DataFreshnessGuard cam_right_freshness_;
  DataFreshnessGuard imu_freshness_;
  DataFreshnessGuard foot_force_freshness_;

  ImuSample latest_imu_{};
  FootForceSample latest_foot_force_{};
  std::uint64_t lidar_ts_{0};
  std::uint64_t cam_left_ts_{0};
  std::uint64_t cam_right_ts_{0};
  bool lidar_valid_{false};
  bool cam_left_valid_{false};
  bool cam_right_valid_{false};
  bool imu_valid_{false};
  bool foot_force_valid_{false};
};

}  // namespace tech_1_6
}  // namespace inspection_execution
