#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/common/data_freshness_guard.hpp"

namespace inspection_execution {
namespace tech_1_8 {

/// 红外测温原始帧（30fps 视觉链路）
struct ThermalFrame {
  double raw_temperature{0.0};    // 原始红外测温值（℃）
  double angle{0.0};             // 测量角度（°，相对光轴）
  double distance{0.0};          // 测量距离（m）
  double emissivity{1.0};        // 目标辐射率
  double correction_factor{1.0}; // 修正系数
  std::uint64_t timestamp_ns{0};
  bool valid{false};
};

/// 同步后的红外 + IMU 数据包
/// 红外 30fps 与 IMU 1000Hz 通过滑窗匹配对齐（IMU 为最高频基准）
struct SyncedThermalPacket {
  ThermalFrame thermal{};
  ImuSample imu{};
  bool thermal_valid{false};
  bool imu_valid{false};
  std::uint64_t synced_timestamp_ns{0};  // 取 IMU 时间戳为基准
};

/// 红外 / 视觉 / IMU 时间同步器（技术 1.8）
///
/// 对齐 30fps 红外帧与 1000Hz IMU：
///   - IMU 为最高频基准，缺 IMU 时不同步（不虚构 IMU 数据，红线）
///   - 滑窗匹配：在 tolerance 内取最近一帧红外 + 最近一个 IMU 样本
///   - 数据超时自动标记 invalid（复用 common/DataFreshnessGuard）
///
/// 红线：
///   - IMU 无效/超时时返回 std::nullopt，不虚构同步结果
///   - 红外帧超时 → thermal_valid=false，但仍输出（仅 IMU 有效时）
class ThermalVisualImuSynchronizer {
 public:
  static constexpr double kDefaultToleranceMs = 5.0;  // 5ms 容差（30fps≈33ms）

  explicit ThermalVisualImuSynchronizer(double tolerance_ms = kDefaultToleranceMs);

  /// 推入红外帧（30fps）
  void UpdateThermal(const ThermalFrame& frame);

  /// 推入 IMU 样本（1000Hz）
  void UpdateImu(const ImuSample& sample);

  /// 尝试同步：返回最新对齐的数据包
  /// IMU 无效或超时 → 返回 std::nullopt（不虚构）
  std::optional<SyncedThermalPacket> TrySync();

  /// IMU 是否有效（最近 tolerance 内有数据）
  bool IsImuValid() const;

  /// 红外是否有效（最近 tolerance 内有数据）
  bool IsThermalValid() const;

  /// 容差（ms）
  double ToleranceMs() const;

 private:
  DataFreshnessGuard thermal_freshness_;
  DataFreshnessGuard imu_freshness_;
  ThermalFrame latest_thermal_{};
  ImuSample latest_imu_{};
  bool thermal_valid_{false};
  bool imu_valid_{false};
  double tolerance_ms_{kDefaultToleranceMs};
};

}  // namespace tech_1_8
}  // namespace inspection_execution
