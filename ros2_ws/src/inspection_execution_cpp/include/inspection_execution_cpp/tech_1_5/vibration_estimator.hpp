#pragma once

#include <array>
#include <cstdint>

#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/tech_1_5/foot_contact_estimator.hpp"

namespace inspection_execution {
namespace tech_1_5 {

// 振动严重等级
enum class VibrationSeverity : int {
  kNone = 0,     // 无异常
  kMild = 1,     // 轻微振动
  kModerate = 2, // 中度振动（影响步态）
  kSevere = 3,   // 严重共振（需立即修正）
};

// 振动估计结果
struct VibrationState {
  double accel_rms{0.0};           // 加速度 RMS (m/s²)
  double dominant_freq_hz{0.0};   // 估计主频
  VibrationSeverity severity{VibrationSeverity::kNone};
  bool resonance_detected{false}; // 钢格网共振特征
  std::uint64_t timestamp_ns{0};

  // 4 足振动分量（力抖动 RMS）
  std::array<double, 4> foot_force_rms{};
};

/// 振动估计器：融合 500Hz 足力 + 1000Hz IMU → 振动状态
/// 滑窗 RMS + 零交叉频率估计，轻量适配 1000Hz
class VibrationEstimator {
 public:
  static constexpr std::size_t kImuWindow = 32;       // IMU 滑窗（1000Hz × 32 = 32ms）
  static constexpr std::size_t kForceWindow = 16;      // 足力滑窗（500Hz × 16 = 32ms）
  static constexpr double kMildThreshold = 1.5;        // 轻微阈值 (m/s²)
  static constexpr double kModerateThreshold = 3.0;   // 中度阈值
  static constexpr double kSevereThreshold = 5.0;      // 严重阈值
  static constexpr double kResonanceFreqMin = 3.0;     // 钢格网共振频率下界
  static constexpr double kResonanceFreqMax = 12.0;   // 上界

  VibrationEstimator();

  /// 更新 IMU 数据（1000Hz 调用）
  void UpdateImu(const ImuSample& sample);

  /// 更新足力数据（500Hz 调用）
  void UpdateFootForce(const FootForceSample& sample);

  /// 计算当前振动状态
  VibrationState Estimate() const;

 private:
  // IMU 滑窗
  std::array<double, kImuWindow> imu_abs_buf_{};
  std::size_t imu_head_{0};
  bool imu_filled_{false};
  double imu_sum_sq_{0.0};  // sum of mag² for proper RMS
  std::array<double, kImuWindow> imu_ax_buf_{};  // 用于零交叉频率估计
  std::size_t imu_ax_head_{0};
  std::uint64_t imu_count_{0};

  // 足力滑窗
  struct ForceBuf {
    std::array<double, kForceWindow> buf{};
    std::size_t head{0};
    bool filled{false};
    double sum{0.0};
    double sum_sq{0.0};  // for RMS
  };
  std::array<ForceBuf, 4> force_windows_{};

  std::uint64_t latest_ts_{0};

  double ComputeRms() const;
  double EstimateFreq() const;
  static VibrationSeverity Classify(double rms);
};

}  // namespace tech_1_5
}  // namespace inspection_execution
