#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace inspection_execution {
namespace tech_1_5 {

// 单步态异常事件
struct GaitAnomalyEvent {
  std::uint64_t timestamp_ns{0};
  int foot_index{0};
  std::string anomaly_type;  // "false_contact", "resonance", "stumble"
  double severity{0.0};
};

// 钢格网运行指标快照
struct GratingMetrics {
  std::uint64_t total_steps{0};        // 总步数
  std::uint64_t anomaly_count{0};      // 异常步数
  double anomaly_rate{0.0};            // 异常率
  double avg_speed{0.0};              // 平均速度
  double total_distance{0.0};         // 累计巡检距离
  double max_vibration_rms{0.0};      // 最大振动 RMS
  bool grating_mode_active{false};   // 钢格网模式是否激活
  std::uint64_t measurement_start_ns{0};
  std::uint64_t measurement_end_ns{0};
};

/// 钢格网指标记录器（线程安全，供高频线程调用）
class GratingMetricsRecorder {
 public:
  GratingMetricsRecorder();

  /// 记录一步（高频线程调用，加锁）
  void RecordStep(std::uint64_t ts_ns);

  /// 记录异常事件
  void RecordAnomaly(const GaitAnomalyEvent& event);

  /// 更新速度（由编码器/里程计反馈，非高频线程）
  void UpdateSpeed(double speed_mps, std::uint64_t ts_ns);

  /// 更新振动峰值
  void UpdateVibration(double rms);

  /// 设置钢格网模式
  void SetGratingMode(bool active);

  /// 获取当前指标快照（线程安全拷贝）
  GratingMetrics Snapshot() const;

  /// 重置
  void Reset();

  // PPT 阈值
  static constexpr double kAnomalyRateTarget = 0.15;   // ≤15%（即降低 85%）
  static constexpr double kMinAvgSpeed = 0.8;           // ≥0.8 m/s
  static constexpr double kMinDistance = 2000.0;        // ≥2 km = 2000 m

  /// 评估 PPT 阈值是否通过
  bool ThresholdsPass() const;

 private:
  mutable std::mutex mtx_;
  std::uint64_t total_steps_{0};
  std::uint64_t anomaly_count_{0};
  double speed_sum_{0.0};
  std::uint64_t speed_count_{0};
  double total_distance_{0.0};
  double max_vibration_rms_{0.0};
  bool grating_mode_active_{false};
  std::uint64_t start_ns_{0};
  std::uint64_t last_ts_{0};
  double last_speed_{0.0};
};

}  // namespace tech_1_5
}  // namespace inspection_execution
