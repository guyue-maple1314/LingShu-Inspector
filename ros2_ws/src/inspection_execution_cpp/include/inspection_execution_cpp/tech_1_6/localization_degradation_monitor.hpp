#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_6/multi_sensor_synchronizer.hpp"

namespace inspection_execution {
namespace tech_1_6 {

// 定位退化等级
enum class DegradationLevel : int {
  kHealthy = 0,     // 激光有效，全源融合
  kDegraded = 1,    // 激光失效，视觉+惯导+运动学+足力维持
  kLost = 2,        // 关键源不足，定位不可用
};

// 退化监控结果
struct DegradationStatus {
  DegradationLevel level{DegradationLevel::kHealthy};
  bool lidar_valid{false};
  bool visual_valid{false};
  bool imu_valid{false};
  bool foot_force_valid{false};
  std::vector<std::string> valid_sources;     // 有效数据源名
  std::string localization_state{"healthy"}; // healthy/degraded/lost
  std::uint64_t timestamp_ns{0};
};

/// 定位退化监控器
/// 识别激光失效并标记有效数据源；激光失效时不伪造有效状态（红线）
/// 退化条件：激光失效 → 依赖视觉+惯导+运动学+足力维持定位
class LocalizationDegradationMonitor {
 public:
  static constexpr std::size_t kLostSourceThreshold = 2;  // 有效源 < 2 → lost

  LocalizationDegradationMonitor();

  /// 根据同步数据包评估退化状态
  DegradationStatus Evaluate(const SyncedSensorPacket& packet);

  /// 最近一次退化状态
  const DegradationStatus& Latest() const;

  /// 当前退化等级
  DegradationLevel Level() const;

  /// 当前有效数据源
  const std::vector<std::string>& ValidSources() const;

 private:
  DegradationStatus latest_{};
};

}  // namespace tech_1_6
}  // namespace inspection_execution
