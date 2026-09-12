#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_7/position_matcher.hpp"

namespace inspection_execution {
namespace tech_1_7 {

// SemanticAlarm 消息字段（与 inspection_interfaces/msg/SemanticAlarm.msg 对齐）
//   header / detection_event / physical_location / bim_id / slam_id / confidence
struct SemanticAlarmFields {
  std::uint64_t timestamp_ns{0};
  std::string detection_event;     // event_id 或 alert_type
  double location_x{0.0};          // physical_location.position
  double location_y{0.0};
  double location_z{0.0};
  double orientation_qw{1.0};       // physical_location.orientation
  double orientation_qx{0.0};
  double orientation_qy{0.0};
  double orientation_qz{0.0};
  std::string bim_id;
  std::string slam_id;
  double confidence{0.0};
  bool localized{false};           // 红线标记：未定位时不写物理位置
};

/// 报警位置发布器（技术 1.7）
///
/// 把 PositionMatchResult 封装为 SemanticAlarm 消息字段。
///
/// 红线（与 SemanticAlarm.msg 设计约束一致）：
///   - 位置结果必须包含 confidence + bim_id/slam_id（坐标来源）
///   - 未定位（localized=false）时保持"未定位"：
///       confidence=0, bim_id="", slam_id="", 位置保持 0（不写虚假物理监测点）
///   - 不虚构物理监测点
class AlarmLocationPublisher {
 public:
  /// 把单个匹配结果封装为 SemanticAlarm 字段
  /// 未定位事件也发布（带 localized=false 标记，让上层知晓事件到达但无法定位）
  static SemanticAlarmFields Publish(const PositionMatchResult& result);

  /// 批量封装
  static std::vector<SemanticAlarmFields> PublishAll(
      const std::vector<PositionMatchResult>& results);

  /// 统计：本批次成功定位率
  static double LocalizedRatio(const std::vector<PositionMatchResult>& results);
};

}  // namespace tech_1_7
}  // namespace inspection_execution
