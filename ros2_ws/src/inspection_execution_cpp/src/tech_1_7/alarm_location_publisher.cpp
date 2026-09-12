#include "inspection_execution_cpp/tech_1_7/alarm_location_publisher.hpp"

namespace inspection_execution {
namespace tech_1_7 {

SemanticAlarmFields AlarmLocationPublisher::Publish(
    const PositionMatchResult& result) {
  SemanticAlarmFields fields;
  fields.timestamp_ns = result.timestamp_ns;
  fields.detection_event = result.event_id;
  fields.bim_id = result.bim_id;
  fields.slam_id = result.slam_id;

  if (result.localized) {
    // 成功定位：写入物理位置 + 置信度 + 坐标来源标识
    fields.location_x = result.matched_x;
    fields.location_y = result.matched_y;
    fields.location_z = result.matched_z;
    fields.confidence = result.confidence;
    fields.localized = true;
    // orientation 保持单位四元数（语义报警不带姿态）
    fields.orientation_qw = 1.0;
  } else {
    // 红线：未定位时保持"未定位"
    //   不写虚假物理监测点（位置保持 0），confidence=0
    //   bim_id/slam_id 已清空，让上层知晓事件到达但无法定位
    fields.location_x = 0.0;
    fields.location_y = 0.0;
    fields.location_z = 0.0;
    fields.confidence = 0.0;
    fields.bim_id.clear();
    fields.slam_id.clear();
    fields.localized = false;
  }
  return fields;
}

std::vector<SemanticAlarmFields> AlarmLocationPublisher::PublishAll(
    const std::vector<PositionMatchResult>& results) {
  std::vector<SemanticAlarmFields> out;
  out.reserve(results.size());
  for (const auto& r : results) {
    out.push_back(Publish(r));
  }
  return out;
}

double AlarmLocationPublisher::LocalizedRatio(
    const std::vector<PositionMatchResult>& results) {
  if (results.empty()) return 0.0;
  std::size_t localized = 0;
  for (const auto& r : results) {
    if (r.localized) ++localized;
  }
  return static_cast<double>(localized) / static_cast<double>(results.size());
}

}  // namespace tech_1_7
}  // namespace inspection_execution
