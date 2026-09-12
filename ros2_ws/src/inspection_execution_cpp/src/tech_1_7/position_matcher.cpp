#include <algorithm>
#include <cmath>

#include "inspection_execution_cpp/tech_1_7/position_matcher.hpp"

namespace inspection_execution {
namespace tech_1_7 {

PositionMatcher::PositionMatcher(double search_radius_m)
    : search_radius_m_(search_radius_m > 0.0 ? search_radius_m
                                              : kDefaultSearchRadiusM) {}

double PositionMatcher::SearchRadius() const { return search_radius_m_; }

bool PositionMatcher::FindBestCell(const DetectionEvent& event,
                                    const WeightedGrid& grid,
                                    const GridCell** best_cell) const {
  *best_cell = nullptr;
  if (!grid.valid || grid.cells.empty()) return false;

  double best_weight = -1.0;
  const double r = search_radius_m_;
  const double r_sq = r * r;

  for (const auto& cell : grid.cells) {
    // 栅格中心坐标
    const double cx = grid.origin_x + (cell.index_x + 0.5) * grid.resolution_m;
    const double cy = grid.origin_y + (cell.index_y + 0.5) * grid.resolution_m;
    const double dx = cx - event.detection_x;
    const double dy = cy - event.detection_y;
    if (dx * dx + dy * dy > r_sq) continue;
    if (cell.weight > best_weight) {
      best_weight = cell.weight;
      *best_cell = &cell;
    }
  }
  return *best_cell != nullptr && best_weight > 0.0;
}

double PositionMatcher::ComputeConfidence(const GridCell& cell) const {
  // 综合权重本身已在 [0,1]（三种来源因子和=1.0）
  double c = cell.weight;
  if (c < 0.0) c = 0.0;
  if (c > 1.0) c = 1.0;
  return c;
}

PositionMatchResult PositionMatcher::Match(const DetectionEvent& event,
                                            const WeightedGrid& grid) const {
  PositionMatchResult result;
  result.event_id = event.event_id;
  result.timestamp_ns = event.timestamp_ns;

  // 红线：告警未带有效位姿（如红外/声纹只给类别不给位置）时不做位置匹配，
  //       直接保持"未定位"，避免用默认零位姿匹配出假物理监测点。
  if (!event.valid) {
    result.coordinate_source = "unlocalized";
    result.confidence = 0.0;
    result.localized = false;
    return result;
  }

  const GridCell* best = nullptr;
  if (!FindBestCell(event, grid, &best) || best == nullptr) {
    // 红线：无法对应时保持"未定位"
    result.coordinate_source = "unlocalized";
    result.confidence = 0.0;
    result.localized = false;
    return result;
  }

  const double confidence = ComputeConfidence(*best);
  if (confidence < kMinConfidence) {
    // 置信度过低，不输出物理监测点
    result.coordinate_source = "unlocalized";
    result.confidence = confidence;  // 保留实际置信度供上层决策
    result.localized = false;
    return result;
  }

  // 最佳位置 = 栅格中心（厘米级精度由 5cm 分辨率保证）
  result.matched_x = grid.origin_x + (best->index_x + 0.5) * grid.resolution_m;
  result.matched_y = grid.origin_y + (best->index_y + 0.5) * grid.resolution_m;
  result.matched_z = event.detection_z;  // 高度沿用检测事件
  result.bim_id = best->bim_id;
  result.slam_id.clear();  // 当前简化：slam 标识可由 Python 侧补充
  result.confidence = confidence;
  result.localized = true;

  // 坐标来源：按主导权重确定
  if (best->bim_weight >= best->slam_weight &&
      best->bim_weight >= best->cloud_weight && !best->bim_id.empty()) {
    result.coordinate_source = "bim";
  } else if (best->slam_weight >= best->cloud_weight) {
    result.coordinate_source = "slam";
  } else {
    result.coordinate_source = "cloud";
  }

  return result;
}

std::vector<PositionMatchResult> PositionMatcher::MatchAll(
    const std::vector<DetectionEvent>& events,
    const WeightedGrid& grid) const {
  std::vector<PositionMatchResult> results;
  results.reserve(events.size());
  for (const auto& e : events) {
    results.push_back(Match(e, grid));
  }
  return results;
}

}  // namespace tech_1_7
}  // namespace inspection_execution
