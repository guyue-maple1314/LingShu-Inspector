#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_7/weighted_grid_builder.hpp"

namespace inspection_execution {
namespace tech_1_7 {

// 检测事件（来自 1.1/1.8/1.9 的 InspectionAlert，简化结构）
struct DetectionEvent {
  std::string event_id;
  std::string alert_type;          // 告警类型
  std::string source_device;       // 来源设备
  double detected_value{0.0};
  double detection_x{0.0};         // 检测位姿（地图系）
  double detection_y{0.0};
  double detection_z{0.0};
  std::uint64_t timestamp_ns{0};
  bool valid{false};
};

// 位置匹配结果
struct PositionMatchResult {
  std::string event_id;
  double matched_x{0.0};            // 最佳位置估算
  double matched_y{0.0};
  double matched_z{0.0};
  std::string bim_id;               // 匹配到的 BIM 构件（可空）
  std::string slam_id;              // 匹配到的 SLAM 标识（可空）
  std::string coordinate_source;   // "bim" / "slam" / "cloud" / "unlocalized"
  double confidence{0.0};          // [0,1]
  bool localized{false};            // 是否成功定位（红线：未定位时为 false）
  std::uint64_t timestamp_ns{0};
};

/// 位置匹配器（技术 1.7）
///
/// 在带权重的栅格地图上为检测事件计算最佳位置估算。
///
/// 匹配策略：
///   1. 以检测事件位置为中心，在半径 search_radius_m 内查找权重最高的栅格
///   2. 候选位置：BIM 构件中心 + SLAM 标识位置 + 点云聚类中心
///   3. 最佳位置 = 权重加权质心
///   4. 置信度 = 候选最高权重（归一化）
///
/// 红线：
///   - 位置结果必须包含 confidence + coordinate_source
///   - 无法对应时保持"未定位"（localized=false, coordinate_source="unlocalized"）
///   - 不输出虚假物理监测点
class PositionMatcher {
 public:
  static constexpr double kDefaultSearchRadiusM = 0.30;  // 30cm 搜索半径
  static constexpr double kMinConfidence = 0.3;          // 低于此阈值判为未定位

  explicit PositionMatcher(double search_radius_m = kDefaultSearchRadiusM);

  /// 单事件匹配
  /// grid 无效或 confidence < kMinConfidence 时返回未定位
  PositionMatchResult Match(const DetectionEvent& event,
                            const WeightedGrid& grid) const;

  /// 批量匹配
  std::vector<PositionMatchResult> MatchAll(
      const std::vector<DetectionEvent>& events,
      const WeightedGrid& grid) const;

  /// 搜索半径
  double SearchRadius() const;

 private:
  double search_radius_m_{kDefaultSearchRadiusM};

  // 在栅格中查找事件位置附近权重最高的单元
  bool FindBestCell(const DetectionEvent& event, const WeightedGrid& grid,
                    const GridCell** best_cell) const;

  // 计算匹配置信度
  double ComputeConfidence(const GridCell& cell) const;
};

}  // namespace tech_1_7
}  // namespace inspection_execution
