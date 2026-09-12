#include <algorithm>
#include <cmath>

#include "inspection_execution_cpp/tech_1_7/weighted_grid_builder.hpp"

namespace inspection_execution {
namespace tech_1_7 {

WeightedGridBuilder::WeightedGridBuilder(double resolution_m)
    : resolution_m_(resolution_m > 0.01 ? resolution_m : kDefaultResolutionM) {}

void WeightedGridBuilder::SetMapExtent(double origin_x, double origin_y,
                                       int size_x, int size_y) {
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  size_x_ = size_x > 0 ? size_x : 0;
  size_y_ = size_y > 0 ? size_y : 0;
}

void WeightedGridBuilder::LoadBim(const std::vector<BimElement>& bim_elements) {
  bim_elements_ = bim_elements;
}

void WeightedGridBuilder::LoadSlam(const SlamObservation& slam) {
  slam_ = slam;
}

void WeightedGridBuilder::LoadCloud(const TransformedCloud& cloud) {
  cloud_ = cloud;
}

std::size_t WeightedGridBuilder::BimCount() const {
  return bim_elements_.size();
}

void WeightedGridBuilder::ToIndex(double x, double y, int* ix, int* iy) const {
  *ix = static_cast<int>(std::floor((x - origin_x_) / resolution_m_));
  *iy = static_cast<int>(std::floor((y - origin_y_) / resolution_m_));
}

WeightedGrid WeightedGridBuilder::Build(std::uint64_t timestamp_ns) const {
  WeightedGrid grid;
  grid.origin_x = origin_x_;
  grid.origin_y = origin_y_;
  grid.resolution_m = resolution_m_;
  grid.size_x = size_x_;
  grid.size_y = size_y_;
  grid.timestamp_ns = timestamp_ns;

  // 无尺寸或无任何数据源时返回无效（不虚构）
  const bool has_any =
      !bim_elements_.empty() || !slam_.landmarks.empty() ||
      (cloud_.valid && !cloud_.points.empty());
  if (size_x_ <= 0 || size_y_ <= 0 || !has_any) {
    grid.valid = false;
    return grid;
  }

  grid.cells.assign(static_cast<std::size_t>(size_x_) * size_y_, GridCell{});

  // 1. 写入 BIM 先验权重
  for (const auto& bim : bim_elements_) {
    int ix = 0, iy = 0;
    ToIndex(bim.center_x, bim.center_y, &ix, &iy);
    if (ix < 0 || ix >= size_x_ || iy < 0 || iy >= size_y_) continue;
    auto& cell = grid.cells[static_cast<std::size_t>(iy) * size_x_ + ix];
    cell.bim_weight = bim.prior_weight;
    cell.bim_id = bim.id;
    cell.has_measurement = true;
  }

  // 2. 写入 SLAM 实时权重
  for (const auto& lm : slam_.landmarks) {
    int ix = 0, iy = 0;
    ToIndex(lm.x, lm.y, &ix, &iy);
    if (ix < 0 || ix >= size_x_ || iy < 0 || iy >= size_y_) continue;
    auto& cell = grid.cells[static_cast<std::size_t>(iy) * size_x_ + ix];
    cell.slam_weight = slam_.confidence;
    cell.has_measurement = true;
  }

  // 3. 写入点云实测权重（密度归一化）
  if (cloud_.valid && !cloud_.points.empty()) {
    // 统计每个栅格的点云命中数
    std::vector<int> hits(grid.cells.size(), 0);
    int max_hits = 0;
    for (const auto& p : cloud_.points) {
      int ix = 0, iy = 0;
      ToIndex(p.x, p.y, &ix, &iy);
      if (ix < 0 || ix >= size_x_ || iy < 0 || iy >= size_y_) continue;
      int& h = hits[static_cast<std::size_t>(iy) * size_x_ + ix];
      ++h;
      if (h > max_hits) max_hits = h;
    }
    if (max_hits > 0) {
      for (std::size_t i = 0; i < grid.cells.size(); ++i) {
        if (hits[i] > 0) {
          grid.cells[i].cloud_weight =
              static_cast<double>(hits[i]) / static_cast<double>(max_hits);
          grid.cells[i].has_measurement = true;
        }
      }
    }
  }

  // 4. 填充栅格索引 + 计算综合权重（加权和，三种来源都缺失时保持 0）
  for (int iy = 0; iy < size_y_; ++iy) {
    for (int ix = 0; ix < size_x_; ++ix) {
      auto& cell = grid.cells[static_cast<std::size_t>(iy) * size_x_ + ix];
      cell.index_x = ix;
      cell.index_y = iy;
      cell.weight = cell.bim_weight * kBimWeightFactor +
                    cell.slam_weight * kSlamWeightFactor +
                    cell.cloud_weight * kCloudWeightFactor;
    }
  }

  grid.valid = true;
  return grid;
}

}  // namespace tech_1_7
}  // namespace inspection_execution
