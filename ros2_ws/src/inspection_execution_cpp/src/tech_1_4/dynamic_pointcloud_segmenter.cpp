#include "inspection_execution_cpp/tech_1_4/dynamic_pointcloud_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace inspection_execution {
namespace tech_1_4 {

DynamicPointcloudSegmenter::DynamicPointcloudSegmenter(double min_robot_width)
    : min_robot_width_(min_robot_width) {}

CorridorSegmentation DynamicPointcloudSegmenter::Segment(
    const std::vector<Point2D>& points) const {
  CorridorSegmentation result;
  double left = -std::numeric_limits<double>::infinity();
  double right = std::numeric_limits<double>::infinity();

  for (const auto& p : points) {
    if (p.x < 0.0) {
      left = std::max(left, p.x);
    } else if (p.x > 0.0) {
      right = std::min(right, p.x);
    }
  }

  result.left_x = left;
  result.right_x = right;
  result.width = right - left;
  result.center_x = (left + right) / 2.0;
  result.valid = std::isfinite(left) && std::isfinite(right) &&
                 result.width >= min_robot_width_;
  return result;
}

}  // namespace tech_1_4
}  // namespace inspection_execution
