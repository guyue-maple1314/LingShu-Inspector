#pragma once

#include <vector>

namespace inspection_execution {
namespace tech_1_4 {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct CorridorSegmentation {
  bool valid{false};
  double width{0.0};
  double center_x{0.0};
  double left_x{0.0};
  double right_x{0.0};
};

// 从点云提取最窄通道：以机器人中心 x=0 为参考，找左右最近障碍。
class DynamicPointcloudSegmenter {
 public:
  explicit DynamicPointcloudSegmenter(double min_robot_width = 0.46);
  CorridorSegmentation Segment(const std::vector<Point2D>& points) const;

 private:
  double min_robot_width_;
};

}  // namespace tech_1_4
}  // namespace inspection_execution
