#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/tech_1_4/corridor_fusion.hpp"
#include "inspection_execution_cpp/tech_1_4/dynamic_pointcloud_segmenter.hpp"
#include "inspection_execution_cpp/tech_1_4/narrow_corridor_executor.hpp"
#include "inspection_execution_cpp/tech_1_4/visual_texture_validator.hpp"
#include "inspection_interfaces/msg/corridor_state.hpp"

namespace inspection_execution {
namespace tech_1_4 {

class Tech14Node : public rclcpp::Node {
 public:
  Tech14Node()
      : Node(node_names::kTech1_4Node) {
    this->declare_parameter<double>("robot_width", 0.46);
    this->declare_parameter<double>("safety_margin", 0.10);
    this->declare_parameter<double>("min_visual_confidence", 0.6);

    publisher_ = this->create_publisher<inspection_interfaces::msg::CorridorState>(
        topic_names::kCorridorState, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { Tick(); });
    RCLCPP_INFO(this->get_logger(), "%s started (示例点云输入)",
                node_names::kTech1_4Node);
  }

 private:
  geometry_msgs::msg::Point MakePoint(double x, double y) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    return p;
  }

  void Tick() {
    // 示例输入：真实激光/视觉接入后替换为订阅回调。
    const std::vector<Point2D> points = {{-0.25, 0.0}, {0.25, 0.0}};
    const double texture_confidence = 0.9;

    const auto seg = segmenter_.Segment(points);
    const auto vis = visual_.Validate(texture_confidence);
    const auto fused = fusion_.Fuse(seg, vis);

    auto msg = inspection_interfaces::msg::CorridorState();
    msg.header.stamp = this->now();
    msg.corridor_width = static_cast<float>(fused.width);
    msg.visual_validation_passed = fused.visual_validation_passed;
    msg.confidence = fused.confidence;
    msg.centerline = {MakePoint(fused.center_x, 0.0)};
    msg.left_boundary = {MakePoint(fused.left_x, 0.0)};
    msg.right_boundary = {MakePoint(fused.right_x, 0.0)};
    publisher_->publish(msg);

    const double robot_width = this->get_parameter("robot_width").as_double();
    if (fused.valid) {
      if (executor_.State() == CorridorExecutionState::kIdle ||
          executor_.State() == CorridorExecutionState::kFailed ||
          executor_.State() == CorridorExecutionState::kBlocked) {
        executor_.Start(robot_width, fused.width);
      }
      executor_.Update(fused.width, fused.visual_validation_passed);
    }
  }

  DynamicPointcloudSegmenter segmenter_;
  VisualTextureValidator visual_;
  CorridorFusion fusion_;
  NarrowCorridorExecutor executor_;
  rclcpp::Publisher<inspection_interfaces::msg::CorridorState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tech_1_4
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_execution::tech_1_4::Tech14Node>());
  rclcpp::shutdown();
  return 0;
}
