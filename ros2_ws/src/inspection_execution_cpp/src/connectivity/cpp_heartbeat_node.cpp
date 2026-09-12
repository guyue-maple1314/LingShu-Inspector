#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_interfaces/msg/robot_state.hpp"
#include "inspection_interfaces/msg/operator_instruction.hpp"

namespace inspection_execution {

class CppHeartbeatNode : public rclcpp::Node {
 public:
  CppHeartbeatNode()
      : Node(node_names::kCppHeartbeatNode) {
    const std::string robot_state_topic =
        declare_parameter<std::string>("robot_state_topic", topic_names::kRobotState);
    const std::string instruction_topic =
        declare_parameter<std::string>("operator_instruction_topic", topic_names::kOperatorInstruction);

    publisher_ = create_publisher<inspection_interfaces::msg::RobotState>(robot_state_topic, 10);
    subscription_ = create_subscription<inspection_interfaces::msg::OperatorInstruction>(
        instruction_topic, 10,
        [this](const inspection_interfaces::msg::OperatorInstruction::SharedPtr msg) {
          RCLCPP_INFO(get_logger(), "Received instruction [%s]: %s",
                      msg->source.c_str(), msg->raw_content.c_str());
        });

    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { PublishState(); });
    // 引用著作权常量，使其编入二进制（strings 可验证），并在启动日志输出
    RCLCPP_INFO(get_logger(), "%s", ownership::kStartupBanner);
    RCLCPP_INFO(get_logger(), "%s", ownership::kCopyrightEn);
    RCLCPP_INFO(get_logger(), "%s", ownership::kBuildOwnershipMark);
    RCLCPP_INFO(get_logger(), "C++ heartbeat node ready. [%s]", ownership::kTeamMark);
  }

 private:
  void PublishState() {
    inspection_interfaces::msg::RobotState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.battery_level = 80.0;
    msg.current_gait = "standing";
    msg.execution_state = "idle";
    publisher_->publish(msg);
  }

  rclcpp::Publisher<inspection_interfaces::msg::RobotState>::SharedPtr publisher_;
  rclcpp::Subscription<inspection_interfaces::msg::OperatorInstruction>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_execution::CppHeartbeatNode>());
  rclcpp::shutdown();
  return 0;
}
