#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/tech_1_1/abnormal_switch_monitor.hpp"
#include "inspection_execution_cpp/tech_1_1/bt_task_executor.hpp"
#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"
#include "inspection_interfaces/msg/structured_goal.hpp"
#include "inspection_interfaces/msg/task.hpp"
#include "inspection_interfaces/srv/validate_goal.hpp"

namespace inspection_execution {
namespace tech_1_1 {

class Tech11Node : public rclcpp::Node {
 public:
  using ValidateGoal = inspection_interfaces::srv::ValidateGoal;

  Tech11Node()
      : Node(node_names::kBtTaskExecutorNode) {
    declare_parameter("behavior_tree_xml", std::string("main_inspection_tree.xml"));

    subscriber_ = create_subscription<inspection_interfaces::msg::StructuredGoal>(
        topic_names::kStructuredGoal, 10,
        [this](const inspection_interfaces::msg::StructuredGoal::SharedPtr msg) {
          OnGoal(*msg);
        });
    publisher_ = create_publisher<inspection_interfaces::msg::Task>(topic_names::kTask, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { Tick(); });

    validate_service_ = create_service<ValidateGoal>(
        topic_names::kValidateGoalService,
        [this](const std::shared_ptr<ValidateGoal::Request> request,
               std::shared_ptr<ValidateGoal::Response> response) {
          HandleValidateGoal(request, response);
        });

    RCLCPP_INFO(get_logger(), "tech_1_1 executor node ready (BehaviorTree.CPP)");
  }

 private:
  GoalSafetyInput ToInput(const inspection_interfaces::msg::StructuredGoal& msg) const {
    GoalSafetyInput input;
    input.goal_id = msg.goal_id;
    input.task_type = msg.task_type;
    input.valid_until_sec = static_cast<double>(msg.valid_until.sec) +
                            static_cast<double>(msg.valid_until.nanosec) * 1e-9;
    input.now_sec = static_cast<double>(now().seconds()) +
                    static_cast<double>(now().nanoseconds()) * 1e-9;
    input.robot_ready = true;
    input.constraints = msg.constraints;
    return input;
  }

  void OnGoal(const inspection_interfaces::msg::StructuredGoal& msg) {
    const bool is_abnormal = msg.task_type == "abnormal";
    if (is_abnormal) {
      abnormal_monitor_.MarkArrival(std::chrono::steady_clock::now());
    }
    const std::string xml_path = get_parameter("behavior_tree_xml").as_string();
    if (!executor_.StartFromFile(xml_path, ToInput(msg))) {
      RCLCPP_WARN(get_logger(), "failed to start behavior tree for %s", msg.goal_id.c_str());
      return;
    }
    if (is_abnormal) {
      abnormal_monitor_.MarkSwitchComplete(std::chrono::steady_clock::now());
      RCLCPP_INFO(get_logger(), "abnormal switch latency=%.2fms",
                  abnormal_monitor_.LastLatencyMs());
    }
    RCLCPP_INFO(get_logger(), "started behavior tree for %s", msg.goal_id.c_str());
  }

  void Tick() {
    if (!executor_.IsRunning()) {
      return;
    }
    const BT::NodeStatus status = executor_.Tick();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(get_logger(), "behavior tree succeeded (progress=%.2f)", executor_.Progress());
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(get_logger(), "behavior tree failed");
    }
  }

  void HandleValidateGoal(
      const std::shared_ptr<ValidateGoal::Request> request,
      std::shared_ptr<ValidateGoal::Response> response) {
    const GoalSafetyResult result = validator_.Validate(ToInput(request->goal));
    response->allowed = result.allowed;
    response->rejection_reason = result.reason;
  }

  rclcpp::Subscription<inspection_interfaces::msg::StructuredGoal>::SharedPtr subscriber_;
  rclcpp::Publisher<inspection_interfaces::msg::Task>::SharedPtr publisher_;
  rclcpp::Service<ValidateGoal>::SharedPtr validate_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  GoalSafetyValidator validator_;
  BtTaskExecutor executor_;
  AbnormalSwitchMonitor abnormal_monitor_;
};

}  // namespace tech_1_1
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_execution::tech_1_1::Tech11Node>());
  rclcpp::shutdown();
  return 0;
}
