#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "inspection_execution_cpp/tech_1_2/task_lifecycle_executor.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inspection_execution::tech_1_2::TaskLifecycleExecutor>());
  rclcpp::shutdown();
  return 0;
}
