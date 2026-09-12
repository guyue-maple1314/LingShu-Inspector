#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"

namespace BT {
class Tree;
}

namespace inspection_execution {
namespace tech_1_1 {

// BehaviorTree.CPP 执行器：注册自定义节点、加载 XML 树、驱动 tick。
class BtTaskExecutor {
 public:
  BtTaskExecutor();
  ~BtTaskExecutor();

  bool StartFromFile(const std::string& xml_path, const GoalSafetyInput& goal);
  BT::NodeStatus Tick();
  double Progress() const;
  bool IsRunning() const;

 private:
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  double progress_{0.0};
  bool running_{false};
};

}  // namespace tech_1_1
}  // namespace inspection_execution
