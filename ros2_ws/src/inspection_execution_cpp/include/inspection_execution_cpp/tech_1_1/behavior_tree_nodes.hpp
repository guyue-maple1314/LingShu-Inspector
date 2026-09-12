#pragma once

#include "behaviortree_cpp/action_node.h"

#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"

namespace inspection_execution {
namespace tech_1_1 {

// 目标安全校验节点：校验通过返回 SUCCESS，否则 FAILURE。
class ValidateGoalNode : public BT::SyncActionNode {
 public:
  ValidateGoalNode(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

 private:
  GoalSafetyValidator validator_;
};

// 任务执行节点：模拟推进进度，RUNNING 直到完成。
class ExecuteTaskNode : public BT::StatefulActionNode {
 public:
  ExecuteTaskNode(const std::string& name, const BT::NodeConfig& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  double progress_{0.0};
};

}  // namespace tech_1_1
}  // namespace inspection_execution
