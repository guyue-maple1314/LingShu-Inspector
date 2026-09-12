#include "inspection_execution_cpp/tech_1_1/behavior_tree_nodes.hpp"

#include <string>

namespace inspection_execution {
namespace tech_1_1 {

ValidateGoalNode::ValidateGoalNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList ValidateGoalNode::providedPorts() {
  return {
      BT::InputPort<std::string>("goal_id"),
      BT::InputPort<std::string>("task_type"),
      BT::InputPort<double>("valid_until_sec"),
      BT::InputPort<double>("now_sec"),
      BT::InputPort<bool>("robot_ready"),
      BT::OutputPort<bool>("goal_valid"),
  };
}

BT::NodeStatus ValidateGoalNode::tick() {
  GoalSafetyInput input;
  input.goal_id = getInput<std::string>("goal_id").value();
  input.task_type = getInput<std::string>("task_type").value();
  input.valid_until_sec = getInput<double>("valid_until_sec").value();
  input.now_sec = getInput<double>("now_sec").value();
  input.robot_ready = getInput<bool>("robot_ready").value();

  const GoalSafetyResult result = validator_.Validate(input);
  setOutput("goal_valid", result.allowed);
  return result.allowed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ExecuteTaskNode::ExecuteTaskNode(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList ExecuteTaskNode::providedPorts() {
  return {
      BT::InputPort<std::string>("goal_id"),
      BT::OutputPort<double>("progress"),
      BT::OutputPort<std::string>("current_state"),
  };
}

BT::NodeStatus ExecuteTaskNode::onStart() {
  progress_ = 0.0;
  setOutput("progress", progress_);
  setOutput("current_state", std::string("running"));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteTaskNode::onRunning() {
  progress_ += 0.05;
  if (progress_ >= 1.0) {
    progress_ = 1.0;
    setOutput("progress", progress_);
    setOutput("current_state", std::string("completed"));
    return BT::NodeStatus::SUCCESS;
  }
  setOutput("progress", progress_);
  setOutput("current_state", std::string("running"));
  return BT::NodeStatus::RUNNING;
}

void ExecuteTaskNode::onHalted() {
  progress_ = 0.0;
}

}  // namespace tech_1_1
}  // namespace inspection_execution
