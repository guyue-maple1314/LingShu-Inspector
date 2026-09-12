#include "inspection_execution_cpp/tech_1_1/bt_task_executor.hpp"

#include "inspection_execution_cpp/tech_1_1/behavior_tree_nodes.hpp"

namespace inspection_execution {
namespace tech_1_1 {

BtTaskExecutor::BtTaskExecutor() {
  factory_.registerNodeType<ValidateGoalNode>("ValidateGoal");
  factory_.registerNodeType<ExecuteTaskNode>("ExecuteTask");
}

BtTaskExecutor::~BtTaskExecutor() = default;

bool BtTaskExecutor::StartFromFile(const std::string& xml_path, const GoalSafetyInput& goal) {
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(xml_path));
  auto bb = tree_->rootBlackboard();
  bb->set<std::string>("goal_id", goal.goal_id);
  bb->set<std::string>("task_type", goal.task_type);
  bb->set<double>("valid_until_sec", goal.valid_until_sec);
  bb->set<double>("now_sec", goal.now_sec);
  bb->set<bool>("robot_ready", goal.robot_ready);
  progress_ = 0.0;
  running_ = true;
  return true;
}

BT::NodeStatus BtTaskExecutor::Tick() {
  if (!tree_) {
    return BT::NodeStatus::FAILURE;
  }
  const BT::NodeStatus status = tree_->tickOnce();
  double progress = 0.0;
  if (tree_->rootBlackboard()->get<double>("progress", progress)) {
    progress_ = progress;
  }
  if (status != BT::NodeStatus::RUNNING) {
    running_ = false;
  }
  return status;
}

double BtTaskExecutor::Progress() const {
  return progress_;
}

bool BtTaskExecutor::IsRunning() const {
  return running_;
}

}  // namespace tech_1_1
}  // namespace inspection_execution
