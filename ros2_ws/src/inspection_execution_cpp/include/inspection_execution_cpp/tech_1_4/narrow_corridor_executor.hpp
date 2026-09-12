#pragma once

namespace inspection_execution {
namespace tech_1_4 {

enum class CorridorExecutionState { kIdle, kTracking, kCompleted, kBlocked, kFailed };

// 跟踪规划路径并做闭环安全修正：通道变窄或视觉失效时停止。
class NarrowCorridorExecutor {
 public:
  explicit NarrowCorridorExecutor(double safety_margin = 0.10);
  bool Start(double robot_width, double corridor_width);
  bool Update(double corridor_width, bool visual_valid);
  void Complete();
  CorridorExecutionState State() const;
  double Progress() const;

 private:
  double safety_margin_;
  double robot_width_{0.0};
  double progress_{0.0};
  CorridorExecutionState state_{CorridorExecutionState::kIdle};
};

}  // namespace tech_1_4
}  // namespace inspection_execution
