#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"
#include "inspection_execution_cpp/tech_1_5/foot_contact_estimator.hpp"
#include "inspection_execution_cpp/tech_1_5/vibration_estimator.hpp"

namespace inspection_execution {
namespace tech_1_5 {

// MPC 修正输出
struct MpcCorrection {
  std::array<double, 12> torque_corrections{};   // 12 关节扭矩修正量
  std::array<double, 12> position_offsets{};      // 12 关节位置偏移量
  double damping_factor{0.0};                      // 阻尼系数 (0..1)
  bool active{false};                              // 是否激活修正
  std::uint64_t timestamp_ns{0};
};

// MPC 求解输入
struct MpcSolveInput {
  VibrationState vibration;
  ContactEstimate contact;
  RobotStateRaw robot_state;
  double target_speed{0.0};
  std::uint64_t timestamp_ns{0};
};

/// 抽象 MPC 求解器接口（不锁具体库，与 1.3 AbstractTrainer 同理）
class AbstractMpcSolver {
 public:
  virtual ~AbstractMpcSolver() = default;
  virtual bool Initialize() = 0;
  virtual bool IsReady() const = 0;
  virtual bool Solve(const MpcSolveInput& input, MpcCorrection* output) = 0;
};

/// MPC 振动抑制控制器
/// 管理 solver 生命周期，根据振动严重等级决定是否激活修正
class MpcVibrationController {
 public:
  explicit MpcVibrationController(std::shared_ptr<AbstractMpcSolver> solver);

  /// 尝试求解修正量（高频调用）
  bool ComputeCorrection(const MpcSolveInput& input, MpcCorrection* output);

  /// 是否激活（振动 >= Mild 时激活）
  bool IsActive() const;

  /// 最新修正量
  const MpcCorrection& LatestCorrection() const;

  /// 设置激活阈值
  void SetActivationSeverity(VibrationSeverity threshold);

 private:
  std::shared_ptr<AbstractMpcSolver> solver_;
  MpcCorrection latest_correction_{};
  VibrationSeverity activation_threshold_{VibrationSeverity::kMild};
};

/// 测试用假 MPC 求解器（不虚构推理结果，返回确定性阻尼修正）
/// 仅在 solver 已 Initialize 且振动非 None 时返回有效修正
class FakeMpcSolver : public AbstractMpcSolver {
 public:
  bool Initialize() override { initialized_ = true; return true; }
  bool IsReady() const override { return initialized_; }
  bool Solve(const MpcSolveInput& input, MpcCorrection* output) override;

 private:
  bool initialized_{false};
};

/// 将 MPC 修正叠加到关节命令上（安全校验：不超过限幅）
JointCommand ApplyCorrection(const JointCommand& base_cmd,
                             const MpcCorrection& correction,
                             double max_torque_correction = 5.0,
                             double max_position_offset = 0.02);

// 12 关节常量
inline constexpr std::size_t kTotalJoints = 12;

}  // namespace tech_1_5
}  // namespace inspection_execution
