#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "inspection_execution_cpp/tech_1_6/multi_sensor_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_6/kinematic_constraint_builder.hpp"

namespace inspection_execution {
namespace tech_1_6 {

// 融合位姿输出
struct FusionPoseOutput {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double qw{1.0};  // 四元数
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  std::array<double, 6> covariance{};  // x,y,z,roll,pitch,yaw
  std::vector<std::string> valid_sources;  // 有效数据源
  std::string localization_state{"unknown"};
  std::uint64_t timestamp_ns{0};
  bool valid{false};
};

// 求解输入
struct LocalizationSolveInput {
  SyncedSensorPacket sensors;
  ConstraintSet constraints;
  FusionPoseOutput prev_pose;  // 上一帧位姿（预测用）
  std::uint64_t timestamp_ns{0};
};

/// 抽象紧耦合定位后端接口
/// 融合/SLAM 库未批准，与 AbstractMpcSolver 同理：不锁具体库
class AbstractLocalizerBackend {
 public:
  virtual ~AbstractLocalizerBackend() = default;
  virtual bool Initialize() = 0;
  virtual bool IsReady() const = 0;
  virtual bool Solve(const LocalizationSolveInput& input,
                     FusionPoseOutput* output) = 0;
};

/// 紧耦合定位器
/// 管理后端生命周期，融合视觉—激光—惯导—运动学
/// 激光失效时自动降级，不伪造激光有效状态
class TightlyCoupledLocalizer {
 public:
  explicit TightlyCoupledLocalizer(
      std::shared_ptr<AbstractLocalizerBackend> backend);

  /// 尝试定位
  bool Localize(const LocalizationSolveInput& input,
                FusionPoseOutput* output);

  /// 最近一次位姿
  const FusionPoseOutput& LatestPose() const;

  /// 当前有效数据源列表
  const std::vector<std::string>& ValidSources() const;

  /// 定位状态：跟踪/降级/失效
  const std::string& LocalizationState() const;

 private:
  std::shared_ptr<AbstractLocalizerBackend> backend_;
  FusionPoseOutput latest_pose_{};
  std::vector<std::string> valid_sources_{};
  std::string localization_state_{"idle"};
};

/// 测试用假后端（确定性位姿输出，不虚构 SLAM 结果）
/// 仅在后端已 Initialize 且传感器数据非空时返回有效位姿
class FakeLocalizerBackend : public AbstractLocalizerBackend {
 public:
  bool Initialize() override { initialized_ = true; return true; }
  bool IsReady() const override { return initialized_; }
  bool Solve(const LocalizationSolveInput& input,
             FusionPoseOutput* output) override;

 private:
  bool initialized_{false};
};

}  // namespace tech_1_6
}  // namespace inspection_execution
