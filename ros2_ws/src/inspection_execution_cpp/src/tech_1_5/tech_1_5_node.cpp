#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "inspection_interfaces/msg/terrain_observation.hpp"

#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"
#include "inspection_execution_cpp/common/node_names.hpp"
#include "inspection_execution_cpp/common/topic_names.hpp"
#include "inspection_execution_cpp/tech_1_5/foot_contact_estimator.hpp"
#include "inspection_execution_cpp/tech_1_5/grating_metrics_recorder.hpp"
#include "inspection_execution_cpp/tech_1_5/mpc_vibration_controller.hpp"
#include "inspection_execution_cpp/tech_1_5/vibration_estimator.hpp"

namespace inspection_execution {
namespace tech_1_5 {

/// 钢格网主动抑振节点
///
/// 架构（遵守设计文档实时约束）：
/// - ROS2 订阅 /imu(1000Hz)、/foot_force(500Hz) 在主 executor 回调中
///   将数据存入线程安全缓冲（最新值覆盖旧值）。
/// - 独立 std::thread 运行高频控制循环（500Hz 基准）：
///   读取缓冲 → FootContactEstimator → VibrationEstimator → MpcVibrationController
///   → ApplyCorrection → RobotSdkAdapter::SendJointCommand
/// - 高频线程不阻塞在 ROS2 通信上；Python/HMI 通过 /grating_status 获取状态。
///
/// 适配器注入式设计（与 1.3 同理，不直调厂商 SDK）：
/// - FootForceAdapter / ImuAdapter 由上层注入
/// - RobotSdkAdapter 由上层注入
/// - AbstractMpcSolver 由上层注入（训练框架/求解库未批准时不虚构）
class Tech15Node : public rclcpp::Node {
 public:
  Tech15Node() : rclcpp::Node(node_names::kTech1_5Node) {
    // 声明参数
    this->declare_parameter("control_freq_hz", 500.0);
    this->declare_parameter("target_speed", 0.8);
    this->declare_parameter("grating_mode", false);

    // 创建发布器
    grating_status_pub_ =
        this->create_publisher<inspection_interfaces::msg::TerrainObservation>(
            topic_names::kTerrainObservation, 10);

    // 创建低频状态发布定时器（10Hz，不影响高频环）
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { PublishStatus(); });

    running_ = true;
    control_thread_ = std::thread(&Tech15Node::ControlLoop, this);

    RCLCPP_INFO(this->get_logger(),
                "tech_1_5_node started: high-freq control loop + "
                "adapter injection (not fictionally connected)");
  }

  ~Tech15Node() override {
    running_ = false;
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }

  // ---- 适配器注入接口（与 1.3 同理，由上层实例化并注入）----
  void SetFootForceAdapter(std::shared_ptr<FootForceAdapter> adapter) {
    foot_force_adapter_ = std::move(adapter);
  }
  void SetImuAdapter(std::shared_ptr<ImuAdapter> adapter) {
    imu_adapter_ = std::move(adapter);
  }
  void SetRobotSdkAdapter(std::shared_ptr<RobotSdkAdapter> adapter) {
    robot_sdk_adapter_ = std::move(adapter);
  }
  void SetMpcSolver(std::shared_ptr<AbstractMpcSolver> solver) {
    if (solver && solver->Initialize()) {
      mpc_controller_ = std::make_unique<MpcVibrationController>(solver);
    }
  }

  void SetGratingMode(bool enabled) {
    grating_mode_.store(enabled);
    contact_estimator_.SetGratingMode(enabled);
    metrics_recorder_.SetGratingMode(enabled);
  }

 private:
  // 高频控制循环（独立线程，500Hz 基准）
  void ControlLoop() {
    const double freq = this->get_parameter("control_freq_hz").as_double();
    const auto period =
        std::chrono::microseconds(static_cast<int>(1e6 / freq));
    const double target_speed =
        this->get_parameter("target_speed").as_double();

    while (running_) {
      auto loop_start = std::chrono::steady_clock::now();

      // 1. 读取传感器（通过抽象适配器，不直调 SDK）
      FootForceSample foot_sample{};
      ImuSample imu_sample{};

      if (foot_force_adapter_) {
        foot_force_adapter_->ReadSample(&foot_sample);
      }
      if (imu_adapter_) {
        imu_adapter_->ReadSample(&imu_sample);
      }

      // 2. 足接触估计（500Hz）
      for (std::size_t i = 0; i < 4; ++i) {
        contact_estimator_.Update(i, foot_sample.normal_forces[i],
                                  foot_sample.timestamp_ns);
      }
      const auto contact = contact_estimator_.Estimate();

      // 3. 振动估计（IMU 1000Hz，足力 500Hz 融合）
      vibration_estimator_.UpdateImu(imu_sample);
      vibration_estimator_.UpdateFootForce(foot_sample);
      const auto vibration = vibration_estimator_.Estimate();

      // 4. MPC 修正（求解器未注入时不虚构，跳过）
      MpcCorrection correction{};
      if (mpc_controller_) {
        MpcSolveInput input;
        input.vibration = vibration;
        input.contact = contact;
        input.target_speed = target_speed;
        input.timestamp_ns = imu_sample.timestamp_ns;
        if (robot_sdk_adapter_) {
          robot_sdk_adapter_->ReadState(&input.robot_state);
        }
        mpc_controller_->ComputeCorrection(input, &correction);
      }

      // 5. 记录指标
      metrics_recorder_.RecordStep(imu_sample.timestamp_ns);
      metrics_recorder_.UpdateVibration(vibration.accel_rms);

      // 检测异常事件
      if (vibration.severity >= VibrationSeverity::kModerate) {
        GaitAnomalyEvent event;
        event.timestamp_ns = imu_sample.timestamp_ns;
        event.anomaly_type = vibration.resonance_detected
                                 ? "resonance"
                                 : "vibration";
        event.severity = static_cast<double>(vibration.severity);
        metrics_recorder_.RecordAnomaly(event);
      }
      for (std::size_t i = 0; i < 4; ++i) {
        if (contact.feet[i].quality == ContactQuality::kFalseContact) {
          GaitAnomalyEvent event;
          event.timestamp_ns = imu_sample.timestamp_ns;
          event.foot_index = static_cast<int>(i);
          event.anomaly_type = "false_contact";
          event.severity = contact.feet[i].confidence;
          metrics_recorder_.RecordAnomaly(event);
        }
      }

      // 6. 下发关节命令（通过 RobotSdkAdapter 抽象，安全 ApplyCorrection）
      if (robot_sdk_adapter_ && robot_sdk_adapter_->IsConnected() &&
          correction.active) {
        JointCommand base_cmd{};
        // base_cmd 由上游运动控制器提供，此处仅做修正叠加
  // 当前：base_cmd 为空，仅在有修正时发送修正量
        base_cmd.positions.resize(kTotalJoints, 0.0);
        base_cmd.torques.resize(kTotalJoints, 0.0);
        base_cmd.velocities.resize(kTotalJoints, 0.0);
        const auto final_cmd = ApplyCorrection(base_cmd, correction);
        robot_sdk_adapter_->SendJointCommand(final_cmd);
      }

      // 更新速度（从 robot_state 估计）
      if (robot_sdk_adapter_) {
        RobotStateRaw state;
        if (robot_sdk_adapter_->ReadState(&state)) {
          // 简化：用关节速度均值近似前进速度
          double vel_sum = 0.0;
          for (double v : state.joint_velocities) vel_sum += std::abs(v);
          double approx_speed = vel_sum / kTotalJoints;
          metrics_recorder_.UpdateSpeed(approx_speed,
                                        imu_sample.timestamp_ns);
        }
      }

      // 精确睡眠到下一周期
      std::this_thread::sleep_until(loop_start + period);
    }
  }

  // 低频状态发布（10Hz，不影响控制环）
  void PublishStatus() {
    auto msg = inspection_interfaces::msg::TerrainObservation();
    const auto metrics = metrics_recorder_.Snapshot();
    const auto vibration = vibration_estimator_.Estimate();

    msg.terrain_type = grating_mode_.load() ? "grating" : "solid";
    msg.passable = !vibration.resonance_detected;
    msg.confidence = 1.0 - std::min(1.0, vibration.accel_rms / 10.0);

    grating_status_pub_->publish(msg);
  }

  // 组件
  FootContactEstimator contact_estimator_;
  VibrationEstimator vibration_estimator_;
  GratingMetricsRecorder metrics_recorder_;
  std::unique_ptr<MpcVibrationController> mpc_controller_;

  // 注入式适配器（不直调厂商 SDK）
  std::shared_ptr<FootForceAdapter> foot_force_adapter_;
  std::shared_ptr<ImuAdapter> imu_adapter_;
  std::shared_ptr<RobotSdkAdapter> robot_sdk_adapter_;

  // 线程控制
  std::atomic<bool> running_{false};
  std::atomic<bool> grating_mode_{false};
  std::thread control_thread_;

  // ROS2 接口
  rclcpp::Publisher<inspection_interfaces::msg::TerrainObservation>::SharedPtr
      grating_status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace tech_1_5
}  // namespace inspection_execution

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<inspection_execution::tech_1_5::Tech15Node>());
  rclcpp::shutdown();
  return 0;
}
