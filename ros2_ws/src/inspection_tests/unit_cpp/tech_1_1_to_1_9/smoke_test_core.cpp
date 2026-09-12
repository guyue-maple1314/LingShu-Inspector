#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "inspection_execution_cpp/tech_1_1/abnormal_switch_monitor.hpp"
#include "inspection_execution_cpp/tech_1_1/goal_safety_validator.hpp"
#include "inspection_execution_cpp/tech_1_1/ppo_runtime_controller.hpp"
#include "inspection_execution_cpp/tech_1_2/preemption_latency_monitor.hpp"
#include "inspection_execution_cpp/tech_1_2/task_progress_store.hpp"
#include "inspection_execution_cpp/tech_1_2/task_resume_executor.hpp"

// tech_1_3 组件
#include "inspection_execution_cpp/adapters/foot_force_adapter.hpp"
#include "inspection_execution_cpp/adapters/imu_adapter.hpp"
#include "inspection_execution_cpp/adapters/robot_sdk_adapter.hpp"
#include "inspection_execution_cpp/tech_1_3/locomotion_command_adapter.hpp"
#include "inspection_execution_cpp/tech_1_3/observation_builder.hpp"
#include "inspection_execution_cpp/tech_1_3/ppo_policy_loader.hpp"
#include "inspection_execution_cpp/tech_1_3/policy_output_validator.hpp"
#include "inspection_execution_cpp/tech_1_4/corridor_fusion.hpp"
#include "inspection_execution_cpp/tech_1_4/dynamic_pointcloud_segmenter.hpp"
#include "inspection_execution_cpp/tech_1_4/narrow_corridor_executor.hpp"
#include "inspection_execution_cpp/tech_1_4/visual_texture_validator.hpp"

// tech_1_5 组件
#include "inspection_execution_cpp/tech_1_5/foot_contact_estimator.hpp"
#include "inspection_execution_cpp/tech_1_5/grating_metrics_recorder.hpp"
#include "inspection_execution_cpp/tech_1_5/mpc_vibration_controller.hpp"
#include "inspection_execution_cpp/tech_1_5/vibration_estimator.hpp"

// tech_1_6 组件
#include "inspection_execution_cpp/tech_1_6/multi_sensor_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_6/kinematic_constraint_builder.hpp"
#include "inspection_execution_cpp/tech_1_6/tightly_coupled_localizer.hpp"
#include "inspection_execution_cpp/tech_1_6/localization_degradation_monitor.hpp"
#include "inspection_execution_cpp/tech_1_6/navigation_executor.hpp"

// tech_1_7 组件
#include "inspection_execution_cpp/tech_1_7/pointcloud_transformer.hpp"
#include "inspection_execution_cpp/tech_1_7/weighted_grid_builder.hpp"
#include "inspection_execution_cpp/tech_1_7/position_matcher.hpp"
#include "inspection_execution_cpp/tech_1_7/alarm_location_publisher.hpp"

// tech_1_8 组件
#include "inspection_execution_cpp/tech_1_8/thermal_visual_imu_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_8/thermal_image_stabilizer.hpp"
#include "inspection_execution_cpp/tech_1_8/emissivity_compensator.hpp"
#include "inspection_execution_cpp/tech_1_8/angle_distance_compensator.hpp"
#include "inspection_execution_cpp/tech_1_8/thermal_range_validator.hpp"
#include "inspection_execution_cpp/tech_1_9/directional_beamformer.hpp"
#include "inspection_execution_cpp/tech_1_9/microphone_synchronizer.hpp"
#include "inspection_execution_cpp/tech_1_9/snr_estimator.hpp"
#include "inspection_execution_cpp/tech_1_9/spatial_filter.hpp"

using namespace inspection_execution;

int main() {
  // 1. 目标安全校验
  tech_1_1::GoalSafetyValidator validator;
  tech_1_1::GoalSafetyInput input;
  input.goal_id = "g-1";
  input.task_type = "inspection";
  input.valid_until_sec = 100.0;
  input.now_sec = 0.0;
  input.robot_ready = true;
  assert(validator.Validate(input).allowed);

  input.task_type = "unknown";
  assert(!validator.Validate(input).allowed);
  input.task_type = "inspection";

  input.robot_ready = false;
  assert(!validator.Validate(input).allowed);

  // 2. 任务进度存储
  tech_1_2::TaskProgressStore store;
  tech_1_2::TaskProgress progress{0.5, "running"};
  store.Save("task-a", progress);
  assert(store.Has("task-a"));
  tech_1_2::TaskProgress loaded;
  assert(store.Load("task-a", &loaded));
  assert(loaded.progress == 0.5);
  store.Clear("task-a");
  assert(!store.Has("task-a"));

  // 2b. 恢复执行器
  store.Save("task-a", progress);
  tech_1_2::TaskResumeExecutor resume(store);
  assert(resume.CanResume("task-a"));
  tech_1_2::TaskProgress resume_point;
  assert(resume.LoadResumePoint("task-a", &resume_point));
  assert(resume_point.progress == 0.5);
  store.Clear("task-a");

  // 3. 耗时监控
  using Clock = std::chrono::steady_clock;
  tech_1_2::PreemptionLatencyMonitor preempt;
  const auto t0 = Clock::now();
  preempt.MarkArrival(t0);
  preempt.MarkSwitchComplete(t0 + std::chrono::milliseconds(300));
  assert(preempt.Count() == 1);
  assert(preempt.LastLatencyMs() > 250.0 && preempt.LastLatencyMs() < 350.0);

  tech_1_1::AbnormalSwitchMonitor abnormal;
  abnormal.MarkArrival(t0);
  abnormal.MarkSwitchComplete(t0 + std::chrono::milliseconds(100));
  assert(abnormal.MaxLatencyMs() > 50.0);

  // ========== tech_1_3：多场景运动智能迁移组件 ==========

  // 4. ObservationBuilder：默认 17 维观测，IMU 四元数→RPY + 四足接触/足力归一化
  {
    tech_1_3::ObservationBuilder builder;
    assert(builder.ExpectedDimension() == tech_1_3::kDefaultObservationDim);
    tech_1_3::ObservationInput in;
    in.imu.qw = 1.0;            // identity quat → rpy=0
    in.imu.qx = in.imu.qy = in.imu.qz = 0.0;
    in.imu.gx = 0.01; in.imu.gy = 0.02; in.imu.gz = 0.03;
    in.foot_force.normal_forces = {60.0, 55.0, 40.0, 50.0};  // 单位 N，阈值 50N
    in.foot_force.contact_flags = {1.0, 1.0, 0.0, 1.0};
    in.body_linear_vel = {0.8, 0.0, 0.0};  // PPT 1.5 0.8 m/s
    in.terrain_type = "grating";
    std::vector<float> obs;
    assert(builder.Build(in, &obs));
    assert(obs.size() == tech_1_3::kDefaultObservationDim);
    // rpy=0
    assert(std::abs(obs[0]) < 1e-5 && std::abs(obs[1]) < 1e-5 && std::abs(obs[2]) < 1e-5);
    // angular vel
    assert(std::abs(obs[3] - 0.01f) < 1e-5);
    // foot contact: 1,1,0,1
    assert(obs[9] == 1.0f && obs[10] == 1.0f && obs[11] == 0.0f && obs[12] == 1.0f);
    // foot force normalized by 50N → FL 60/50=1.2；FR 1.1；RL 0.8；RR 1.0
    assert(std::abs(obs[13] - 1.2f) < 1e-3);
    assert(std::abs(obs[14] - 1.1f) < 1e-3);
    assert(std::abs(obs[15] - 0.8f) < 1e-3);
    assert(std::abs(obs[16] - 1.0f) < 1e-3);
  }

  // 5. PolicyOutputValidator：维度 / 范围 / 时效 + Clamp
  {
    tech_1_3::PolicyOutputValidator v;
    v.SetExpectedDimension(tech_1_3::kDefaultActionDim);
    v.SetActionRangePerDim(1.0f);
    v.SetMaxAgeMs(100);
    // 非法维度
    std::vector<float> bad_dim{0.1f, 0.2f};
    assert(!v.Validate(bad_dim, 0, 0).success);
    // 超范围（默认 1.0）但未 Clamp → Validate 失败
    std::vector<float> out_of_range(12, 2.0f);
    assert(!v.Validate(out_of_range, 0, 0).success);
    // Clamp → 全变为 ±1.0
    assert(v.Clamp(&out_of_range) == ErrorCode::kOk);
    for (float x : out_of_range) assert(x == 1.0f);
    // Clamp 后 Validate 通过且不超时
    assert(v.Validate(out_of_range, 1000ULL, 1000ULL + 50ULL * 1000000ULL).success);
    // 超时（100+1ms > 100ms）
    assert(!v.Validate(out_of_range, 0, 101ULL * 1000000ULL).success);
  }

  // 6. PpoPolicyLoaderImpl：读取 Python policy_exporter 写出的 metadata.json，
  //    不虚构推理：IsLoaded=false，Infer=false（遵守红线）
  {
    // 在当前工作目录放一个临时 metadata 文件（避免 mkdir -p 在 Windows 上失效）
    const std::string meta_path = "./tech_1_3_policy_smoke_metadata.json";
    {
      std::ofstream f(meta_path, std::ios::trunc);
      assert(f.is_open());
      f << "{\n"
        << "  \"version\": \"1.3.0-smoke\",\n"
        << "  \"observation_dim\": 17,\n"
        << "  \"action_dim\": 12,\n"
        << "  \"policy_filename\": \"policy.onnx\"\n"
        << "}\n";
    }
    // Load 需要 policy_dir，里面包含 policy_metadata.json；
    // 简化：用当前目录（"."）并期望找到 "./policy_metadata.json"。
    // 所以把文件复制到标准名字：
    const std::string standard_meta = "./policy_metadata.json";
    {
      std::ofstream dst(standard_meta, std::ios::binary | std::ios::trunc);
      std::ifstream src(meta_path, std::ios::binary);
      assert(dst && src);
      dst << src.rdbuf();
    }
    tech_1_3::PpoPolicyLoaderImpl loader;
    assert(loader.Load("."));                 // 解析 metadata 成功
    assert(!loader.IsLoaded());               // 没有权重钩子实现，不虚构 loaded=true
    const auto& m = loader.Metadata();
    assert(m.version == "1.3.0-smoke");
    assert(m.observation_dim == 17);
    assert(m.action_dim == 12);
    // Infer 在未 loaded 时必须返回 false（不伪造 action）
    std::vector<float> obs(17, 0.0f);
    std::vector<float> act;
    assert(!loader.Infer(obs, &act));
    // 清理临时文件
    (void)std::remove(meta_path.c_str());
    (void)std::remove(standard_meta.c_str());
  }

  // 7. LocomotionCommandAdapter：通过 RobotSdkAdapter 抽象接口（不直调厂商 SDK）
  {
    struct FakeRobot : public RobotSdkAdapter {
      bool connected = false;
      int send_count = 0;
      JointCommand last_cmd;
      bool Connect() override { connected = true; return true; }
      void Disconnect() override { connected = false; }
      bool IsConnected() const override { return connected; }
      bool SendJointCommand(const JointCommand& c) override {
        last_cmd = c;
        ++send_count;
        return true;
      }
      bool ReadState(RobotStateRaw*) override { return false; }
    };
    auto fake = std::make_shared<FakeRobot>();
    fake->Connect();
    tech_1_3::LocomotionCommandAdapter adapter(fake);
    adapter.SetJointDeltaRangeRad(0.04);
    adapter.SetTargetField(tech_1_3::LocomotionCommandAdapter::TargetField::kPositions);

    // 维度错误 → ConvertOnly 失败，且不调用 Send
    std::vector<float> bad_dim{0.1f, 0.2f};
    assert(!adapter.ConvertAndSend(bad_dim).success);
    assert(fake->send_count == 0);

    // 正确 12 维 ±1 → positions 映射为 ±0.04
    std::vector<float> action(12, -1.0f);
    for (std::size_t i = 0; i < action.size(); ++i) action[i] = (i % 2 == 0) ? 1.0f : -1.0f;
    JointCommand out;
    assert(adapter.ConvertOnly(action, &out).success);
    assert(out.positions.size() == tech_1_3::kTotalJoints);
    for (std::size_t i = 0; i < out.positions.size(); ++i) {
      const double expect = (i % 2 == 0 ? 1.0 : -1.0) * 0.04;
      assert(std::abs(out.positions[i] - expect) < 1e-9);
    }
    // ConvertAndSend → Send 通过 FakeRobot
    assert(adapter.ConvertAndSend(action).success);
    assert(fake->send_count == 1);

    // 未连接时发送 → 失败（安全边界）
    fake->Disconnect();
    assert(!adapter.ConvertAndSend(action).success);
  }

  // 8. 极窄通道感知与执行（1.4）
  tech_1_4::DynamicPointcloudSegmenter segmenter(0.46);
  const std::vector<tech_1_4::Point2D> wide = {
      {-0.3, 0.0}, {-0.25, 1.0}, {0.25, 0.0}, {0.3, 2.0}};
  const auto seg = segmenter.Segment(wide);
  assert(seg.valid && seg.width > 0.46);
  const auto narrow = segmenter.Segment({{-0.2, 0.0}, {0.2, 0.0}});
  assert(!narrow.valid);

  tech_1_4::VisualTextureValidator visual(0.6);
  const auto vis_ok = visual.Validate(0.8);
  const auto vis_bad = visual.Validate(0.3);
  assert(vis_ok.passed && !vis_bad.passed);

  tech_1_4::CorridorFusion fusion;
  assert(fusion.Fuse(seg, vis_ok).valid);
  assert(!fusion.Fuse(seg, vis_bad).valid);

  tech_1_4::NarrowCorridorExecutor executor(0.10);
  assert(executor.Start(0.46, 0.75));
  assert(executor.State() == tech_1_4::CorridorExecutionState::kTracking);
  assert(!executor.Update(0.75, false));  // 视觉失效 -> 阻塞
  assert(executor.State() == tech_1_4::CorridorExecutionState::kBlocked);

  // ========== tech_1_5：钢格网主动抑振组件 ==========

  // 8. FootContactEstimator：500Hz 足力 → 接触状态分类
  {
    tech_1_5::FootContactEstimator estimator;
    for (std::size_t i = 0; i < tech_1_5::FootContactEstimator::kWindow; ++i) {
      estimator.Update(0, 45.0, i * 2000000ULL);
      estimator.Update(1, 10.0, i * 2000000ULL);
      estimator.Update(2, 5.0, i * 2000000ULL);
      estimator.Update(3, i % 2 == 0 ? 50.0 : 5.0, i * 2000000ULL);
    }
    auto contact = estimator.Estimate();
    assert(contact.feet[0].quality == tech_1_5::ContactQuality::kSolidContact);
    assert(contact.feet[1].quality == tech_1_5::ContactQuality::kGratingContact);
    assert(contact.feet[2].quality == tech_1_5::ContactQuality::kNoContact);
    assert(contact.feet[3].quality == tech_1_5::ContactQuality::kFalseContact);
  }

  // 9. VibrationEstimator：融合 IMU + 足力 → 振动状态
  {
    tech_1_5::VibrationEstimator estimator;
    for (std::size_t i = 0; i < tech_1_5::VibrationEstimator::kImuWindow; ++i) {
      ImuSample s;
      s.ax = 0.1; s.ay = 0.1; s.az = -9.81;
      s.timestamp_ns = i * 1000000ULL;
      estimator.UpdateImu(s);
    }
    FootForceSample ff{};
    ff.normal_forces = {40.0, 35.0, 30.0, 38.0};
    for (std::size_t i = 0; i < tech_1_5::VibrationEstimator::kForceWindow; ++i) {
      estimator.UpdateFootForce(ff);
    }
    auto vib = estimator.Estimate();
    assert(vib.severity == tech_1_5::VibrationSeverity::kNone);

    for (std::size_t i = 0; i < tech_1_5::VibrationEstimator::kImuWindow; ++i) {
      ImuSample s;
      s.ax = 6.0 * (i % 2 == 0 ? 1.0 : -1.0);  // 6 m/s² 振荡 → RMS=6.0 >= 5.0
      s.ay = 0.0; s.az = -9.81;
      s.timestamp_ns = (100 + i) * 1000000ULL;
      estimator.UpdateImu(s);
    }
    auto vib2 = estimator.Estimate();
    assert(vib2.severity == tech_1_5::VibrationSeverity::kSevere);
    assert(vib2.dominant_freq_hz > 0.0);
  }

  // 10. MpcVibrationController + FakeMpcSolver + ApplyCorrection
  {
    auto solver = std::make_shared<tech_1_5::FakeMpcSolver>();
    assert(solver->Initialize());
    tech_1_5::MpcVibrationController controller(solver);

    tech_1_5::MpcSolveInput input;
    input.vibration.severity = tech_1_5::VibrationSeverity::kNone;
    input.vibration.accel_rms = 0.5;
    tech_1_5::MpcCorrection corr;
    assert(!controller.ComputeCorrection(input, &corr));

    input.vibration.severity = tech_1_5::VibrationSeverity::kSevere;
    input.vibration.accel_rms = 6.0;
    assert(controller.ComputeCorrection(input, &corr));
    assert(corr.active && corr.damping_factor > 0.0);

    JointCommand base;
    base.positions.resize(12, 0.0);
    base.torques.resize(12, 0.0);
    base.velocities.resize(12, 0.0);
    auto final_cmd = tech_1_5::ApplyCorrection(base, corr);
    assert(final_cmd.torques.size() == 12);
    bool any_nonzero = false;
    for (double t : final_cmd.torques) if (std::abs(t) > 1e-9) { any_nonzero = true; break; }
    assert(any_nonzero);
  }

  // 11. GratingMetricsRecorder：线程安全 + PPT 阈值
  {
    tech_1_5::GratingMetricsRecorder recorder;
    recorder.SetGratingMode(true);
    for (std::size_t i = 0; i < 100; ++i) recorder.RecordStep(i * 2000000ULL);
    for (std::size_t i = 0; i < 5; ++i) {
      tech_1_5::GaitAnomalyEvent e;
      e.timestamp_ns = i * 2000000ULL;
      e.anomaly_type = "resonance";
      recorder.RecordAnomaly(e);
    }
    for (std::size_t i = 0; i < 100; ++i) recorder.UpdateSpeed(0.85, i * 1000000000ULL);
    auto m = recorder.Snapshot();
    assert(m.total_steps == 100 && m.anomaly_count == 5);
    assert(std::abs(m.anomaly_rate - 0.05) < 1e-6);
    assert(m.avg_speed >= 0.8);
    assert(!recorder.ThresholdsPass());  // 距离不够
    for (std::size_t i = 0; i < 3000; ++i) recorder.UpdateSpeed(0.85, (200 + i) * 1000000000ULL);
    assert(recorder.ThresholdsPass());
  }

  // ========== tech_1_6：弱纹理融合导航组件 ==========

  // 12. MultiSensorSynchronizer：多传感器时间同步
  {
    tech_1_6::MultiSensorSynchronizer sync(5.0);
    ImuSample imu;
    imu.timestamp_ns = 1000000ULL;
    sync.UpdateImu(imu);
    FootForceSample ff;
    ff.timestamp_ns = 1000000ULL;
    ff.normal_forces = {40.0, 35.0, 30.0, 38.0};
    sync.UpdateFootForce(ff);
    // 激光/相机未接入 → valid=false（不伪造）
    sync.UpdateLidar(1000000ULL, false);
    auto packet = sync.TrySync();
    assert(packet.has_value());
    assert(packet->imu_valid);
    assert(packet->foot_force_valid);
    assert(!packet->lidar_valid);  // 未接入，不伪造
    assert(!sync.IsSensorValid(tech_1_6::SensorType::kLidar));
    assert(sync.IsSensorValid(tech_1_6::SensorType::kImu));

    // DataFreshnessGuard 超时验证：tolerance=1ms，sleep 后标记为 stale
    tech_1_6::MultiSensorSynchronizer fresh_sync(1.0);
    fresh_sync.UpdateImu(imu);
    assert(fresh_sync.IsSensorValid(tech_1_6::SensorType::kImu));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    assert(!fresh_sync.IsSensorValid(tech_1_6::SensorType::kImu));
  }

  // 13. KinematicConstraintBuilder：50N 足力约束 + 关节限位
  {
    tech_1_6::KinematicConstraintBuilder builder;
    // 关节限位
    RobotStateRaw state;
    state.joint_positions = {0.1, -0.2, 0.3, 0.0, -0.1, 0.2,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    state.joint_velocities = {1.0, 2.0, 0.5, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    auto cs_robot = builder.BuildFromRobotState(state, 1000ULL);
    assert(cs_robot.all_satisfied);
    // 超限速度
    state.joint_velocities[0] = 20.0;
    auto cs_bad = builder.BuildFromRobotState(state, 1000ULL);
    assert(!cs_bad.all_satisfied);

    // 50N 足力约束
    FootForceSample ff;
    ff.normal_forces = {40.0, 55.0, 30.0, 38.0};  // 55N > 50N
    auto cs_foot = builder.BuildFromFootForce(ff);
    assert(!cs_foot.all_satisfied);  // 第 1 足超 50N
    // 全部 <= 50N
    ff.normal_forces = {40.0, 45.0, 30.0, 38.0};
    auto cs_ok = builder.BuildFromFootForce(ff);
    assert(cs_ok.all_satisfied);

    // Merge
    auto merged = tech_1_6::KinematicConstraintBuilder::Merge(cs_robot, cs_ok);
    assert(merged.all_satisfied);
  }

  // 14. TightlyCoupledLocalizer + FakeLocalizerBackend：紧耦合定位
  {
    auto backend = std::make_shared<tech_1_6::FakeLocalizerBackend>();
    assert(backend->Initialize());
    tech_1_6::TightlyCoupledLocalizer localizer(backend);

    tech_1_6::LocalizationSolveInput input;
    input.sensors.imu_valid = true;
    input.sensors.foot_force_valid = true;
    input.sensors.lidar_valid = false;      // 激光失效（不伪造）
    input.sensors.camera_left_valid = false;
    input.sensors.camera_right_valid = false;
    input.constraints.all_satisfied = true;
    input.prev_pose.x = 1.0;
    input.prev_pose.y = 2.0;
    input.timestamp_ns = 1000ULL;

    tech_1_6::FusionPoseOutput out;
    // IMU+足力 = 2 源 → degraded 但 valid（源数 >= 2）
    assert(localizer.Localize(input, &out));
    assert(out.valid);
    assert(out.localization_state == "degraded");  // 激光失效 → 降级
    // 位姿保持上一帧（不虚构运动）
    assert(std::abs(out.x - 1.0) < 1e-9);
    // 有效源不包含 lidar（不伪造）
    bool has_lidar = false;
    for (const auto& s : out.valid_sources) {
      if (s == "lidar") has_lidar = true;
    }
    assert(!has_lidar);

    // 全源有效 → tracking
    input.sensors.lidar_valid = true;
    input.sensors.camera_left_valid = true;
    input.sensors.camera_right_valid = true;
    assert(localizer.Localize(input, &out));
    assert(out.localization_state == "tracking");
    assert(out.valid_sources.size() == 5);

    // IMU 无效 → 定位失败
    input.sensors.imu_valid = false;
    input.sensors.lidar_valid = true;
    assert(!localizer.Localize(input, &out));
  }

  // 15. LocalizationDegradationMonitor：退化监控
  {
    tech_1_6::LocalizationDegradationMonitor monitor;
    // 全源有效 → healthy
    tech_1_6::SyncedSensorPacket full;
    full.lidar_valid = true;
    full.camera_left_valid = true;
    full.camera_right_valid = true;
    full.imu_valid = true;
    full.foot_force_valid = true;
    auto status = monitor.Evaluate(full);
    assert(status.level == tech_1_6::DegradationLevel::kHealthy);
    assert(status.valid_sources.size() == 5);

    // 激光失效 → degraded（不伪造）
    tech_1_6::SyncedSensorPacket no_lidar;
    no_lidar.lidar_valid = false;
    no_lidar.camera_left_valid = true;
    no_lidar.imu_valid = true;
    no_lidar.foot_force_valid = true;
    auto deg = monitor.Evaluate(no_lidar);
    assert(deg.level == tech_1_6::DegradationLevel::kDegraded);
    assert(deg.lidar_valid == false);

    // 源不足 → lost
    tech_1_6::SyncedSensorPacket minimal;
    minimal.imu_valid = true;
    auto lost = monitor.Evaluate(minimal);
    assert(lost.level == tech_1_6::DegradationLevel::kLost);
  }

  // 16. NavigationExecutor：跨楼层路径执行
  {
    tech_1_6::NavigationExecutor nav;
    std::vector<tech_1_6::NavWaypoint> path;
    tech_1_6::NavWaypoint wp;
    wp.x = 1.0; wp.floor_id = 1; path.push_back(wp);
    wp.x = 2.0; wp.floor_id = 1; wp.is_stairs = true; path.push_back(wp);
    wp.x = 3.0; wp.z = 3.0; wp.floor_id = 2; wp.is_stairs = false; path.push_back(wp);
    assert(nav.LoadPath(path));
    assert(nav.Start());

    // 到达第一个航点
    tech_1_6::FusionPoseOutput pose;
    pose.x = 1.0; pose.valid = true;
    auto snap = nav.Update(pose);
    assert(snap.current_waypoint >= 1);
    assert(nav.Progress() > 0.0);

    // 退化时暂停
    nav.OnDegradation(true);
    pose.x = 2.0;
    snap = nav.Update(pose);
    assert(snap.state == tech_1_6::NavigationState::kDegradedPause);

    // 恢复后继续
    nav.OnDegradation(false);
    pose.x = 2.0;
    snap = nav.Update(pose);
    assert(snap.current_waypoint >= 2);

    // 到达终点
    pose.x = 3.0; pose.z = 3.0;
    snap = nav.Update(pose);
    assert(snap.state == tech_1_6::NavigationState::kCompleted);
    assert(std::abs(nav.Progress() - 1.0) < 1e-9);
  }

  // ========== tech_1_7：语义地图与定位组件 ==========

  // 17. PointcloudTransformer：点云坐标转换（位姿无效时不虚构）
  {
    tech_1_7::PointcloudTransformer transformer;
    transformer.SetTargetFrame("map");
    assert(transformer.TargetFrame() == "map");

    // 位姿有效：转换本体系点 → map 系
    tech_1_7::RobotPose pose;
    pose.x = 1.0; pose.y = 2.0; pose.z = 0.0;
    pose.qw = 1.0;  // 单位四元数（无旋转）
    pose.valid = true;
    pose.localization_state = "tracking";
    pose.timestamp_ns = 1000ULL;

    std::vector<tech_1_7::Point3D> cloud;
    cloud.push_back(tech_1_7::Point3D{0.5, 0.0, 0.0});
    auto out = transformer.Transform(cloud, pose);
    assert(out.valid);
    assert(out.points.size() == 1);
    assert(std::abs(out.points[0].x - 1.5) < 1e-9);  // 0.5 + 1.0 平移
    assert(std::abs(out.points[0].y - 2.0) < 1e-9);

    // 红线：位姿失效 → valid=false（不虚构）
    pose.valid = false;
    auto bad = transformer.Transform(cloud, pose);
    assert(!bad.valid);

    // 红线：定位 lost → valid=false（不虚构）
    pose.valid = true;
    pose.localization_state = "lost";
    auto lost = transformer.Transform(cloud, pose);
    assert(!lost.valid);
  }

  // 18. WeightedGridBuilder：三源加权（BIM/SLAM/点云）
  {
    tech_1_7::WeightedGridBuilder builder(0.05);  // 5cm 分辨率
    builder.SetMapExtent(0.0, 0.0, 20, 20);  // 1m × 1m 区域

    // BIM 先验
    std::vector<tech_1_7::BimElement> bim;
    bim.push_back(tech_1_7::BimElement{"b1", 0.1, 0.1, 0.0, 0.8});
    builder.LoadBim(bim);

    // SLAM 实时（落入 [2,2] 栅格，与 BIM 同栅格）
    tech_1_7::SlamObservation slam;
    slam.landmarks.push_back(tech_1_7::Point3D{0.12, 0.12, 0.0});
    slam.confidence = 0.6;
    builder.LoadSlam(slam);

    // 点云实测
    tech_1_7::TransformedCloud cloud;
    cloud.valid = true;
    cloud.points.push_back(tech_1_7::Point3D{0.1, 0.1, 0.0});
    cloud.points.push_back(tech_1_7::Point3D{0.11, 0.09, 0.0});
    builder.LoadCloud(cloud);

    auto grid = builder.Build(2000ULL);
    assert(grid.valid);
    assert(grid.size_x == 20 && grid.size_y == 20);
    assert(grid.cells.size() == 400);

    // 找到 (0.1, 0.1) 落入的栅格（0.1 / 0.05 = 2 → 索引 2, 2）
    const auto& cell = grid.cells[2 * 20 + 2];
    assert(cell.bim_weight > 0.0);
    assert(cell.slam_weight > 0.0);
    assert(cell.cloud_weight > 0.0);
    assert(cell.weight > 0.0);
    assert(cell.bim_id == "b1");
    assert(cell.has_measurement);

    // 红线：无任何数据源 → valid=false（不虚构栅格）
    tech_1_7::WeightedGridBuilder empty(0.05);
    empty.SetMapExtent(0.0, 0.0, 10, 10);
    auto empty_grid = empty.Build(3000ULL);
    assert(!empty_grid.valid);
  }

  // 19. PositionMatcher：位置匹配 + 红线（未定位不输出物理位置）
  {
    tech_1_7::WeightedGridBuilder builder(0.05);
    builder.SetMapExtent(0.0, 0.0, 20, 20);
    std::vector<tech_1_7::BimElement> bim;
    bim.push_back(tech_1_7::BimElement{"b1", 0.1, 0.1, 0.0, 0.9});
    builder.LoadBim(bim);
    auto grid = builder.Build(4000ULL);
    assert(grid.valid);

    tech_1_7::PositionMatcher matcher(0.30);  // 30cm 搜索半径

    // 检测事件落在权重栅格附近 → 成功定位
    tech_1_7::DetectionEvent ev;
    ev.event_id = "alert_1";
    ev.detection_x = 0.1;
    ev.detection_y = 0.1;
    ev.detection_z = 0.0;
    ev.valid = true;
    ev.timestamp_ns = 4000ULL;
    auto result = matcher.Match(ev, grid);
    assert(result.localized);
    assert(result.confidence > 0.0);
    assert(result.bim_id == "b1");
    assert(result.coordinate_source == "bim");  // BIM 主导
    // 厘米级位置：栅格中心
    assert(std::abs(result.matched_x - 0.125) < 1e-9);  // (2+0.5)*0.05

    // 红线：检测事件远离任何栅格 → 未定位
    ev.detection_x = 5.0;  // 远离 (0.1, 0.1)
    ev.detection_y = 5.0;
    auto unlocalized = matcher.Match(ev, grid);
    assert(!unlocalized.localized);
    assert(unlocalized.confidence == 0.0 || unlocalized.confidence < matcher.kMinConfidence);
    assert(unlocalized.coordinate_source == "unlocalized");

    // 批量匹配
    std::vector<tech_1_7::DetectionEvent> events;
    tech_1_7::DetectionEvent e1; e1.event_id = "e1";
    e1.detection_x = 0.1; e1.detection_y = 0.1; e1.valid = true;
    tech_1_7::DetectionEvent e2; e2.event_id = "e2";
    e2.detection_x = 5.0; e2.detection_y = 5.0; e2.valid = true;
    events.push_back(e1);
    events.push_back(e2);
    auto results = matcher.MatchAll(events, grid);
    assert(results.size() == 2);
    assert(results[0].localized);
    assert(!results[1].localized);

    // 红线：grid 无效 → 全部未定位
    tech_1_7::WeightedGrid invalid_grid;
    invalid_grid.valid = false;
    auto r = matcher.Match(e1, invalid_grid);
    assert(!r.localized);
    assert(r.coordinate_source == "unlocalized");
  }

  // 20. AlarmLocationPublisher：封装 SemanticAlarm + 红线
  {
    // 成功定位 → 写入物理位置 + 置信度 + 来源
    tech_1_7::PositionMatchResult ok;
    ok.event_id = "alert_ok";
    ok.matched_x = 1.5;
    ok.matched_y = 2.5;
    ok.matched_z = 0.0;
    ok.bim_id = "b1";
    ok.slam_id = "";
    ok.confidence = 0.85;
    ok.localized = true;
    ok.coordinate_source = "bim";
    ok.timestamp_ns = 5000ULL;
    auto fields = tech_1_7::AlarmLocationPublisher::Publish(ok);
    assert(fields.localized);
    assert(fields.detection_event == "alert_ok");
    assert(std::abs(fields.location_x - 1.5) < 1e-9);
    assert(std::abs(fields.location_y - 2.5) < 1e-9);
    assert(fields.bim_id == "b1");
    assert(std::abs(fields.confidence - 0.85) < 1e-9);

    // 红线：未定位 → 不写虚假物理监测点
    tech_1_7::PositionMatchResult unloc;
    unloc.event_id = "alert_unloc";
    unloc.localized = false;
    unloc.coordinate_source = "unlocalized";
    unloc.confidence = 0.0;
    auto empty_fields = tech_1_7::AlarmLocationPublisher::Publish(unloc);
    assert(!empty_fields.localized);
    assert(empty_fields.bim_id.empty());
    assert(empty_fields.slam_id.empty());
    assert(std::abs(empty_fields.confidence) < 1e-9);
    assert(std::abs(empty_fields.location_x) < 1e-9);  // 位置保持 0

    // LocalizedRatio 统计
    std::vector<tech_1_7::PositionMatchResult> all = {ok, unloc};
    double ratio = tech_1_7::AlarmLocationPublisher::LocalizedRatio(all);
    assert(std::abs(ratio - 0.5) < 1e-9);
  }

  // ========== tech_1_8：动态红外测温组件 ==========

  // 21. ThermalVisualImuSynchronizer：30fps+1000Hz 同步（红线：IMU 无效不虚构）
  {
    tech_1_8::ThermalVisualImuSynchronizer sync(5.0);
    assert(std::abs(sync.ToleranceMs() - 5.0) < 1e-9);

    // 红线：未推入 IMU → TrySync 返回 nullopt（不虚构同步结果）
    assert(!sync.TrySync().has_value());

    tech_1_8::ThermalFrame frame;
    frame.raw_temperature = 50.0;
    frame.angle = 30.0;
    frame.distance = 3.0;
    frame.emissivity = 0.95;
    frame.correction_factor = 1.0;
    frame.timestamp_ns = 1000ULL;
    frame.valid = true;
    sync.UpdateThermal(frame);

    ImuSample imu;
    imu.gx = 0.0; imu.gy = 1.0; imu.gz = 0.0;
    imu.timestamp_ns = 1100ULL;
    sync.UpdateImu(imu);

    auto packet = sync.TrySync();
    assert(packet.has_value());
    assert(packet->imu_valid);
    assert(packet->thermal_valid);
    assert(std::abs(packet->thermal.raw_temperature - 50.0) < 1e-9);
    assert(packet->synced_timestamp_ns == 1100ULL);  // IMU 为基准
    assert(sync.IsImuValid());
    assert(sync.IsThermalValid());
  }

  // 22. ThermalImageStabilizer：IMU 角速度→位移补偿（红线：IMU/thermal 无效不虚构）
  {
    tech_1_8::ThermalVisualImuSynchronizer sync(5.0);
    tech_1_8::ThermalImageStabilizer stab(8.0, 400.0);
    assert(std::abs(stab.ExposureMs() - 8.0) < 1e-9);

    tech_1_8::ThermalFrame frame;
    frame.raw_temperature = 50.0;
    frame.angle = 30.0;
    frame.distance = 3.0;
    frame.emissivity = 0.95;
    frame.correction_factor = 1.0;
    frame.valid = true;
    frame.timestamp_ns = 1000ULL;
    sync.UpdateThermal(frame);

    ImuSample imu;
    imu.gx = 0.0; imu.gy = 1.0; imu.gz = 0.0;  // 1 rad/s 绕 y
    imu.timestamp_ns = 1100ULL;
    sync.UpdateImu(imu);

    auto packet = sync.TrySync();
    assert(packet.has_value());

    auto out = stab.Stabilize(*packet);
    assert(out.valid);
    assert(out.error_state == "ok");
    // displacement_x = gy * dt * focal = 1.0 * 0.008 * 400 = 3.2
    assert(std::abs(out.displacement_x - 3.2) < 1e-6);
    assert(std::abs(out.displacement_y - 0.0) < 1e-6);  // gx=0
    assert(std::abs(out.stabilized_temperature - 50.0) < 1e-9);
    assert(std::abs(out.emissivity - 0.95) < 1e-9);

    // 红线：IMU 无效 → valid=false
    tech_1_8::SyncedThermalPacket no_imu;
    no_imu.thermal = frame;
    no_imu.thermal_valid = true;
    no_imu.imu_valid = false;
    auto bad = stab.Stabilize(no_imu);
    assert(!bad.valid);
    assert(bad.error_state == "imu_invalid");

    // 红线：thermal 无效 → valid=false
    tech_1_8::SyncedThermalPacket no_th;
    no_th.thermal_valid = false;
    no_th.imu_valid = true;
    auto bad2 = stab.Stabilize(no_th);
    assert(!bad2.valid);
    assert(bad2.error_state == "thermal_invalid");
  }

  // 23. EmissivityCompensator：辐射率补偿（红线：非物理辐射率标记 invalid）
  {
    tech_1_8::EmissivityCompensator comp;

    tech_1_8::StabilizedThermal in;
    in.stabilized_temperature = 100.0;
    in.emissivity = 0.95;
    in.angle = 30.0;
    in.distance = 3.0;
    in.correction_factor = 1.0;
    in.valid = true;
    auto out = comp.Compensate(in);
    assert(out.valid);
    assert(out.error_state == "ok");
    // compensated = 100 / 0.95^0.25 ≈ 100 / 0.987278 ≈ 101.289
    assert(std::abs(out.compensated_temperature - 100.0 / 0.987277) < 0.01);
    assert(std::abs(out.emissivity - 0.95) < 1e-9);

    // 红线：辐射率 ≤0 → invalid_emissivity
    in.emissivity = 0.0;
    auto bad = comp.Compensate(in);
    assert(!bad.valid);
    assert(bad.error_state == "invalid_emissivity");

    // 红线：辐射率 >1 → invalid_emissivity
    in.emissivity = 1.5;
    auto bad2 = comp.Compensate(in);
    assert(!bad2.valid);
    assert(bad2.error_state == "invalid_emissivity");

    // 红线：输入无效 → input_invalid
    in.emissivity = 0.95;
    in.valid = false;
    auto bad3 = comp.Compensate(in);
    assert(!bad3.valid);
    assert(bad3.error_state == "input_invalid");
  }

  // 24. AngleDistanceCompensator：角度+距离补偿（红线：输入无效不虚构）
  {
    tech_1_8::AngleDistanceCompensator comp;

    tech_1_8::EmissivityCompensated in;
    in.compensated_temperature = 100.0;
    in.angle = 30.0;
    in.distance = 3.0;
    in.correction_factor = 1.0;
    in.valid = true;
    auto out = comp.Compensate(in);
    assert(out.valid);
    assert(out.error_state == "ok");
    // compensated = 100 / cos(30°) * 1.0 ≈ 100 / 0.8660254 ≈ 115.470
    assert(std::abs(out.compensated_temperature - 100.0 / 0.8660254) < 0.01);

    // 距离补偿：correction_factor=1.05
    in.correction_factor = 1.05;
    auto out2 = comp.Compensate(in);
    assert(out2.valid);
    assert(std::abs(out2.compensated_temperature -
                    (100.0 / 0.8660254) * 1.05) < 0.01);

    // 红线：输入无效 → input_invalid
    in.valid = false;
    auto bad = comp.Compensate(in);
    assert(!bad.valid);
    assert(bad.error_state == "input_invalid");
  }

  // 25. ThermalRangeValidator：范围校验（红线：超范围不按范围内精度发布）
  {
    tech_1_8::ThermalRangeValidator validator;

    // 范围内：angle=30° (±60), distance=3m (1-5), cf=1.0 (0.95-1.05), em=0.95
    tech_1_8::AngleDistanceCompensated in;
    in.compensated_temperature = 100.0;
    in.angle = 30.0;
    in.distance = 3.0;
    in.correction_factor = 1.0;
    in.valid = true;
    auto ok = validator.Validate(in, 0.95);
    assert(ok.in_range);
    assert(ok.error_state == "ok");
    assert(std::abs(ok.final_temperature - 100.0) < 1e-9);
    assert(validator.AngleInRange(30.0));
    assert(validator.DistanceInRange(3.0));
    assert(validator.CorrectionFactorInRange(1.0));
    assert(validator.EmissivityValid(0.95));

    // 边界：±60°、1m、5m、0.95、1.05 均在范围内
    assert(validator.AngleInRange(-60.0));
    assert(validator.AngleInRange(60.0));
    assert(validator.DistanceInRange(1.0));
    assert(validator.DistanceInRange(5.0));
    assert(validator.CorrectionFactorInRange(0.95));
    assert(validator.CorrectionFactorInRange(1.05));

    // 红线：角度超 ±60° → angle_out_of_range
    in.angle = 70.0;
    auto a_out = validator.Validate(in, 0.95);
    assert(!a_out.in_range);
    assert(a_out.error_state == "angle_out_of_range");
    assert(!validator.AngleInRange(70.0));
    assert(!validator.AngleInRange(-61.0));

    // 红线：距离超 1-5m → distance_out_of_range
    in.angle = 30.0;
    in.distance = 6.0;
    auto d_out = validator.Validate(in, 0.95);
    assert(!d_out.in_range);
    assert(d_out.error_state == "distance_out_of_range");
    assert(!validator.DistanceInRange(6.0));
    assert(!validator.DistanceInRange(0.5));

    // 红线：修正系数超 0.95-1.05 → coefficient_out_of_range
    in.distance = 3.0;
    in.correction_factor = 1.1;
    auto c_out = validator.Validate(in, 0.95);
    assert(!c_out.in_range);
    assert(c_out.error_state == "coefficient_out_of_range");
    assert(!validator.CorrectionFactorInRange(1.1));
    assert(!validator.CorrectionFactorInRange(0.94));

    // 红线：辐射率超 (0,1] → emissivity_out_of_range
    in.correction_factor = 1.0;
    auto e_out = validator.Validate(in, 0.0);
    assert(!e_out.in_range);
    assert(e_out.error_state == "emissivity_out_of_range");
    assert(!validator.EmissivityValid(0.0));
    assert(!validator.EmissivityValid(1.5));

    // 红线：输入无效 → input_invalid
    in.valid = false;
    auto bad = validator.Validate(in, 0.95);
    assert(!bad.in_range);
    assert(bad.error_state == "input_invalid");
  }

  // 9. 声纹前处理（1.9）
  tech_1_9::MultiChannelAudio audio;
  audio.sample_rate = 48000;
  audio.timestamp_ns = 1000000;
  for (std::size_t i = 0; i < tech_1_9::kNumMicrophones; ++i) {
    audio.channels[i] = {1.0f, 2.0f, 3.0f, 4.0f};
  }
  tech_1_9::MicrophoneSynchronizer synchronizer;
  assert(synchronizer.Synchronize(audio, 1000000 + 1000000).valid);
  assert(!synchronizer.Synchronize(audio, 1000000 + 100000000).valid);  // 100ms 过期

  tech_1_9::DirectionalBeamformer beamformer;
  const auto beam = beamformer.Beamform(audio, 0.0);
  assert(beam.valid && beam.mono.size() == 4);
  assert(std::abs(beam.mono[0] - 1.0f) < 1e-6f);

  tech_1_9::MultiChannelAudio common_mode = audio;
  tech_1_9::SpatialFilter spatial_filter;
  assert(spatial_filter.Filter(&common_mode));
  assert(std::abs(common_mode.channels[0][0]) < 1e-6f);

  tech_1_9::SnrEstimator snr_estimator;
  const double snr_db = snr_estimator.EstimateDb({10.0f, 10.0f, 10.0f}, {1.0f, 1.0f, 1.0f});
  assert(std::abs(snr_db - 20.0) < 1e-6);

  std::cout << "core logic smoke test passed (incl. tech_1_3 + tech_1_4 + tech_1_5 + "
               "tech_1_6 + tech_1_7 + tech_1_8 + tech_1_9 components)"
            << std::endl;
  return 0;
}
