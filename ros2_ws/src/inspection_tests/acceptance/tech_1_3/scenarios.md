# 技术 1.3 必测场景清单

## S1：离线地形 PPT 参数化与随机化
- 输入：seed=1..10，terrain_type ∈ {flat, slope, stairs, grating, rough}
- 期望：
  - 相同 seed 下 terrain_id、elevations、frictions、heights 完全一致（可复现性）。
  - grating 场景 slope_deg=0、step_height=0，stairs 场景 grate_bar_xxx=0。
  - elevations.shape = (rows, cols)，rows×cols = grid_rows×grid_cols。
- 证据：单测 `test_terrain_1_3`、`test_randomizers_1_3` 通过截图。

## S2：离线 PPO 抽象训练管线 + Checkpoint + 早停
- 输入：FakeTrainer，total=400k step，ckpt=100k step，early_stop_success_rate=0.9
- 期望：
  - 无早停（阈值 1.001 > max_pass_rate 1.0）时，保存 checkpoint 数 ≥ 4。
  - 启用早停（阈值 0.9）时，训练在 pass_rate 达阈后中止且 `early_stop_reached=True`。
  - 每轮保存 training_manifest.json，包含 training_id / config / stats。
- 证据：单测 `test_trainer_export_metrics_1_3` 通过截图；生成的 manifest.json 片段。

## S3：策略导出与 Metadata 可读
- 输入：`PolicyExporter.write_metadata()` 写出的 JSON
- 期望：
  - `observation_dim / action_dim / observation_schema / action_schema / randomization_version` 齐全。
  - C++ `PpoPolicyLoaderImpl.Load(dir)` 能解析并填入 `PolicyMetadata`。
- 证据：单测 + smoke_test `loader.Load(".")` 断言通过。

## S4：迁移指标 ≥ PPT 阈值
- 输入：`SimPassLog` 组合（grating/rough/slope/stairs 场景），含 30 次实机试跑
- 期望：
  - `terrain_pass_rate("complex") >= 0.9`
  - `sim_to_real_transfer_success_rate() >= 0.968`
  - `summary_text()` 可读，`thresholds_pass()` 为 True
- 证据：单测 `PPT_thresholds_pass` 通过截图；一份真实 CSV 级汇总报告。

## S5：在线 C++ ObservationBuilder 17 维观测
- 输入：IMU identity 四元数 + 足力 {60,55,40,50}N + contact={1,1,0,1} + vel=0.8m/s
- 期望：
  - 维度 17；rpy=0；angular vel 通过；contact 正确；
  - 足力以 50N 归一：[1.2, 1.1, 0.8, 1.0]。
- 证据：smoke_test 断言通过。

## S6：在线 C++ PolicyOutputValidator 安全拦截
- 输入：维度错误、范围超 1.0、时效 101ms
- 期望：三种异常全部 Validate 失败；范围超限时 Clamp 生效且无越界。
- 证据：smoke_test 断言通过。

## S7：在线 C++ PpoPolicyLoaderImpl 不虚构推理
- 输入：metadata.json 已加载但无权重后端（未注入 `inference_hook`）
- 期望：`Load(dir)==true`，`IsLoaded()==false`，`Infer(...)==false`，act 不被写入假数据。
- 红线：任何情况下不伪造策略输出。
- 证据：smoke_test 断言通过。

## S8：在线 C++ LocomotionCommandAdapter 透传安全
- 输入：12 维 ±1 动作映射 ±0.04 rad；维度错误；SDK 未连接
- 期望：
  - 12 维 OK → positions[i] = action[i] × joint_delta_range_rad；
  - 维度错误 → 失败且不调用 SDK Send；
  - SDK 断开 → ConvertAndSend 失败。
- 证据：smoke_test 断言通过（含 FakeRobot 计数与位置值断言）。

## S9：ROS launch 装配 + Topic 命名
- 输入：`ros2 launch inspection_bringup tech_1_3.launch.py`
- 期望：
  - 节点名为 `kTech13NodeName`（公共 node_names.hpp 集中定义），无散写字符串；
  - 订阅 `/imu /foot_force`，发布 `/terrain_observation`（公共 topic_names 集中定义）。
- 证据：`ros2 node list`、`ros2 topic list` 截图；CMakeLists add_executable 片段。

## S10：实机迁移闭环（最终指标汇总）
- 输入：PPT 场景（光栅、台阶、斜坡、碎石）× 3 个实机试跑者
- 期望：按 S4 统计通行率、迁移成功率，满足 PPT 阈值；附 ROS bag、策略导出目录、参数化 JSON、transfer_metrics JSON。
- 证据：现场记录与统计表按本场景归档。
