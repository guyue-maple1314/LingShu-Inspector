# 技术 1.4 必测场景清单

## S1：C++ DynamicPointcloudSegmenter 点云分割（smoke）
- 输入：机器人宽度 0.46m；宽通道点云 `{-0.3,0},{-0.25,1.0},{0.25,0},{0.3,2.0}`；窄通道 `{-0.2,0},{0.2,0}`
- 期望：
  - 宽通道：`seg.valid && seg.width > 0.46`；
  - 窄通道（可通行宽度 < 0.46）：`!narrow.valid`（不强行通过）。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S2：C++ VisualTextureValidator 视觉纹理校验（smoke）
- 输入：阈值 0.6；`Validate(0.8)` / `Validate(0.3)`
- 期望：`vis_ok.passed && !vis_bad.passed`（低置信度视觉不放过）。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S3：C++ CorridorFusion 融合校验（smoke）
- 输入：几何分割结果 + 视觉校验结果
- 期望：
  - `Fuse(seg, vis_ok).valid == true`（几何+视觉双通过）；
  - `Fuse(seg, vis_bad).valid == false`（视觉失效→整条通道无效）。
- 红线：融合任一源失效不虚构有效通行。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S4：C++ NarrowCorridorExecutor 通行状态机（smoke）
- 输入：margin=0.10；`Start(robot_width=0.46, corridor_width=0.75)` → `Update(0.75, visual_ok=false)`
- 期望：
  - `Start` 返回 true，`State()==kTracking`；
  - 视觉失效 `Update` 返回 false，`State()==kBlocked`（安全停止，不强行通行）。
- 证据：`smoke_test_core.cpp` 断言通过截图。

## S5：Python corridor_plan_validator 通道校验（unittest）
- 输入：`{width=0.75, visual_validation_passed=True, confidence=0.9}` 及变体（width=0.5 太窄 / visual=False）
- 期望：
  - 合法：`validate_corridor → (True, _)`；
  - 太窄（0.5 < 0.46 机器人宽度的合理下限）：`(False, _)`；
  - 视觉失败：`(False, _)`。
- 证据：`test_corridor_1_4.py`（`TestCorridorPlanValidator`）全通过截图。

## S6：Python narrow_corridor_path_planner 路径规划（unittest）
- 输入：合法 corridor 的 centerline `[{0,0},{0.1,1.0}]`；非法（width=0.4）
- 期望：
  - 合法：`plan_path → [(0.0,0.0), ...]`，长度 2，首点为中心线起点；
  - 非法：`plan_path → []`（不输出路径，不虚构）。
- 证据：`test_corridor_1_4.py`（`TestNarrowCorridorPathPlanner`）全通过截图。

## S7：Python CorridorMetrics 通过率统计（unittest）
- 输入：`record_trial(True)` ×2 + `record_trial(False)` ×1
- 期望：`pass_rate() == 2/3`。
- 证据：`test_corridor_1_4.py`（`TestCorridorMetrics`）全通过截图。

## S8：ROS launch 装配 + Topic 命名
- 输入：`ros2 launch inspection_bringup tech_1_4.launch.py`
- 期望：
  - 节点名取自公共 `node_names.hpp`（`kTech14NodeName`，无散写字符串）；
  - 发布 `CorridorState`、订阅感知相关 topic（公共 `topic_names` 集中定义）。
- 证据：`ros2 node list`、`ros2 topic list` 截图；CMakeLists `add_executable` 片段。

## S9：75cm 窄廊道稳定通行实机闭环（最终指标）
- 输入：46cm 机器人通过 75cm 窄廊道 × 多轮
- 期望：
  - 稳定通行（通行率达标）；
  - 通过能力较行业水平提升 25%（需项目方统一基线）；
  - 改造成本降低 90% 以上（需项目方统一口径）；
  - 视觉/几何失效时安全停止（不强行通行）。
- 证据：现场记录与统计表按本场景归档。
