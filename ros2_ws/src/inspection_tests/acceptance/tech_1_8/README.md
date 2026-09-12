# 技术 1.8 验收（抗振抑扰的动态红外精准测温）

## 硬边界
- 稳像与温度补偿在 C++ 执行，Python 决策侧只做异常高温判定（不重复稳像/补偿，红线）。
- 超出 PPT 范围（角度 ±60°、距离 1–5 m、修正系数 0.95–1.05、辐射率 (0,1]）时标记 `xxx_out_of_range`，`in_range=false`，不按范围内精度发布（红线）。
- IMU 无效（适配器未注入 / `ReadSample` 失败）时 `TrySync` 返回 `nullopt`，不发布测温结果（不虚构温度/精度，红线）。
- 辐射率非物理值时 `EmissivityCompensator` 标 `invalid`；`AngleDistanceCompensator` 中 `cos(angle)` 下限 0.01 防发散。

## adapter / 注入说明
`tech_1_8_node` 内部五组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **ImuAdapter**：通过 `SetImuAdapter()` 注入；未注入或 `ReadSample` 失败时不发布（与 tech_1_3_node / tech_1_5_node / tech_1_6_node 同理）。
- **ThermalFrame**：通过 `SetThermalFrame()` 注入（占位），真实红外传感器接入后替换为订阅回调（与 tech_1_4_node / tech_1_6_node 示例同理）。
- **ThermalMeasurement**：复用已有消息接口，由节点发布，不新增消息类型。

## PPT 指标（阈值已落地 `thermal_metrics.py`）
- 范围内偏差 ≤ **0.2 ℃**（`PPT_STATIC_DEVIATION_C`）
- 动态精度 ± **0.5 ℃**（`PPT_DYNAMIC_ACCURACY_C`）
- 效率提升 ≥ **10 倍**（`PPT_EFFICIENCY_GAIN`）

## 组件架构
```
ThermalVisualImuSynchronizer → ThermalImageStabilizer
→ EmissivityCompensator → AngleDistanceCompensator
→ ThermalRangeValidator（发布 ThermalMeasurement）
```
- **ThermalVisualImuSynchronizer**：30fps 红外 + 1000Hz IMU 滑窗时间同步，IMU 无效返回 `nullopt`
- **ThermalImageStabilizer**：IMU 角速度→位移补偿，IMU/红外无效时 `valid=false`
- **EmissivityCompensator**：`compensated = raw / ε^0.25`，非物理辐射率标 `invalid`
- **AngleDistanceCompensator**：`compensated = temp / cos(angle) * correction_factor`，`cos` 下限 0.01
- **ThermalRangeValidator**：PPT 范围校验（±60° / 1–5m / 0.95–1.05 / ε∈(0,1]），超范围标 `xxx_out_of_range`

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 本机可运行验证
- Python unittest：
  ```
  set PYTHONPATH=...\ros2_ws\src\inspection_planning_py
  cd ros2_ws\src\inspection_tests\unit_python
  python -m unittest tech_1_1_to_1_9.test_thermal_1_8 -v
  ```
  预期：14/14 通过。
- C++ 纯逻辑 smoke test（clang++ 脱离 ROS）：
  ```
  # 注：<...实现> 必须排除所有 *_node.cpp（依赖 rclcpp）及
  #   tech_1_1/bt_task_executor.cpp、tech_1_1/behavior_tree_nodes.cpp、
  #   tech_1_2/task_lifecycle_executor.cpp（ROS/behaviortree 依赖）
  # 参见 src/tech_1_8/README.md 的完整 PowerShell 命令
  clang++ -std=c++17 -Wall -Wextra -Wpedantic -I<include> smoke_test_core.cpp <common/1.1纯逻辑/1.2纯逻辑/1.3/1.4/1.5/1.6/1.7/1.8 纯逻辑组件 cpp> -o smoke_test.exe
  smoke_test.exe  # 应打印 "core logic smoke test passed (incl. tech_1_3 + tech_1_4 + tech_1_5 + tech_1_6 + tech_1_7 + tech_1_8 components)"
  ```
- ROS 真实编译（需 ROS 2 + colcon）：
  ```
  colcon build --packages-up-to inspection_execution_cpp inspection_planning_py
  ros2 launch inspection_bringup tech_1_8.launch.py
  ```
