# tech_1_8 抗振抑扰的动态红外精准测温（C++）

## 五组件
- `thermal_visual_imu_synchronizer`：30fps 红外 + 1000Hz IMU 滑窗时间同步（容差可配），IMU 无效返回 `nullopt`（不虚构）
- `thermal_image_stabilizer`：IMU 角速度→位移补偿（曝光时间 × 焦距因子），IMU/红外无效 `valid=false`
- `emissivity_compensator`：`compensated = raw / ε^0.25`，非物理辐射率标 `invalid`
- `angle_distance_compensator`：`compensated = temp / cos(angle) * correction_factor`，`cos` 下限 0.01 防发散
- `thermal_range_validator`：PPT 范围校验（±60° / 1–5m / 0.95–1.05 / ε∈(0,1]），超范围标 `xxx_out_of_range`，`in_range=false`

## adapter / 注入说明
`tech_1_8_node` 内部五组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **ImuAdapter**：通过 `SetImuAdapter()` 注入；未注入或 `ReadSample` 失败时不发布（与 tech_1_3_node / tech_1_5_node / tech_1_6_node 同理）。
- **ThermalFrame**：通过 `SetThermalFrame()` 注入（占位），真实红外传感器接入后替换为订阅回调（与 tech_1_4_node / tech_1_6_node 示例同理）。
- **ThermalMeasurement**：复用已有消息接口，由节点发布，不新增消息类型。

## 红线
- IMU 无效时不发布测温结果（不虚构温度/精度）。
- 超出 PPT 范围（±60°、1–5m、0.95–1.05、ε∈(0,1]）时标记 `xxx_out_of_range`，`in_range=false`，不按范围内精度发布。
- 稳像与温度补偿在 C++ 执行，Python 只做异常高温判定（不重复稳像/补偿）。
- 辐射率非物理值标 `invalid`；`cos(angle)` 下限 0.01 防发散。

## 本机可运行验证
- C++ 纯逻辑 smoke test（clang++ 脱离 ROS，PowerShell）：
  ```powershell
  cd ros2_ws\src
  $src = @()
  $src += "inspection_tests/unit_cpp/tech_1_1_to_1_9/smoke_test_core.cpp"
  $src += Get-ChildItem "inspection_execution_cpp/src/common/*.cpp" | ForEach-Object { $_.FullName }
  $src += @(
    "inspection_execution_cpp/src/tech_1_1/goal_safety_validator.cpp",
    "inspection_execution_cpp/src/tech_1_1/abnormal_switch_monitor.cpp",
    "inspection_execution_cpp/src/tech_1_2/task_resume_executor.cpp",
    "inspection_execution_cpp/src/tech_1_2/task_progress_store.cpp",
    "inspection_execution_cpp/src/tech_1_2/preemption_latency_monitor.cpp"
  )
  foreach ($d in @("tech_1_3","tech_1_4","tech_1_5","tech_1_6","tech_1_7","tech_1_8")) {
    $src += Get-ChildItem "inspection_execution_cpp/src/$d/*.cpp" |
            Where-Object { $_.Name -notlike "*_node.cpp" } |
            ForEach-Object { $_.FullName }
  }
  clang++ -std=c++17 -Wall -Wextra -Wpedantic -Iinspection_execution_cpp/include $src -o smoke_test_1_8.exe
  ./smoke_test_1_8.exe
  ```
  预期：`core logic smoke test passed (incl. tech_1_3 + tech_1_4 + tech_1_5 + tech_1_6 + tech_1_7 + tech_1_8 components)`
  注：必须排除所有 `*_node.cpp`（依赖 rclcpp，本机无 ROS）及 3 个 ROS/behaviortree 依赖文件（`tech_1_1/bt_task_executor.cpp`、`tech_1_1/behavior_tree_nodes.cpp`、`tech_1_2/task_lifecycle_executor.cpp`）。上面 `Where-Object { -notlike "*_node.cpp" }` 已自动排除各 tech 目录的 node 文件。

- ROS 真实编译（需 ROS 2 + colcon）：
  ```
  colcon build --packages-up-to inspection_execution_cpp
  ros2 launch inspection_bringup tech_1_8.launch.py
  ```
