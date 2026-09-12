# tech_1_7 高精度时空融合语义地图与定位（C++）

## 四组件
- `pointcloud_transformer`：点云坐标转换（位姿 lost 时不输出，避免虚假点云）
- `weighted_grid_builder`：栅格赋权（无数据源时 `valid=false`）
- `position_matcher`：BIM 先验与实时 SLAM 关联（半径 30cm），位置结果带 `confidence` + `coordinate_source`（`bim_id`/`slam_id`/`unlocalized`）
- `alarm_location_publisher`：发布 `SemanticAlarm`（复用已有消息，不新增类型）

## adapter / 注入说明
`tech_1_7_node` 内部四组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **AbstractBimLoader**：BIM 格式批准后创建具体派生，通过 `SetBimLoader()` 注入；未注入时使用默认 `FakeBimLoader`，不虚构 BIM 要素（与 `AbstractMpcSolver` / `AbstractLocalizerBackend` 同理）。
- **FusionPose / SLAM 位姿**：上层节点通过 `OnFusionPose()` / `OnSlamPose()` 回调注入；位姿 lost 时不转换点云。
- **点云 / 检测事件**：当前为占位时间戳（`valid=false`），真实传感器接入后替换为订阅回调（与 tech_1_4_node / tech_1_6_node 示例同理）。

## 红线
- 位姿 lost 时不转换点云；栅格无数据源时 `valid=false`。
- 无法对应时保持「未定位」（`localized=false`，`coordinate_source="unlocalized"`），不伪造物理监测点。
- BIM 走抽象接口 `AbstractBimLoader`，不锁具体库（BIM 格式尚未获批）。

## 本机可运行验证
- C++ 纯逻辑 smoke test（clang++ 脱离 ROS，PowerShell）：
  ```powershell
  cd ros2_ws\src
  $src = "inspection_tests/unit_cpp/tech_1_1_to_1_9/smoke_test_core.cpp"
  $src += (Get-ChildItem "inspection_execution_cpp/src/common/*.cpp").FullName
  $src += "inspection_execution_cpp/src/tech_1_1/goal_safety_validator.cpp"
  $src += "inspection_execution_cpp/src/tech_1_1/abnormal_switch_monitor.cpp"
  $src += "inspection_execution_cpp/src/tech_1_2/task_resume_executor.cpp"
  $src += "inspection_execution_cpp/src/tech_1_2/task_progress_store.cpp"
  $src += "inspection_execution_cpp/src/tech_1_2/preemption_latency_monitor.cpp"
  foreach ($d in @("tech_1_3","tech_1_4","tech_1_5","tech_1_6","tech_1_7")) {
    $src += (Get-ChildItem "inspection_execution_cpp/src/$d/*.cpp" |
             Where-Object { $_.Name -notlike "*_node.cpp" }).FullName
  }
  clang++ -std=c++17 -Wall -Wextra -Wpedantic -Iinspection_execution_cpp/include $src -o smoke_test_1_7.exe
  ./smoke_test_1_7.exe
  ```
  预期：`core logic smoke test passed (incl. tech_1_3 + tech_1_4 + tech_1_5 + tech_1_6 + tech_1_7 components)`
  注：必须排除所有 `*_node.cpp`（依赖 rclcpp，本机无 ROS）及 3 个 ROS/behaviortree 依赖文件（`tech_1_1/bt_task_executor.cpp`、`tech_1_1/behavior_tree_nodes.cpp`、`tech_1_2/task_lifecycle_executor.cpp`）。上面 `Where-Object { -notlike "*_node.cpp" }` 已自动排除各 tech 目录的 node 文件。

- ROS 真实编译（需 ROS 2 + colcon）：
  ```
  colcon build --packages-up-to inspection_execution_cpp
  ros2 launch inspection_bringup tech_1_7.launch.py
  ```
