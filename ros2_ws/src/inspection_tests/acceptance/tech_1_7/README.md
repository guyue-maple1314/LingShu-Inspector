# 技术 1.7 验收（高精度时空融合语义地图构建与定位）

## 硬边界
- C++ 侧点云转换 / 栅格赋权 / 位置匹配在节点内运行，Python 决策侧不阻塞地图构建回路。
- BIM 格式尚未获批：`AbstractBimLoader` 抽象接口，`FakeBimLoader` 仅做确定性 BIM 要素输出，不虚构 BIM 数据。
- 位置结果带 `confidence` + `coordinate_source`（`bim_id` / `slam_id` / `unlocalized`），无法对应时保持「未定位」（`localized=false`），不输出虚假物理监测点（红线）。
- 位姿 lost 时 `PointcloudTransformer` 不转换点云；`WeightedGridBuilder` 无数据源时 `valid=false`。

## adapter / 注入说明
`tech_1_7_node` 内部四组件采用 `SetXxx(...)` 注入式设计，组件本身不直接订阅 Topic 或实例化真实适配器：
- **AbstractBimLoader**：BIM 格式批准后创建具体派生，通过 `SetBimLoader()` 注入；未注入时使用默认 `FakeBimLoader`，不虚构 BIM 要素。
- **FusionPose / SLAM 位姿**：由上层节点启动时通过 `OnFusionPose()` / `OnSlamPose()` 回调注入；位姿 lost 时不转换点云。
- **点云 / 检测事件**：当前为占位时间戳（标记 `valid=false`），真实传感器接入后替换为订阅回调，与 tech_1_4_node / tech_1_6_node 示例同理。
- **SemanticAlarm**：复用已有消息接口，由 `AlarmLocationPublisher` 发布，不新增消息类型。

## PPT 指标（阈值已落地 `semantic_location_metrics.py`）
- 语义位置映射精度 **5cm**（0.05m，`SEMANTIC_MAPPING_TOLERANCE_M`）
- 位置报告准确率 ≥ **98%**（`PPT_LOCATION_REPORT_ACCURACY`）

## 组件架构
```
PointcloudTransformer → WeightedGridBuilder
→ PositionMatcher（BIM-SLAM 关联，30cm 半径）
→ AlarmLocationPublisher（发布 SemanticAlarm）
```
- **PointcloudTransformer**：点云坐标转换，位姿 lost 时不输出
- **WeightedGridBuilder**：栅格赋权，无数据源时 valid=false
- **PositionMatcher**：BIM 先验与实时 SLAM 关联，半径 30cm，带 confidence + 坐标来源
- **AlarmLocationPublisher**：检测事件映射到语义位置，发布 SemanticAlarm

## 必测场景
详见 [scenarios.md](./scenarios.md)。

## 证据模板
验收证据按本目录 `scenarios.md` 的场景逐项归档。

## 本机可运行验证
- Python unittest：
  ```
  set PYTHONPATH=...\ros2_ws\src\inspection_planning_py
  cd ros2_ws\src\inspection_tests\unit_python
  python -m unittest tech_1_1_to_1_9.test_semantic_map_1_7 -v
  ```
- C++ 纯逻辑 smoke test（clang++ 脱离 ROS）：
  ```
  # 注：<...实现> 必须排除所有 *_node.cpp（依赖 rclcpp）及
  #   tech_1_1/bt_task_executor.cpp、tech_1_1/behavior_tree_nodes.cpp、
  #   tech_1_2/task_lifecycle_executor.cpp（ROS/behaviortree 依赖）
  # 参见 src/tech_1_7/README.md 的完整 PowerShell 命令
  clang++ -std=c++17 -Wall -Wextra -Wpedantic -I<include> smoke_test_core.cpp <common/1.1纯逻辑/1.2纯逻辑/1.3/1.4/1.5/1.6/1.7 纯逻辑组件 cpp> -o smoke_test.exe
  smoke_test.exe  # 应打印 "core logic smoke test passed (incl. tech_1_3 + tech_1_4 + tech_1_5 + tech_1_6 + tech_1_7 components)"
  ```
- ROS 真实编译（需 ROS 2 + colcon）：
  ```
  colcon build --packages-up-to inspection_execution_cpp inspection_planning_py
  ros2 launch inspection_bringup tech_1_7.launch.py
  ```
