# 技术 1.6 验收场景（弱纹理环境高精度融合导航）

## S1：全源融合定位（healthy）
- **输入**：激光+双目相机+IMU+足力全部有效
- **预期**：定位状态 `healthy`，5 个有效数据源，定位精度 ±2cm
- **验证**：`localization_state == "healthy"`，`valid_sources.size() == 5`

## S2：激光失效降级（degraded，不伪造）
- **输入**：激光失效，视觉+惯导+足力有效
- **预期**：定位状态 `degraded`，`lidar_valid == false`，有效源不含 "lidar"
- **红线**：不伪造激光有效状态，降级融合维持定位
- **验证**：`degradation.level == kDegraded`，`lidar_failed == true`

## S3：定位丢失（lost）
- **输入**：仅 IMU 有效（源数 < 2）
- **预期**：定位状态 `lost`，定位不可用
- **验证**：`degradation.level == kLost`

## S4：50N 足力约束
- **输入**：足力 [40, 55, 30, 38] N（55N > 50N）
- **预期**：约束集 `all_satisfied == false`
- **验证**：足力约束触发，第 1 足超限

## S5：50N 足力约束通过
- **输入**：足力 [40, 45, 30, 38] N（全部 ≤ 50N）
- **预期**：约束集 `all_satisfied == true`

## S6：关节限位约束
- **输入**：关节速度 20 rad/s（超 ±10 rad/s 限位）
- **预期**：约束集 `all_satisfied == false`

## S7：跨楼层导航执行
- **输入**：3 航点路径（F1 → 楼梯 → F2）
- **预期**：依次到达航点，进度 0→1，状态 kIdle→kNavigating→kOnStairs→kCompleted
- **验证**：`nav.Progress() == 1.0`，`state == kCompleted`

## S8：退化时暂停导航
- **输入**：导航执行中注入退化信号
- **预期**：状态切换为 `kDegradedPause`，不推进航点
- **验证**：恢复后继续推进

## S9：PPT 定位精度通过
- **输入**：10 次定位采样，最大误差 1.5cm
- **预期**：`accuracy_pass == true`（1.5cm < 2cm）

## S10：PPT 定位精度失败
- **输入**：定位误差 5cm > 2cm
- **预期**：`accuracy_pass == false`

## S11：退化维持率通过
- **输入**：10 次退化，0 次丢失
- **预期**：`degraded_maintain_pass == true`（100% ≥ 90%）

## S12：退化维持率失败
- **输入**：5 次退化 + 5 次丢失
- **预期**：`degraded_maintain_pass == false`（50% < 90%）

## S13：激光失效比例统计
- **输入**：5 次激光有效 + 5 次激光失效
- **预期**：`lidar_failure_ratio == 0.5`（钢格网楼梯感知盲区指标）

## S14：有效源覆盖率
- **输入**：3/5 有效源
- **预期**：`source_coverage == 0.6`

## S15：/navigate_goal Action 闭环（需 ROS 2）
- **输入**：向 `tech_1_6_node` 发送 `NavigateGoal` 目标（目标点与当前位置重合）
- **预期**：接受目标 → 反馈 `current_pose` 与 `state` → 到达容差 10 cm 内返回
  `success=true`；`nav_timeout_sec`（默认 60 s）内未到达则 abort，超时原因写入 message
- **证据**：`ros2 action send_goal /navigate_goal ...` 输出 + 反馈序列
