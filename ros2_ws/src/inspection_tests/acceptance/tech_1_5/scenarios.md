# 技术 1.5 必测场景清单

## S1：FootContactEstimator 接触状态分类（C++ smoke）
- 输入：500Hz 足力，4 足分别模拟：45N 稳定 / 10N 低稳定 / 5N 太低 / 高方差(50N↔5N)
- 期望：SolidContact / GratingContact / NoContact / FalseContact
- 证据：smoke_test 断言通过

## S2：VibrationEstimator 振动分级（C++ smoke）
- 输入：1000Hz IMU，先低振动(0.1 m/s²)再高振动(±4 m/s² 振荡)
- 期望：低振动→None，高振动→Severe + dominant_freq > 0
- 证据：smoke_test 断言通过

## S3：MpcVibrationController 抽象求解器（C++ smoke）
- 输入：FakeMpcSolver，振动 None→不激活，Severe→激活+阻尼>0
- 期望：ApplyCorrection 后 12 关节扭矩非零
- 证据：smoke_test 断言通过

## S4：GratingMetricsRecorder PPT 阈值（C++ smoke）
- 输入：100 步 / 5 异常 / 0.85 m/s / 2000m+
- 期望：anomaly_rate=5% ≤ 15%，avg_speed≥0.8，distance≥2000，ThresholdsPass=True
- 证据：smoke_test 断言通过

## S5：Python grating_metrics 阈值评估
- 输入：多种 run 场景（pass / 高异常 / 低速 / 短距离 / 多 run 混合）
- 期望：thresholds_pass() 正确判定，summary() 可读
- 证据：test_grating_metrics_1_5.py 全通过

## S6：高频线程独立性（ROS 环境验证）
- 输入：tech_1_5_node 启动，Python supervisor 延迟注入
- 期望：C++ 控制环 500Hz 不中断，Python 不阻塞
- 证据：ros2 launch 截图 + 频率统计

## S7：launch 装配 + Topic 命名
- 输入：ros2 launch inspection_bringup tech_1_5.launch.py
- 期望：tech_1_5_node + grating_motion_supervisor_node 均启动，/grating_status 可见
- 证据：ros2 node list / ros2 topic list 截图

## S8：实机钢格网巡检闭环（最终指标）
- 输入：钢格网场景实机试跑 2km
- 期望：异常率 -85%、均速 ≥0.8 m/s、单次 2km 全通过
- 证据：现场记录与统计表按本场景归档。
