# 技术 1.1 验收（大模型—行为树—PPO 双环协同）

## 必测场景

- 状态变化、设备告警、语音/文字指令触发结构化目标
- 异常触发后目标更新与任务切换

## PPT 指标

- 结构化指令生成成功率 ≥95%
- 异常任务切换响应时间 ≤500ms

## 证据

- 指令样本集、时间戳日志（C++ `AbnormalSwitchMonitor`）、任务执行记录
- 运行：`ros2 launch inspection_bringup tech_1_1.launch.py`
