# 技术 1.2 验收（动态任务抢占与恢复）

## 必测场景

- 多任务并发、突发高优先级任务插入、原任务恢复、低电量返航/充电

## PPT 指标

- 多任务执行成功率 92.3%
- 高优先级任务抢占响应 ≤500ms

## 证据

- 调度日志（`TaskDecision`）、Action 反馈、进度恢复记录（`TaskProgressStore`）
- 运行：`ros2 launch inspection_bringup tech_1_2.launch.py`
