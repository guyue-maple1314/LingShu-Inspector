# tech_1_2 动态任务抢占与恢复（Python 调度侧）

- candidate_task_pool_node：汇聚巡检 / 异常 / 采集任务
- task_value_evaluator：风险 / 收益 / 紧急度评估
- energy_constraint_evaluator：电量 / 返航距离 / 进度约束
- preemption_scheduler_node：执行 / 抢占 / 恢复 / 充电决定
- scheduler_metrics：任务成功率与抢占响应时间

## 安全电量策略（自动返航）

- battery_safety_policy：30% 固定基准、动态返航阈值与新任务准入（纯逻辑，不依赖 rclpy）
- energy_constraint_node：发布 EnergyConstraint，含预计返航耗电、30% 基准与动态安全阈值
- preemption_scheduler / preemption_scheduler_node：CHARGE 分支接入安全电量策略
- 参数文件：`config/battery_safety.yaml`，launch 默认参数与之一致

规则：

1. 安全电量基准 `base_safe_soc = 30%`；
2. 动态阈值 `safe_soc = max(30%, 预计返航耗电 + 10pp)`；
3. `SOC <= safe_soc`：保存当前任务进度（C++ task_lifecycle_executor 随 Action 取消保存）、
   停止启动新普通巡检/采集任务、进入返航充电调度（TaskDecision = CHARGE）；
4. 新任务准入：`SOC - 预计任务耗电 - 预计返航耗电 >= 10pp`，否则不启动（无在执行任务时
   决策 WAIT；高价值任务抢占不满足准入时保持当前任务）；
5. 30% 是任务调度层临时阈值，不是 BMS 硬件欠压保护值，实机标定完成前不得调低
   （policy_status = provisional_until_real_robot_calibration）。

边界与示例：

| SOC | 预计返航耗电 | safe_soc | 结果 |
|---|---|---|---|
| 25% | 0% | 30% | CHARGE 返航 |
| 30% | 0% | 30% | CHARGE（等号触发） |
| 33% | 25% | 35% | CHARGE（动态阈值） |
| 80% | 20%，任务5% | 30% | 准入通过（余量55pp），EXECUTE |
| 36% | 20%，任务20% | 30% | 不返航但准入不足（余量-4pp），WAIT |

数据链：预计任务耗电随 `/task`（`Task.estimated_consumption`）进入调度器，
预计返航耗电随 `/energy_constraint`（`EnergyConstraint.estimated_return_consumption`）进入调度器；
两者缺一即退化为只按返航余量判断，不额外虚构数值。

测试：

- 单元：`unit_python/tech_1_1_to_1_9/test_battery_safety_policy.py`（21 例，Windows unittest 可跑）
- 集成：`integration/test_low_battery_return.py`（需 ROS 2，pytest + rclpy）
