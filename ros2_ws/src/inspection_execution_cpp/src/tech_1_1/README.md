# tech_1_1 大模型—行为树—PPO 双环协同（C++ 执行侧）

- bt_task_executor_node：BehaviorTree.CPP 调度、校验、执行与切换
- goal_safety_validator：结构化目标与安全约束校验
- ppo_runtime_controller：PPO 在线推理，输出连续步态控制量
- abnormal_switch_monitor：记录异常切换耗时（500 ms 验收）

