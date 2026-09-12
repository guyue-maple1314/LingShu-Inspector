# tech_1_3 多场景运动智能迁移（C++ 实机运行时）

- ppo_policy_loader：加载训练导出的策略及元数据
- observation_builder：组装姿态 / 接触 / 速度观测
- policy_output_validator：校验策略输出维度 / 范围 / 时效
- locomotion_command_adapter：策略输出转本体 SDK 命令

