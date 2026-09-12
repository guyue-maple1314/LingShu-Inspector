"""tech_1_3 多场景运动智能迁移（Python 离线训练侧）

全部为纯模块 / 脚本，不注册 ROS 节点，不连接实机关节执行接口。
- terrain_parameterization：地形参数化（钢格网/坡道/楼梯/窄通道）
- terrain_randomizer：地形随机生成与边界条件
- ppo_training_pipeline：抽象 Trainer 接口 + PPO 训练管线（训练框架未批准，不锁 PyTorch/sb3）
- sim_to_real_randomization：Sim-to-Real 域随机化（摩擦/负载/噪声/延时）
- policy_exporter：导出供 C++ 运行时加载的策略及元数据
- transfer_metrics：通行率（>90%）与实机迁移成功率（96.8%）统计
"""
