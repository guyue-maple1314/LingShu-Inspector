# 能耗约束（1.2）
std_msgs/Header header
float64 battery_level                  # 当前电量 SOC（%）
float64 return_distance                # 返航距离（m）
float64 estimated_consumption          # 预计任务耗电（%）
# 以下三字段见设计方案 §6.2
float64 estimated_return_consumption   # 预计返航耗电（%，返航距离×单位距离耗电）
float64 base_safe_soc_percent          # 30% 固定基准（调度层临时阈值，非 BMS 保护值）
float64 safe_soc                       # 动态安全阈值 = max(30%, 预计返航耗电 + 10pp)
