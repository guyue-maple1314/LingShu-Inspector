# 钢格网抑振状态与实测指标（1.5）
std_msgs/Header header
string terrain_type          # solid / grating
bool grating_mode_active     # 钢格网模式是否激活
bool valid                   # 是否已有实测数据；false 时下列指标无意义，不得据此评估
bool passable                # 当前是否可通过（共振 / 误接触时 false）
float64 confidence           # 观测置信度 0..1
float64 accel_rms            # 实测振动加速度 RMS（m/s²）
float64 max_vibration_rms    # 本次运行最大振动 RMS
bool resonance_detected      # 是否检测到共振
uint64 total_steps           # 累计步数
uint64 anomaly_count         # 累计异常步数
float64 anomaly_rate         # 异常率 0..1
float64 avg_speed            # 实测平均速度（m/s）
float64 distance_m           # 累计巡检距离（m）
