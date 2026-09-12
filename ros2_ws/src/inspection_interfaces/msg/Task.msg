# 统一任务建模（1.2）
std_msgs/Header header
string task_id
string task_type        # inspection / abnormal / collection
string status
float64 risk
float64 benefit
float64 urgency
float64 progress        # 0.0..1.0
float64 estimated_consumption   # 预计任务耗电（%）
