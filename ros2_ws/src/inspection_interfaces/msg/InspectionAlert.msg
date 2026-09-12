# 设备告警（1.1、1.7、1.8、1.9）
std_msgs/Header header
string alert_type
float64 detected_value
string source_device
geometry_msgs/Pose detection_pose
bool pose_valid        # detection_pose 是否为有效测量位姿；false 时不得据此做位置匹配
