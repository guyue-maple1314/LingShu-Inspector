# 地形观测（1.3、1.5 的 PPO/MPC 输入）
std_msgs/Header header
string terrain_type
geometry_msgs/PoseWithCovariance pose
bool[] foot_contact
geometry_msgs/Twist velocity
float64 confidence
bool passable
