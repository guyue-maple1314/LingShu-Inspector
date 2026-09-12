# 调度结果（1.2）
std_msgs/Header header
string current_task_id
string[] ranked_task_ids
string decision          # EXECUTE / PREEMPT / RESUME / CHARGE / WAIT
builtin_interfaces/Time decision_time
