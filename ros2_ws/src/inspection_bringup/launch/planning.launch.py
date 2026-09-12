from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # llm_backend 默认 deepseek；未设置 DEEPSEEK_API_KEY 时节点自动回退 stub。
    return LaunchDescription(
        [
            Node(
                package="inspection_planning_py",
                executable="py_heartbeat_node",
                name="py_heartbeat_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="llm_online_decision_node",
                name="llm_online_decision_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="goal_replanner_node",
                name="goal_replanner_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="candidate_task_pool_node",
                name="candidate_task_pool_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="energy_constraint_node",
                name="energy_constraint_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="preemption_scheduler_node",
                name="preemption_scheduler_node",
                output="screen",
            ),
        ]
    )
