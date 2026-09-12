from launch import LaunchDescription
from launch_ros.actions import Node

# 参数默认值与 config/battery_safety.yaml 一致，可在此覆盖
_BATTERY_SAFETY_PARAMS = {
    "base_safe_soc_percent": 30,
    "return_reserve_percent_points": 10,
    "consumption_per_meter_percent": 0.01,
}


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="inspection_execution_cpp",
                executable="tech_1_2_node",
                name="tech_1_2_node",
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
                parameters=[_BATTERY_SAFETY_PARAMS],
            ),
            Node(
                package="inspection_planning_py",
                executable="preemption_scheduler_node",
                name="preemption_scheduler_node",
                output="screen",
                parameters=[
                    {
                        "base_safe_soc_percent": _BATTERY_SAFETY_PARAMS[
                            "base_safe_soc_percent"
                        ],
                        "return_reserve_percent_points": _BATTERY_SAFETY_PARAMS[
                            "return_reserve_percent_points"
                        ],
                    }
                ],
            ),
        ]
    )
