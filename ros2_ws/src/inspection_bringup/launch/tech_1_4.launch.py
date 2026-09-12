from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="inspection_execution_cpp",
                executable="tech_1_4_node",
                name="tech_1_4_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="narrow_corridor_path_planner_node",
                name="narrow_corridor_path_planner_node",
                output="screen",
            ),
        ]
    )
