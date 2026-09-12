from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="inspection_execution_cpp",
                executable="tech_1_9_node",
                name="tech_1_9_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="adaptive_deep_filter_node",
                name="adaptive_deep_filter_node",
                output="screen",
            ),
            Node(
                package="inspection_planning_py",
                executable="acoustic_fault_diagnosis_node",
                name="acoustic_fault_diagnosis_node",
                output="screen",
            ),
        ]
    )
