from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# qdgwzyjsxy-lszx

def generate_launch_description():
    behavior_tree_xml = PathJoinSubstitution(
        [FindPackageShare("inspection_execution_cpp"), "behavior_trees", "main_inspection_tree.xml"]
    )

    return LaunchDescription(
        [
            Node(
                package="inspection_execution_cpp",
                executable="cpp_heartbeat_node",
                name="cpp_heartbeat_node",
                output="screen",
            ),
            Node(
                package="inspection_execution_cpp",
                executable="tech_1_1_node",
                name="tech_1_1_node",
                output="screen",
                parameters=[{"behavior_tree_xml": behavior_tree_xml}],
            ),
            Node(
                package="inspection_execution_cpp",
                executable="tech_1_2_node",
                name="tech_1_2_node",
                output="screen",
            ),
        ]
    )
