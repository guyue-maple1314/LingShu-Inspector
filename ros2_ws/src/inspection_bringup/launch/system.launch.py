from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = "inspection_bringup"

    execution = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(pkg), "launch", "execution.launch.py"])]
        )
    )
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(pkg), "launch", "planning.launch.py"])]
        )
    )
    hmi_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare(pkg), "launch", "hmi_bridge.launch.py"])]
        )
    )

    return LaunchDescription([execution, planning, hmi_bridge])
