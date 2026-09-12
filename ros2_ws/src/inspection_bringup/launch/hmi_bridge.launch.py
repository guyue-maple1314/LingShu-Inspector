# 依赖第三方 rosbridge_server 包：`sudo apt install ros-$ROS_DISTRO-rosbridge-server`
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[{"port": 9090}],
            ),
        ]
    )
