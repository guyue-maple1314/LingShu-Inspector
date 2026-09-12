"""tech_1_3 启动：在线运行时节点 tech_1_3_node（单进程装配 loader/builder/validator/adapter）

- C++ 在线侧：tech_1_3_node（rclcpp 可执行，由 CMake 注册）
- Python 离线侧：纯模块，不注册 ROS 节点，不在本 launch 启动

policy_dir 默认为空 → 不加载策略权重，不虚构推理结果；
导出模型后可通过 launch 传入：policy_dir:=/path/to/exported_policy_dir
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 节点名统一使用 C++ common/node_names.hpp ::kTech1_3Node 对应字符串
    tech_1_3_cpp = Node(
        package="inspection_execution_cpp",
        executable="tech_1_3_node",
        name="tech_1_3_node",
        output="screen",
        parameters=[
            {"policy_dir": ""},  # 默认空：不加载权重
        ],
    )
    return LaunchDescription([tech_1_3_cpp])
