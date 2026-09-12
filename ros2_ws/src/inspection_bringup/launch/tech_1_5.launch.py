"""tech_1_5 启动：C++ 高频控制节点 + Python 任务级监督节点

（技术 1.5 钢格网主动抑振）：
- C++ tech_1_5_node：500/1000Hz 高频控制环（足接触估计→振动估计→MPC修正→关节命令）
- Python grating_motion_supervisor_node：任务级监督，不进入高频控制环
- 硬边界：高频控制循环在 C++ 独立线程，Python/HMI 不阻塞 MPC 回路
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tech_1_5_cpp = Node(
        package="inspection_execution_cpp",
        executable="tech_1_5_node",
        name="tech_1_5_node",
        output="screen",
        parameters=[
            {"control_freq_hz": 500.0},
            {"target_speed": 0.8},
            {"grating_mode": False},
        ],
    )

    grating_supervisor = Node(
        package="inspection_planning_py",
        executable="grating_motion_supervisor_node",
        name="grating_motion_supervisor_node",
        output="screen",
        parameters=[
            {"target_speed": 0.8},
            {"grating_mode": False},
            {"run_distance_target": 2000.0},
        ],
    )

    return LaunchDescription([tech_1_5_cpp, grating_supervisor])
