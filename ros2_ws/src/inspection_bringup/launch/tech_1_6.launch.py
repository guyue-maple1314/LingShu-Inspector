"""tech_1_6 启动：C++ 融合定位节点 + Python 任务级导航规划节点

（技术 1.6 弱纹理环境高精度融合导航）：
- C++ tech_1_6_node：五组件装配（同步→约束→紧耦合定位→退化监控→导航执行）
- Python fusion_navigation_planner_node：任务级监督，订阅 /fusion_pose 评估 PPT 指标
- 硬边界：紧耦合定位在 C++ 节点内，Python/HMI 不阻塞定位回路
- 退化时不伪造激光有效状态（红线）
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tech_1_6_cpp = Node(
        package="inspection_execution_cpp",
        executable="tech_1_6_node",
        name="tech_1_6_node",
        output="screen",
        parameters=[
            {"sync_tolerance_ms": 5.0},
            {"control_period_ms": 100},
        ],
    )

    fusion_nav_planner = Node(
        package="inspection_planning_py",
        executable="fusion_navigation_planner_node",
        name="fusion_navigation_planner_node",
        output="screen",
        parameters=[
            {"accuracy_target_m": 0.02},
            {"report_period_sec": 1.0},
        ],
    )

    return LaunchDescription([tech_1_6_cpp, fusion_nav_planner])
