"""tech_1_8 启动：C++ 动态红外测温节点 + Python 异常高温判定节点

（技术 1.8 抗振抑扰的动态红外精准测温）：
- C++ tech_1_8_node：五组件装配（同步→稳像→辐射率补偿→角度/距离补偿→范围校验）
- Python thermal_anomaly_decision_node：范围内异常高温判定 + PPT 指标聚合
- 硬边界：稳像与温度补偿在 C++，Python 只做异常高温判定；
  超 PPT 范围（±60°、1–5m、0.95–1.05）标"范围外"，不按范围内精度发布
  （不虚构温度/精度，红线）
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tech_1_8_cpp = Node(
        package="inspection_execution_cpp",
        executable="tech_1_8_node",
        name="tech_1_8_node",
        output="screen",
        parameters=[
            {"control_period_ms": 33},     # ≈30fps
            {"sync_tolerance_ms": 5.0},
            {"exposure_ms": 8.0},
            {"focal_factor": 400.0},
        ],
    )

    thermal_anomaly = Node(
        package="inspection_planning_py",
        executable="thermal_anomaly_decision_node",
        name="thermal_anomaly_decision_node",
        output="screen",
        parameters=[
            {"anomaly_threshold_c": 80.0},
        ],
    )

    return LaunchDescription([tech_1_8_cpp, thermal_anomaly])
