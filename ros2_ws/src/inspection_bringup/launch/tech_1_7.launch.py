"""tech_1_7 启动：C++ 语义地图节点 + Python 语义标注决策节点

（技术 1.7 高精度时空融合的语义地图构建与定位）：
- C++ tech_1_7_node：四组件装配（点云转换→栅格赋权→位置匹配→报警发布）
- Python semantic_annotation_node：BIM-SLAM 关联决策 + PPT 指标聚合
- 硬边界：BIM 走抽象接口（AbstractBimLoader）；无法对应时保持"未定位"
  （不虚构物理监测点，红线）；Python 不进入 C++ 高频匹配环
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tech_1_7_cpp = Node(
        package="inspection_execution_cpp",
        executable="tech_1_7_node",
        name="tech_1_7_node",
        output="screen",
        parameters=[
            {"grid_resolution_m": 0.05},
            {"search_radius_m": 0.30},
            {"map_origin_x": 0.0},
            {"map_origin_y": 0.0},
            {"map_size_x": 100},
            {"map_size_y": 100},
            {"control_period_ms": 200},
        ],
    )

    semantic_annotation = Node(
        package="inspection_planning_py",
        executable="semantic_annotation_node",
        name="semantic_annotation_node",
        output="screen",
        parameters=[
            {"report_period_sec": 1.0},
        ],
    )

    return LaunchDescription([tech_1_7_cpp, semantic_annotation])
