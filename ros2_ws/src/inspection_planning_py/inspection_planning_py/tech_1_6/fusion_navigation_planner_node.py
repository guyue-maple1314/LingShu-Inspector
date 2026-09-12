"""融合导航规划节点（任务级，不参与 C++ 高频定位环）。

职责：
- 订阅 /fusion_pose 获取 C++ 节点的紧耦合定位结果
- 使用 LocalizationStatusEvaluator 评估定位健康度、退化时长
- 使用 LocalizationMetrics 统计 PPT 指标（±2cm 精度）
- 退化时记录告警，丢失时标记安全暂停
- 不进入 C++ 高频定位环，不阻塞紧耦合回路

（技术 1.6 弱纹理环境高精度融合导航）。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import FusionPose

from inspection_planning_py.common.node_names import FUSION_NAVIGATION_PLANNER_NODE
from inspection_planning_py.common.topic_names import FUSION_POSE
from inspection_planning_py.tech_1_6.localization_metrics import (
    LocalizationMetrics,
    LocalizationSample,
)
from inspection_planning_py.tech_1_6.localization_status_evaluator import (
    LocalizationStatusEvaluator,
)


class FusionNavigationPlannerNode(Node):
    """融合导航规划节点。"""

    def __init__(self) -> None:
        super().__init__(FUSION_NAVIGATION_PLANNER_NODE)

        self.declare_parameter("accuracy_target_m", 0.02)  # PPT ±2cm
        self.declare_parameter("report_period_sec", 1.0)

        self._evaluator = LocalizationStatusEvaluator()
        self._metrics = LocalizationMetrics()
        self._metrics.start_run("fusion_nav_run")

        self.create_subscription(
            FusionPose, FUSION_POSE, self._on_fusion_pose, 10
        )

        # 低频状态打印
        period = float(self.get_parameter("report_period_sec").value)
        self.create_timer(period, self._periodic_report)

        self.get_logger().info(
            "fusion_navigation_planner started: task-level monitoring of "
            "/fusion_pose, PPT target ±2cm accuracy"
        )

    def _on_fusion_pose(self, msg: FusionPose) -> None:
        """接收 C++ 节点的融合定位结果。"""
        # 评估定位状态
        self._evaluator.evaluate(
            localization_state=str(msg.localization_state),
            valid_sources=list(msg.valid_sources),
        )

        # 记录指标采样
        sample = LocalizationSample(
            timestamp_ns=msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec,
            position_x=float(msg.pose.pose.position.x),
            position_y=float(msg.pose.pose.position.y),
            position_z=float(msg.pose.pose.position.z),
            localization_state=str(msg.localization_state),
            valid_sources=list(msg.valid_sources),
        )
        self._metrics.record_sample(sample)

        # 退化告警
        if msg.localization_state == "degraded":
            self.get_logger().warn(
                f"localization degraded: sources={list(msg.valid_sources)}"
            )
        elif msg.localization_state == "lost":
            self.get_logger().error(
                "localization LOST: navigation should pause safely"
            )

    def _periodic_report(self) -> None:
        """周期性打印定位健康度。"""
        self.get_logger().info(
            f"healthy={self._evaluator.healthy_ratio():.1%} "
            f"degraded={self._evaluator.degraded_ratio():.1%} "
            f"lost={self._evaluator.lost_ratio():.1%} "
            f"lidar_failure={self._evaluator.lidar_failure_ratio():.1%} "
            f"samples={self._evaluator.sample_count()}"
        )

    def finish_and_report(self) -> str:
        """结束当前运行并返回汇总报告。"""
        result = self._metrics.finish_run()
        return result.summary()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNavigationPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.finish_and_report()
        node.get_logger().info(report)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
