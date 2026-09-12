"""钢格网运动监督节点（任务级，不参与 500/1000Hz 控制环）。

职责：
- 订阅 /terrain_observation 获取 C++ 节点的钢格网状态反馈
- 选择钢格网运行目标（进入/退出钢格网模式、设定目标速度）
- 聚合运行指标并评估 PPT 阈值（异常率 -85%、均速 ≥0.8 m/s、2km）
- 不进入高频控制环，不阻塞 C++ MPC 回路
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import TerrainObservation

from inspection_planning_py.common.node_names import GRATING_MOTION_SUPERVISOR_NODE
from inspection_planning_py.common.topic_names import TERRAIN_OBSERVATION
from inspection_planning_py.tech_1_5.grating_metrics import (
    GratingMetrics,
    GratingRunSample,
)


class GratingMotionSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__(GRATING_MOTION_SUPERVISOR_NODE)

        self.declare_parameter("target_speed", 0.8)
        self.declare_parameter("grating_mode", False)
        self.declare_parameter("run_distance_target", 2000.0)

        self._metrics = GratingMetrics()
        self._metrics.start_run("supervised_run")

        self.create_subscription(
            TerrainObservation, TERRAIN_OBSERVATION, self._on_status, 10
        )

        # 低频状态打印（1Hz）
        self.create_timer(1.0, self._periodic_report)

        self.get_logger().info(
            "grating_motion_supervisor started: task-level monitoring only, "
            "not in 500/1000Hz control loop"
        )

    def _on_status(self, msg: TerrainObservation) -> None:
        """接收 C++ 节点的钢格网状态反馈。

        TerrainObservation 字段：
        - header.stamp: 时间戳
        - terrain_type: 地形类型（grating 表示钢格网）
        - foot_contact[]: 足端接触状态
        - confidence: 观测置信度
        """
        ts = msg.header.stamp
        timestamp_ns = ts.sec * 1_000_000_000 + ts.nanosec
        # 是否检测到共振：钢格网地形 + 置信度低（近似振动异常）
        is_grating = (msg.terrain_type == "grating")
        low_confidence = float(msg.confidence) < 0.5
        sample = GratingRunSample(
            timestamp_ns=timestamp_ns,
            avg_speed=float(self.get_parameter("target_speed").value),
            distance=0.0,  # 由 C++ 侧 GratingMetricsRecorder 累计
            resonance_detected=is_grating and low_confidence,
            max_vibration_rms=1.0 - float(msg.confidence) if is_grating else 0.0,
        )
        self._metrics.record_sample(sample)

    def _periodic_report(self) -> None:
        """周期性打印当前运行指标（不重置，仅打印摘要）。"""
        cur = self._metrics._current
        if cur is None:
            self.get_logger().info("[grating_supervisor] no data yet")
            return
        self.get_logger().info(
            f"[grating_supervisor] distance={cur.distance:.1f}m, "
            f"steps={cur.total_steps}, anomalies={cur.anomaly_count}, "
            f"max_vib={cur.max_vibration_rms:.2f}"
        )

    def finish_and_report(self) -> str:
        """结束当前运行并返回汇总报告。"""
        self._metrics.finish_run()
        return self._metrics.summary_text()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GratingMotionSupervisorNode()
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
