"""钢格网运动监督节点（任务级，不参与 500/1000Hz 控制环）。

职责：
- 订阅 /grating_status（GratingStatus）获取 C++ 节点的实测状态与指标
- 选择钢格网运行目标（进入/退出钢格网模式、设定目标速度）
- 聚合运行指标并评估 PPT 阈值（异常率 -85%、均速 ≥0.8 m/s、2km）
- 不进入高频控制环，不阻塞 C++ MPC 回路

红线：本节点只使用 C++ 侧实测字段；没有实测数据时不做阈值判定，
      既不虚构速度/距离，也不把"无数据"算成达标。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import GratingStatus

from inspection_planning_py.common.node_names import GRATING_MOTION_SUPERVISOR_NODE
from inspection_planning_py.common.topic_names import GRATING_STATUS
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
        self._last_total_steps = 0
        self._last_anomaly_count = 0
        self._no_data_count = 0

        self.create_subscription(
            GratingStatus, GRATING_STATUS, self._on_status, 10
        )

        # 低频状态打印（1Hz）
        self.create_timer(1.0, self._periodic_report)

        self.get_logger().info(
            "grating_motion_supervisor started: task-level monitoring only, "
            "not in 500/1000Hz control loop"
        )

    def _on_status(self, msg: GratingStatus) -> None:
        """接收 C++ 节点的实测状态与指标。

        GratingStatus 字段：
        - header.stamp: 时间戳
        - valid: 是否已有实测数据（false 时下列指标无意义）
        - terrain_type / grating_mode_active / passable / confidence
        - accel_rms / max_vibration_rms / resonance_detected
        - total_steps / anomaly_count / anomaly_rate / avg_speed / distance_m
        """
        ts = msg.header.stamp
        timestamp_ns = ts.sec * 1_000_000_000 + ts.nanosec

        # 红线：没有实测数据（未注入适配器 / 控制环未产生步数）时不记录、不评估
        if not msg.valid:
            self._no_data_count += 1
            return

        # C++ 侧给的是累计值：步数与异常数转为增量，距离取累计最大值
        total_steps = int(msg.total_steps)
        anomaly_count = int(msg.anomaly_count)
        delta_steps = max(0, total_steps - self._last_total_steps)
        delta_anomalies = max(0, anomaly_count - self._last_anomaly_count)
        self._last_total_steps = total_steps
        self._last_anomaly_count = anomaly_count

        sample = GratingRunSample(
            timestamp_ns=timestamp_ns,
            anomaly_count=delta_anomalies,
            total_steps=delta_steps,
            avg_speed=float(msg.avg_speed),
            distance=float(msg.distance_m),
            max_vibration_rms=float(msg.max_vibration_rms),
            resonance_detected=bool(msg.resonance_detected),
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
            f"avg_speed={cur.avg_speed:.2f}m/s, "
            f"max_vib={cur.max_vibration_rms:.2f}, "
            f"no_data_msgs={self._no_data_count}"
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
