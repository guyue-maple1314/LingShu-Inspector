"""1.5 钢格网监督节点的节点级逻辑单测（离线，无 ROS 依赖）。

覆盖点：
- valid=false 的状态（无实测数据）不进入指标聚合；
- C++ 侧累计值转增量，不能重复累加；
- 结束运行时只用实测值判定 PPT 阈值；
- 全程无数据时输出 NO DATA，不把"无数据"算成达标。
"""

import unittest
from types import SimpleNamespace

from inspection_planning_py.tech_1_5.grating_motion_supervisor_node import (
    GratingMotionSupervisorNode,
)


def _status(sec=1, nanosec=0, valid=True, total_steps=0, anomaly_count=0,
            avg_speed=0.0, distance_m=0.0, max_vibration_rms=0.0,
            resonance_detected=False):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        valid=valid,
        total_steps=total_steps,
        anomaly_count=anomaly_count,
        avg_speed=avg_speed,
        distance_m=distance_m,
        max_vibration_rms=max_vibration_rms,
        resonance_detected=resonance_detected,
    )


class TestNoDataHandling(unittest.TestCase):
    def test_invalid_status_is_not_recorded(self):
        node = GratingMotionSupervisorNode()
        node._on_status(_status(valid=False, total_steps=999, avg_speed=0.9))
        self.assertEqual(node._no_data_count, 1)
        self.assertEqual(node._metrics._current.total_steps, 0)
        self.assertEqual(node._metrics._current.avg_speed, 0.0)

    def test_finish_without_data_reports_no_data(self):
        node = GratingMotionSupervisorNode()
        node.finish_and_report()
        result = node._metrics.runs[0]
        self.assertFalse(result.has_data)
        self.assertFalse(result.all_pass)
        self.assertIn("NO DATA", result.summary())


class TestCumulativeToDelta(unittest.TestCase):
    def test_deltas_not_double_counted(self):
        node = GratingMotionSupervisorNode()
        # C++ 侧给累计值：100 步/5 异常 → 200 步/10 异常
        node._on_status(_status(total_steps=100, anomaly_count=5,
                                avg_speed=0.9, distance_m=1000.0))
        node._on_status(_status(total_steps=200, anomaly_count=10,
                                avg_speed=0.9, distance_m=2100.0))
        cur = node._metrics._current
        self.assertEqual(cur.total_steps, 200)      # 而不是 300
        self.assertEqual(cur.anomaly_count, 10)     # 而不是 15
        self.assertEqual(cur.distance, 2100.0)      # 累计距离取最大值
        self.assertAlmostEqual(cur.avg_speed, 0.9, places=6)

    def test_counter_reset_does_not_produce_negative_delta(self):
        node = GratingMotionSupervisorNode()
        node._on_status(_status(total_steps=200, anomaly_count=10))
        node._on_status(_status(total_steps=50, anomaly_count=2))  # 节点重启
        self.assertEqual(node._metrics._current.total_steps, 200)


class TestThresholdEvaluation(unittest.TestCase):
    def test_pass_run_uses_measured_values(self):
        node = GratingMotionSupervisorNode()
        node._on_status(_status(total_steps=100, anomaly_count=5,
                                avg_speed=0.85, distance_m=2100.0))
        node.finish_and_report()
        result = node._metrics.runs[0]
        self.assertTrue(result.has_data)
        self.assertAlmostEqual(result.anomaly_rate, 0.05, places=6)
        self.assertTrue(result.anomaly_rate_pass)
        self.assertTrue(result.avg_speed_pass)
        self.assertTrue(result.distance_pass)
        self.assertTrue(result.all_pass)

    def test_fail_run_and_resonance_flag(self):
        node = GratingMotionSupervisorNode()
        node._on_status(_status(total_steps=100, anomaly_count=25,
                                avg_speed=0.5, distance_m=500.0,
                                resonance_detected=True))
        node.finish_and_report()
        result = node._metrics.runs[0]
        self.assertFalse(result.anomaly_rate_pass)
        self.assertFalse(result.avg_speed_pass)
        self.assertFalse(result.distance_pass)
        self.assertFalse(result.all_pass)
        self.assertTrue(node._metrics._current is None)


if __name__ == "__main__":
    unittest.main()
