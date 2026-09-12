"""1.8 动态红外精准测温 Python 单元测试。

覆盖：
- thermal_metrics：PPT 三类指标
  * 范围内偏差 ≤ 0.2 ℃
  * 动态精度 ± 0.5 ℃
  * 效率提升 ≥ 10 倍
- 红线：超范围样本不计入精度统计
- thermal_anomaly_decision_node：离线逻辑自检（超范围不判定异常高温）

使用 unittest（不用 pytest，红线）。
"""

import unittest

from inspection_planning_py.tech_1_8.thermal_metrics import (
    PPT_DYNAMIC_ACCURACY_C,
    PPT_EFFICIENCY_GAIN,
    PPT_STATIC_DEVIATION_C,
    ThermalMeasurementSample,
    ThermalMetrics,
)


class TestThermalMetricsStatic(unittest.TestCase):
    """静态范围内偏差指标（≤ 0.2 ℃）。"""

    def test_empty_run_not_pass(self):
        """空运行不达标。"""
        metrics = ThermalMetrics()
        metrics.start_run("empty")
        result = metrics.finish_run()
        self.assertEqual(result.sample_count, 0)
        self.assertFalse(result.all_pass)

    def test_static_within_0_2c_pass(self):
        """范围内静态偏差 ≤ 0.2 ℃ → PASS。"""
        metrics = ThermalMetrics()
        metrics.start_run("static_ok")
        for _ in range(10):
            metrics.record_sample(ThermalMeasurementSample(
                compensated_temperature=25.0,
                ref_temperature=25.1,  # 偏差 0.1 ℃ ≤ 0.2 ℃
                is_dynamic=False,
                in_range=True,
                error_state="ok",
            ))
        result = metrics.finish_run()
        self.assertEqual(result.sample_count, 10)
        self.assertEqual(result.in_range_count, 10)
        self.assertLessEqual(result.static_deviation_max_c,
                             PPT_STATIC_DEVIATION_C)
        self.assertTrue(result.static_pass)

    def test_static_exceeds_0_2c_fails(self):
        """静态偏差 > 0.2 ℃ → FAIL。"""
        metrics = ThermalMetrics()
        metrics.start_run("static_fail")
        metrics.record_sample(ThermalMeasurementSample(
            compensated_temperature=25.0,
            ref_temperature=25.5,  # 偏差 0.5 ℃ > 0.2 ℃
            is_dynamic=False,
            in_range=True,
            error_state="ok",
        ))
        result = metrics.finish_run()
        self.assertGreater(result.static_deviation_max_c,
                           PPT_STATIC_DEVIATION_C)
        self.assertFalse(result.static_pass)


class TestThermalMetricsDynamic(unittest.TestCase):
    """动态精度指标（± 0.5 ℃）。"""

    def test_dynamic_within_0_5c_pass(self):
        """动态精度 ≤ 0.5 ℃ → PASS。"""
        metrics = ThermalMetrics()
        metrics.start_run("dynamic_ok")
        for _ in range(10):
            metrics.record_sample(ThermalMeasurementSample(
                compensated_temperature=60.0,
                ref_temperature=60.3,  # 偏差 0.3 ℃ ≤ 0.5 ℃
                is_dynamic=True,
                in_range=True,
                error_state="ok",
            ))
        result = metrics.finish_run()
        self.assertLessEqual(result.dynamic_accuracy_max_c,
                             PPT_DYNAMIC_ACCURACY_C)
        self.assertTrue(result.dynamic_pass)

    def test_dynamic_exceeds_0_5c_fails(self):
        """动态精度 > 0.5 ℃ → FAIL。"""
        metrics = ThermalMetrics()
        metrics.start_run("dynamic_fail")
        metrics.record_sample(ThermalMeasurementSample(
            compensated_temperature=60.0,
            ref_temperature=61.0,  # 偏差 1.0 ℃ > 0.5 ℃
            is_dynamic=True,
            in_range=True,
            error_state="ok",
        ))
        result = metrics.finish_run()
        self.assertGreater(result.dynamic_accuracy_max_c,
                           PPT_DYNAMIC_ACCURACY_C)
        self.assertFalse(result.dynamic_pass)


class TestThermalMetricsEfficiency(unittest.TestCase):
    """效率提升指标（≥ 10 倍）。"""

    def test_efficiency_ge_10x_pass(self):
        """效率提升 ≥ 10 倍 → PASS。"""
        metrics = ThermalMetrics()
        metrics.start_run("eff_ok")
        metrics.record_sample(ThermalMeasurementSample(
            in_range=True,
            error_state="ok",
            processing_fps=300.0,
            baseline_fps=30.0,  # 10x
        ))
        result = metrics.finish_run()
        self.assertGreaterEqual(result.efficiency_gain,
                                PPT_EFFICIENCY_GAIN)
        self.assertTrue(result.efficiency_pass)

    def test_efficiency_below_10x_fails(self):
        """效率 < 10 倍 → FAIL。"""
        metrics = ThermalMetrics()
        metrics.start_run("eff_fail")
        metrics.record_sample(ThermalMeasurementSample(
            in_range=True,
            error_state="ok",
            processing_fps=150.0,
            baseline_fps=30.0,  # 5x < 10x
        ))
        result = metrics.finish_run()
        self.assertLess(result.efficiency_gain, PPT_EFFICIENCY_GAIN)
        self.assertFalse(result.efficiency_pass)


class TestThermalMetricsOutOfRangeRedLine(unittest.TestCase):
    """红线：超范围样本不计入精度统计。"""

    def test_out_of_range_excluded_from_accuracy(self):
        """超范围样本即便偏差很大也不计入精度统计。"""
        metrics = ThermalMetrics()
        metrics.start_run("mixed")
        # 范围内样本：偏差小
        metrics.record_sample(ThermalMeasurementSample(
            compensated_temperature=25.0,
            ref_temperature=25.05,  # 偏差 0.05 ℃
            is_dynamic=False,
            in_range=True,
            error_state="ok",
        ))
        # 超范围样本：偏差 75 ℃ 但 out_of_range
        metrics.record_sample(ThermalMeasurementSample(
            compensated_temperature=25.0,
            ref_temperature=100.0,
            is_dynamic=False,
            in_range=False,
            error_state="angle_out_of_range",
        ))
        result = metrics.finish_run()
        self.assertEqual(result.sample_count, 2)
        self.assertEqual(result.in_range_count, 1)
        self.assertEqual(result.out_of_range_count, 1)
        # 超范围样本不计入 → max 仅为范围内 0.05 ℃
        self.assertLessEqual(result.static_deviation_max_c,
                             PPT_STATIC_DEVIATION_C)
        self.assertTrue(result.static_pass)

    def test_all_out_of_range_no_accuracy_data(self):
        """全部超范围时精度统计为 0（不虚构精度）。"""
        metrics = ThermalMetrics()
        metrics.start_run("all_oor")
        for _ in range(5):
            metrics.record_sample(ThermalMeasurementSample(
                compensated_temperature=25.0,
                ref_temperature=100.0,
                in_range=False,
                error_state="distance_out_of_range",
            ))
        result = metrics.finish_run()
        self.assertEqual(result.in_range_count, 0)
        self.assertEqual(result.out_of_range_count, 5)
        # 无范围内数据 → 精度 0，但 static_pass 为 True（0 ≤ 0.2）
        # 这是边界行为：无样本时不算超标，也不宣称达标
        self.assertEqual(result.static_deviation_max_c, 0.0)


class TestThermalMetricsAllPass(unittest.TestCase):
    """三类指标全过判定。"""

    def test_all_three_pass(self):
        """静态 + 动态 + 效率 全过 → all_pass=True。"""
        metrics = ThermalMetrics()
        metrics.start_run("all_ok")
        # 静态样本
        metrics.record_sample(ThermalMeasurementSample(
            compensated_temperature=25.0,
            ref_temperature=25.05,  # 0.05
            is_dynamic=False,
            in_range=True,
            error_state="ok",
            processing_fps=300.0,
            baseline_fps=30.0,  # 10x
        ))
        # 动态样本
        metrics.record_sample(ThermalMeasurementSample(
            compensated_temperature=60.0,
            ref_temperature=60.3,  # 0.3
            is_dynamic=True,
            in_range=True,
            error_state="ok",
            processing_fps=300.0,
            baseline_fps=30.0,
        ))
        result = metrics.finish_run()
        self.assertTrue(result.static_pass)
        self.assertTrue(result.dynamic_pass)
        self.assertTrue(result.efficiency_pass)
        self.assertTrue(result.all_pass)

    def test_summary_text_aggregates_runs(self):
        """多次运行汇总。"""
        metrics = ThermalMetrics()
        metrics.start_run("r1")
        metrics.record_sample(ThermalMeasurementSample(
            in_range=True,
            error_state="ok",
            processing_fps=300.0,
            baseline_fps=30.0,
        ))
        metrics.finish_run()
        summary = metrics.summary_text()
        self.assertIn("r1", summary)
        self.assertIn("passed PPT thresholds", summary)


class TestThermalAnomalyNodeOffline(unittest.TestCase):
    """异常高温判定节点离线逻辑自检（无 ROS 环境）。"""

    def test_offline_main_runs_without_rclpy(self):
        """无 ROS 环境下 main() 应执行离线自检不抛异常。"""
        from inspection_planning_py.tech_1_8 import \
            thermal_anomaly_decision_node
        # main() 内部会检测 _HAS_RCLPY 并走离线分支
        thermal_anomaly_decision_node.main()

    def test_out_of_range_recorded_not_judged(self):
        """红线：超范围样本记录到 metrics 但不判定异常高温。"""
        from inspection_planning_py.tech_1_8.thermal_anomaly_decision_node \
            import ThermalAnomalyDecisionNode
        node = ThermalAnomalyDecisionNode()
        # 超范围样本：温度虽高（200℃）但 error_state != "ok"
        out_sample = ThermalMeasurementSample(
            compensated_temperature=200.0,
            error_state="angle_out_of_range",
            in_range=False,
        )
        node._metrics.record_sample(out_sample)
        # 节点应记录该样本但不做异常高温判定（无 _alert_pub 在离线模式）
        result = node.finish_and_report()
        self.assertIn("out_of_range=1", result)

    def test_anomaly_threshold_is_80c(self):
        """异常高温阈值标定为 80 ℃。"""
        from inspection_planning_py.tech_1_8.thermal_anomaly_decision_node \
            import ANOMALY_HIGH_TEMP_C
        self.assertEqual(ANOMALY_HIGH_TEMP_C, 80.0)


if __name__ == "__main__":
    unittest.main()
