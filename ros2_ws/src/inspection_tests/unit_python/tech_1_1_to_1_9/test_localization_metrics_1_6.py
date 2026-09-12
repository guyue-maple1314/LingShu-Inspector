"""tech_1_6 弱纹理融合导航 Python 侧单元测试（标准库 unittest）。"""

import unittest

from inspection_planning_py.tech_1_6.localization_metrics import (
    LocalizationMetrics,
    LocalizationSample,
    PPT_POSITION_ACCURACY_M,
    PPT_DEGRADED_MAINTAIN_RATIO,
)
from inspection_planning_py.tech_1_6.localization_status_evaluator import (
    LocalizationStatusEvaluator,
)


class LocalizationMetricsTestCase(unittest.TestCase):
    def _make_sample(self, **kwargs):
        defaults = dict(
            timestamp_ns=0,
            position_x=1.0,
            position_y=2.0,
            position_z=0.0,
            localization_state="healthy",
            valid_sources=["lidar", "camera_left", "camera_right", "imu", "foot_force"],
            ref_x=1.0,
            ref_y=2.0,
            ref_z=0.0,
        )
        defaults.update(kwargs)
        return LocalizationSample(**defaults)

    def test_ppt_thresholds(self):
        self.assertEqual(PPT_POSITION_ACCURACY_M, 0.02)  # ±2cm
        self.assertEqual(PPT_DEGRADED_MAINTAIN_RATIO, 0.9)

    def test_accuracy_pass(self):
        m = LocalizationMetrics()
        m.start_run("pass")
        # 误差 1cm < 2cm
        m.record_sample(self._make_sample(position_x=1.01, ref_x=1.0))
        result = m.finish_run()
        self.assertTrue(result.accuracy_pass)
        self.assertLess(result.position_error_max_m, 0.02)
        self.assertTrue(result.all_pass)

    def test_accuracy_fail(self):
        m = LocalizationMetrics()
        m.start_run("fail")
        # 误差 5cm > 2cm
        m.record_sample(self._make_sample(position_x=1.05, ref_x=1.0))
        result = m.finish_run()
        self.assertFalse(result.accuracy_pass)
        self.assertGreater(result.position_error_max_m, 0.02)

    def test_degraded_maintain(self):
        m = LocalizationMetrics()
        m.start_run("degraded_ok")
        # 10 次退化，0 次丢失 → 维持率 100%
        for _ in range(10):
            m.record_sample(self._make_sample(localization_state="degraded"))
        result = m.finish_run()
        self.assertTrue(result.degraded_maintain_pass)
        self.assertEqual(result.degraded_ratio, 1.0)
        self.assertEqual(result.lost_ratio, 0.0)

    def test_degraded_lost_too_much(self):
        m = LocalizationMetrics()
        m.start_run("degraded_bad")
        # 5 次退化 + 5 次丢失 → 维持率 50% < 90%
        for _ in range(5):
            m.record_sample(self._make_sample(localization_state="degraded"))
        for _ in range(5):
            m.record_sample(self._make_sample(localization_state="lost"))
        result = m.finish_run()
        self.assertFalse(result.degraded_maintain_pass)

    def test_no_degradation(self):
        """无退化发生时，维持率视为 1.0。"""
        m = LocalizationMetrics()
        m.start_run("all_healthy")
        for _ in range(10):
            m.record_sample(self._make_sample(localization_state="healthy"))
        result = m.finish_run()
        self.assertTrue(result.degraded_maintain_pass)
        self.assertEqual(result.healthy_ratio, 1.0)

    def test_empty_run(self):
        m = LocalizationMetrics()
        m.start_run("empty")
        result = m.finish_run()
        self.assertEqual(result.sample_count, 0)

    def test_multiple_samples_error_stats(self):
        m = LocalizationMetrics()
        m.start_run("multi")
        m.record_sample(self._make_sample(position_x=1.005, ref_x=1.0))  # 0.5cm
        m.record_sample(self._make_sample(position_x=1.015, ref_x=1.0))  # 1.5cm
        m.record_sample(self._make_sample(position_x=1.001, ref_x=1.0))  # 0.1cm
        result = m.finish_run()
        self.assertAlmostEqual(result.position_error_max_m, 0.015, places=3)
        self.assertGreater(result.position_error_mean_m, 0.0)


class LocalizationStatusEvaluatorTestCase(unittest.TestCase):
    def test_healthy_state(self):
        ev = LocalizationStatusEvaluator()
        snap = ev.evaluate("healthy", ["lidar", "imu", "foot_force"])
        self.assertTrue(snap.is_healthy)
        self.assertFalse(snap.is_degraded)
        self.assertFalse(snap.lidar_failed)
        self.assertEqual(snap.valid_source_count, 3)

    def test_degraded_state_lidar_failed(self):
        ev = LocalizationStatusEvaluator()
        # 激光失效（不在有效源中）
        snap = ev.evaluate("degraded", ["camera_left", "imu", "foot_force"])
        self.assertTrue(snap.is_degraded)
        self.assertTrue(snap.lidar_failed)
        self.assertEqual(snap.valid_source_count, 3)

    def test_lost_state(self):
        ev = LocalizationStatusEvaluator()
        snap = ev.evaluate("lost", ["imu"])
        self.assertTrue(snap.is_lost)
        self.assertTrue(snap.lidar_failed)
        self.assertEqual(snap.valid_source_count, 1)

    def test_source_coverage(self):
        ev = LocalizationStatusEvaluator()
        snap = ev.evaluate("healthy", ["lidar", "imu", "foot_force"])
        # 3/5 = 60%
        self.assertAlmostEqual(snap.source_coverage, 0.6, places=1)

    def test_full_source_coverage(self):
        ev = LocalizationStatusEvaluator()
        all_src = ["lidar", "camera_left", "camera_right", "imu", "foot_force"]
        snap = ev.evaluate("healthy", all_src)
        self.assertAlmostEqual(snap.source_coverage, 1.0, places=1)

    def test_ratios(self):
        ev = LocalizationStatusEvaluator()
        for _ in range(6):
            ev.evaluate("healthy", ["lidar", "imu"])
        for _ in range(3):
            ev.evaluate("degraded", ["imu"])
        for _ in range(1):
            ev.evaluate("lost", [])
        self.assertAlmostEqual(ev.healthy_ratio(), 0.6, places=1)
        self.assertAlmostEqual(ev.degraded_ratio(), 0.3, places=1)
        self.assertAlmostEqual(ev.lost_ratio(), 0.1, places=1)
        self.assertEqual(ev.sample_count(), 10)

    def test_lidar_failure_ratio(self):
        ev = LocalizationStatusEvaluator()
        # 5 次激光有效
        for _ in range(5):
            ev.evaluate("healthy", ["lidar", "imu"])
        # 5 次激光失效
        for _ in range(5):
            ev.evaluate("degraded", ["imu"])
        self.assertAlmostEqual(ev.lidar_failure_ratio(), 0.5, places=1)

    def test_reset(self):
        ev = LocalizationStatusEvaluator()
        ev.evaluate("healthy", ["lidar"])
        self.assertEqual(ev.sample_count(), 1)
        ev.reset()
        self.assertEqual(ev.sample_count(), 0)


if __name__ == "__main__":
    unittest.main()
