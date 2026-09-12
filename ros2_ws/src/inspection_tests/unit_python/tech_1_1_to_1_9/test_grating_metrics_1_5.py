"""tech_1_5 钢格网主动抑振 Python 侧单元测试（标准库 unittest）。"""

import unittest

from inspection_planning_py.tech_1_5.grating_metrics import (
    GratingMetrics,
    GratingRunSample,
    PPT_ANOMALY_RATE_TARGET,
    PPT_MIN_AVG_SPEED,
    PPT_MIN_DISTANCE,
)


class GratingMetricsTestCase(unittest.TestCase):
    def _make_sample(self, **kwargs):
        defaults = dict(
            timestamp_ns=0,
            anomaly_count=0,
            total_steps=100,
            avg_speed=0.85,
            distance=100.0,
            max_vibration_rms=2.0,
            resonance_detected=False,
        )
        defaults.update(kwargs)
        return GratingRunSample(**defaults)

    def test_ppt_thresholds(self):
        self.assertEqual(PPT_ANOMALY_RATE_TARGET, 0.15)
        self.assertEqual(PPT_MIN_AVG_SPEED, 0.8)
        self.assertEqual(PPT_MIN_DISTANCE, 2000.0)

    def test_single_run_pass(self):
        m = GratingMetrics()
        m.start_run("pass_run")
        m.record_sample(self._make_sample(
            anomaly_count=5, total_steps=100, avg_speed=0.9, distance=2100.0
        ))
        result = m.finish_run()
        self.assertTrue(result.anomaly_rate_pass)
        self.assertTrue(result.avg_speed_pass)
        self.assertTrue(result.distance_pass)
        self.assertTrue(result.all_pass)
        self.assertIn("PASS", result.summary())

    def test_single_run_fail_high_anomaly(self):
        m = GratingMetrics()
        m.start_run("fail_anomaly")
        m.record_sample(self._make_sample(
            anomaly_count=20, total_steps=100, avg_speed=0.9, distance=2100.0
        ))
        result = m.finish_run()
        self.assertFalse(result.anomaly_rate_pass)  # 20% > 15%
        self.assertTrue(result.avg_speed_pass)
        self.assertTrue(result.distance_pass)
        self.assertFalse(result.all_pass)
        self.assertIn("FAIL", result.summary())

    def test_single_run_fail_low_speed(self):
        m = GratingMetrics()
        m.start_run("fail_speed")
        m.record_sample(self._make_sample(
            anomaly_count=5, total_steps=100, avg_speed=0.6, distance=2100.0
        ))
        result = m.finish_run()
        self.assertTrue(result.anomaly_rate_pass)
        self.assertFalse(result.avg_speed_pass)  # 0.6 < 0.8
        self.assertFalse(result.all_pass)

    def test_single_run_fail_short_distance(self):
        m = GratingMetrics()
        m.start_run("fail_dist")
        m.record_sample(self._make_sample(
            anomaly_count=5, total_steps=100, avg_speed=0.9, distance=500.0
        ))
        result = m.finish_run()
        self.assertFalse(result.distance_pass)  # 500 < 2000
        self.assertFalse(result.all_pass)

    def test_multiple_runs_thresholds(self):
        m = GratingMetrics()
        m.start_run("run1")
        m.record_sample(self._make_sample(
            anomaly_count=5, total_steps=100, avg_speed=0.85, distance=2100.0
        ))
        m.finish_run()
        m.start_run("run2")
        m.record_sample(self._make_sample(
            anomaly_count=10, total_steps=100, avg_speed=0.8, distance=2000.0
        ))
        m.finish_run()
        self.assertEqual(len(m.runs), 2)
        self.assertTrue(m.thresholds_pass())

    def test_multiple_runs_one_fails(self):
        m = GratingMetrics()
        m.start_run("pass")
        m.record_sample(self._make_sample(
            anomaly_count=5, total_steps=100, avg_speed=0.85, distance=2100.0
        ))
        m.finish_run()
        m.start_run("fail")
        m.record_sample(self._make_sample(
            anomaly_count=30, total_steps=100, avg_speed=0.5, distance=500.0
        ))
        m.finish_run()
        self.assertFalse(m.thresholds_pass())

    def test_no_runs(self):
        m = GratingMetrics()
        self.assertFalse(m.thresholds_pass())
        self.assertEqual(m.summary_text(), "No runs recorded.")

    def test_accumulated_samples(self):
        m = GratingMetrics()
        m.start_run("acc")
        for i in range(10):
            m.record_sample(self._make_sample(
                anomaly_count=1, total_steps=10,
                avg_speed=0.85, distance=100.0 * (i + 1),
                max_vibration_rms=3.0 + i * 0.1,
            ))
        result = m.finish_run()
        # 累积：anomaly=10, steps=100, distance=1000, vib=3.9
        self.assertAlmostEqual(result.anomaly_rate, 0.10, places=2)
        self.assertEqual(result.distance, 1000.0)
        self.assertAlmostEqual(result.max_vibration_rms, 3.9, places=1)

    def test_resonance_detected(self):
        m = GratingMetrics()
        m.start_run("resonance")
        m.record_sample(self._make_sample(
            resonance_detected=True,
            anomaly_count=5, total_steps=100, avg_speed=0.85, distance=2100.0,
        ))
        result = m.finish_run()
        self.assertTrue(result.all_pass)  # resonance 不影响阈值通过


if __name__ == "__main__":
    unittest.main()
