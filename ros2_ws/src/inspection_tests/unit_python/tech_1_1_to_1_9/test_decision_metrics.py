import unittest

from inspection_planning_py.tech_1_1.decision_metrics import DecisionMetrics
from inspection_planning_py.tech_1_2.scheduler_metrics import SchedulerMetrics


class TestDecisionMetrics(unittest.TestCase):
    def test_success_rate_and_p95(self):
        m = DecisionMetrics()
        m.record(True, 100.0)
        m.record(False, 200.0)
        m.record(True, 150.0)
        self.assertAlmostEqual(m.success_rate(), 2.0 / 3.0)
        # 线性插值 p95：sorted=[100,150,200] -> 0.1*150 + 0.9*200 = 195
        self.assertEqual(m.p95_ms(), 195.0)

    def test_empty(self):
        m = DecisionMetrics()
        self.assertEqual(m.success_rate(), 0.0)
        self.assertEqual(m.p95_ms(), 0.0)


class TestSchedulerMetrics(unittest.TestCase):
    def test_task_success_rate(self):
        m = SchedulerMetrics()
        m.record_task(True)
        m.record_task(False)
        m.record_task(True)
        self.assertAlmostEqual(m.task_success_rate(), 2.0 / 3.0)

    def test_max_preemption(self):
        m = SchedulerMetrics()
        m.record_preemption(300.0)
        m.record_preemption(100.0)
        self.assertEqual(m.max_preemption_ms(), 300.0)


if __name__ == "__main__":
    unittest.main()
