import unittest

from inspection_planning_py.tech_1_2.preemption_scheduler import decide


def _task(tid, benefit, risk, urgency):
    return {"task_id": tid, "benefit": benefit, "risk": risk, "urgency": urgency}


class TestPreemptionScheduler(unittest.TestCase):
    def test_charge_when_low_battery(self):
        d = decide([_task("a", 10, 0, 0)], "a", battery_percent=2.0, return_distance_m=100.0)
        self.assertEqual(d.decision, "CHARGE")

    def test_execute_highest(self):
        d = decide([_task("a", 1, 0, 0), _task("b", 10, 0, 0)], None, 100.0, 0.0)
        self.assertEqual(d.decision, "EXECUTE")
        self.assertEqual(d.current_task_id, "b")

    def test_preempt(self):
        d = decide([_task("a", 1, 0, 0), _task("b", 10, 0, 0)], "a", 100.0, 0.0)
        self.assertEqual(d.decision, "PREEMPT")
        self.assertEqual(d.current_task_id, "b")

    def test_wait(self):
        d = decide([], None, 100.0, 0.0)
        self.assertEqual(d.decision, "WAIT")

    def test_resume(self):
        d = decide([], None, 100.0, 0.0, resumable_task_id="saved")
        self.assertEqual(d.decision, "RESUME")


if __name__ == "__main__":
    unittest.main()
