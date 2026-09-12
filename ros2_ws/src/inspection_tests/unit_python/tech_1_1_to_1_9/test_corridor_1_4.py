import unittest

from inspection_planning_py.tech_1_4.corridor_metrics import CorridorMetrics
from inspection_planning_py.tech_1_4.corridor_plan_validator import validate_corridor
from inspection_planning_py.tech_1_4.narrow_corridor_path_planner import plan_path


def _corridor(**overrides):
    corridor = {
        "width": 0.75,
        "centerline": [{"x": 0.0, "y": 0.0}, {"x": 0.1, "y": 1.0}],
        "visual_validation_passed": True,
        "confidence": 0.9,
    }
    corridor.update(overrides)
    return corridor


class TestCorridorPlanValidator(unittest.TestCase):
    def test_valid(self):
        ok, _ = validate_corridor(_corridor())
        self.assertTrue(ok)

    def test_too_narrow(self):
        ok, _ = validate_corridor(_corridor(width=0.5))
        self.assertFalse(ok)

    def test_visual_fail(self):
        ok, _ = validate_corridor(_corridor(visual_validation_passed=False))
        self.assertFalse(ok)


class TestNarrowCorridorPathPlanner(unittest.TestCase):
    def test_plan_from_centerline(self):
        path = plan_path(_corridor())
        self.assertEqual(len(path), 2)
        self.assertEqual(path[0], (0.0, 0.0))

    def test_invalid_returns_empty(self):
        self.assertEqual(plan_path(_corridor(width=0.4)), [])


class TestCorridorMetrics(unittest.TestCase):
    def test_pass_rate(self):
        metrics = CorridorMetrics()
        metrics.record_trial(True)
        metrics.record_trial(True)
        metrics.record_trial(False)
        self.assertAlmostEqual(metrics.pass_rate(), 2.0 / 3.0)


if __name__ == "__main__":
    unittest.main()
