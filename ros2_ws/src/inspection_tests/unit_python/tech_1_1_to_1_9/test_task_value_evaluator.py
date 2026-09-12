import unittest

from inspection_planning_py.tech_1_2.task_value_evaluator import TaskValueEvaluator, evaluate_value


class TestTaskValueEvaluator(unittest.TestCase):
    def test_default_weights(self):
        self.assertEqual(evaluate_value(10, 3, 5), 12.0)

    def test_weighted(self):
        ev = TaskValueEvaluator(benefit_weight=2.0, risk_weight=1.0, urgency_weight=0.5)
        self.assertEqual(ev.evaluate({"benefit": 10, "risk": 4, "urgency": 6}), 19.0)

    def test_higher_value_wins(self):
        ev = TaskValueEvaluator()
        low = ev.evaluate({"benefit": 1, "risk": 0, "urgency": 0})
        high = ev.evaluate({"benefit": 10, "risk": 0, "urgency": 0})
        self.assertGreater(high, low)


if __name__ == "__main__":
    unittest.main()
