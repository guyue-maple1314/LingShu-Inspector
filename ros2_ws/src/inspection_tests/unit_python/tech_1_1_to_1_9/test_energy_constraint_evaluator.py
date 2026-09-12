import unittest

from inspection_planning_py.tech_1_2.energy_constraint_evaluator import evaluate_energy


class TestEnergyConstraintEvaluator(unittest.TestCase):
    def test_feasible(self):
        r = evaluate_energy(100.0, 100.0, 20.0)
        self.assertTrue(r.feasible)

    def test_infeasible(self):
        r = evaluate_energy(20.0, 100.0, 20.0)
        self.assertFalse(r.feasible)
        self.assertLess(r.remaining_after_task, 5.0)


if __name__ == "__main__":
    unittest.main()
