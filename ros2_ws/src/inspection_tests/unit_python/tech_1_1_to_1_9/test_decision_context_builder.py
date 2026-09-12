import unittest

from inspection_planning_py.tech_1_1.decision_context_builder import build_decision_context


class _RobotState:
    battery_level = 80.0
    current_gait = "trot"
    execution_state = "idle"


class TestDecisionContextBuilder(unittest.TestCase):
    def test_empty(self):
        ctx = build_decision_context()
        self.assertEqual(ctx["alerts"], [])
        self.assertEqual(ctx["robot_state"]["battery_level"], None)

    def test_object_and_dict(self):
        ctx = build_decision_context(
            robot_state=_RobotState(),
            instruction={"source": "text", "raw_content": "hi"},
        )
        self.assertEqual(ctx["robot_state"]["battery_level"], 80.0)
        self.assertEqual(ctx["instruction"]["raw_content"], "hi")


if __name__ == "__main__":
    unittest.main()
