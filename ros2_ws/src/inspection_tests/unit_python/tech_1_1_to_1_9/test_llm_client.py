import json
import unittest

from inspection_planning_py.tech_1_1.llm_client import (
    StubLlmClient,
    build_goal_prompt,
    parse_goal_response,
)


def _goal(**overrides):
    goal = {
        "goal_id": "g1",
        "task_type": "inspection",
        "target_pose": {"x": 1.0, "y": 2.0, "z": 0.0},
        "constraints": "",
        "valid_until_sec": 100.0,
    }
    goal.update(overrides)
    return goal


class TestLlmClient(unittest.TestCase):
    def test_build_goal_prompt(self):
        system, user = build_goal_prompt(
            {"instruction": {"raw_content": "去巡检"}, "alerts": [], "robot_state": {}}
        )
        self.assertIn("JSON", system)
        data = json.loads(user)
        self.assertEqual(data["instruction"]["raw_content"], "去巡检")

    def test_parse_plain_json(self):
        goal = parse_goal_response(json.dumps(_goal()))
        self.assertIsNotNone(goal)
        self.assertEqual(goal["goal_id"], "g1")

    def test_parse_fenced_json(self):
        raw = "```json\n" + json.dumps(_goal(task_type="abnormal")) + "\n```"
        goal = parse_goal_response(raw)
        self.assertIsNotNone(goal)
        self.assertEqual(goal["task_type"], "abnormal")

    def test_parse_invalid(self):
        self.assertIsNone(parse_goal_response("not json"))
        self.assertIsNone(parse_goal_response(json.dumps({"goal_id": "x"})))

    def test_stub_client(self):
        goal = StubLlmClient().generate_goal(
            {"instruction": {"raw_content": "hello"}, "alerts": []}
        )
        self.assertEqual(goal["task_type"], "inspection")


if __name__ == "__main__":
    unittest.main()
