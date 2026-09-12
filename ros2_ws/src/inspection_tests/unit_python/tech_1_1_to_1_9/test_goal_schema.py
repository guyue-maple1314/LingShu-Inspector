import unittest

from inspection_planning_py.tech_1_1.goal_schema import to_goal_draft, validate_goal_schema


def _valid_goal():
    return {
        "goal_id": "g-1",
        "task_type": "inspection",
        "target_pose": {"x": 1.0, "y": 2.0, "z": 0.0},
        "constraints": "avoid wet floor",
        "valid_until_sec": 100.0,
    }


class TestGoalSchema(unittest.TestCase):
    def test_valid(self):
        ok, err = validate_goal_schema(_valid_goal())
        self.assertTrue(ok, err)

    def test_missing_field(self):
        goal = _valid_goal()
        del goal["goal_id"]
        ok, _ = validate_goal_schema(goal)
        self.assertFalse(ok)

    def test_bad_task_type(self):
        goal = _valid_goal()
        goal["task_type"] = "unknown"
        ok, _ = validate_goal_schema(goal)
        self.assertFalse(ok)

    def test_to_draft(self):
        draft = to_goal_draft(_valid_goal())
        self.assertEqual(draft.goal_id, "g-1")
        self.assertEqual(draft.target_pose["x"], 1.0)


if __name__ == "__main__":
    unittest.main()
