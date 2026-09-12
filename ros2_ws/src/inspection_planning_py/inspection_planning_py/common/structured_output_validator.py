"""Python 端结构化目标预校验；最终执行权仍属于 C++ 行为树。"""

REQUIRED_GOAL_FIELDS = {"goal_id", "task_type", "target_pose", "valid_until"}


def validate_goal_schema(goal: dict) -> tuple[bool, str]:
    if not isinstance(goal, dict):
        return False, "goal must be a dict"
    missing = REQUIRED_GOAL_FIELDS.difference(goal.keys())
    if missing:
        return False, f"missing fields: {sorted(missing)}"
    if not goal.get("goal_id"):
        return False, "goal_id must not be empty"
    if not goal.get("task_type"):
        return False, "task_type must not be empty"
    return True, ""
