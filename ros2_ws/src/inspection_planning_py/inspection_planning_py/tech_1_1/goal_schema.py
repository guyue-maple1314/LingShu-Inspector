"""1.1 大模型允许输出的结构化目标字段与类型。"""

from dataclasses import dataclass
from typing import Any, Dict

ALLOWED_TASK_TYPES = ("inspection", "abnormal", "collection")
REQUIRED_FIELDS = ("goal_id", "task_type", "target_pose", "constraints", "valid_until_sec")


@dataclass
class GoalDraft:
    goal_id: str
    task_type: str
    target_pose: Dict[str, float]
    constraints: str
    valid_until_sec: float


def validate_goal_schema(goal: Any) -> tuple[bool, str]:
    """校验大模型输出是否满足结构化目标 schema。"""
    if not isinstance(goal, dict):
        return False, "goal must be a dict"

    missing = [f for f in REQUIRED_FIELDS if f not in goal]
    if missing:
        return False, f"missing fields: {missing}"

    goal_id = goal.get("goal_id")
    if not isinstance(goal_id, str) or not goal_id.strip():
        return False, "goal_id must be a non-empty string"

    task_type = goal.get("task_type")
    if task_type not in ALLOWED_TASK_TYPES:
        return False, f"task_type must be one of {ALLOWED_TASK_TYPES}"

    target_pose = goal.get("target_pose")
    if not isinstance(target_pose, dict):
        return False, "target_pose must be a dict"
    for axis in ("x", "y", "z"):
        if axis not in target_pose or not isinstance(target_pose[axis], (int, float)):
            return False, f"target_pose must contain numeric {axis}"

    if not isinstance(goal.get("constraints"), str):
        return False, "constraints must be a string"

    valid_until_sec = goal.get("valid_until_sec")
    if not isinstance(valid_until_sec, (int, float)):
        return False, "valid_until_sec must be numeric"

    return True, ""


def to_goal_draft(goal: Dict[str, Any]) -> GoalDraft:
    ok, err = validate_goal_schema(goal)
    if not ok:
        raise ValueError(err)
    return GoalDraft(
        goal_id=goal["goal_id"],
        task_type=goal["task_type"],
        target_pose={k: float(goal["target_pose"][k]) for k in ("x", "y", "z")},
        constraints=goal["constraints"],
        valid_until_sec=float(goal["valid_until_sec"]),
    )
