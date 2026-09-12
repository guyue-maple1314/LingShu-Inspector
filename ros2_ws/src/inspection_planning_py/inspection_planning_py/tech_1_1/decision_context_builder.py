"""1.1 只整理 PPT 指定的多源状态，构建决策上下文。"""

from typing import Any, Dict, List, Optional


def build_decision_context(
    robot_state: Any = None,
    alerts: Optional[List[Any]] = None,
    instruction: Any = None,
) -> Dict[str, Any]:
    return {
        "robot_state": {
            "battery_level": _field(robot_state, "battery_level"),
            "current_gait": _field(robot_state, "current_gait"),
            "execution_state": _field(robot_state, "execution_state"),
        },
        "alerts": [
            {
                "alert_type": _field(a, "alert_type"),
                "detected_value": _field(a, "detected_value"),
                "source_device": _field(a, "source_device"),
            }
            for a in (alerts or [])
        ],
        "instruction": {
            "source": _field(instruction, "source"),
            "raw_content": _field(instruction, "raw_content"),
        },
    }


def _field(obj: Any, name: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, dict):
        return obj.get(name)
    return getattr(obj, name, None)
