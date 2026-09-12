"""1.4 通道规划校验：路径必须使用有效的融合结果。"""

from typing import Any, Tuple


def validate_corridor(
    corridor: Any,
    robot_width: float = 0.46,
    margin: float = 0.10,
) -> Tuple[bool, str]:
    if not isinstance(corridor, dict):
        return False, "corridor must be a dict"
    if not corridor.get("visual_validation_passed", False):
        return False, "visual validation not passed"
    width = corridor.get("width")
    if width is None or width < robot_width + margin:
        return False, f"corridor width {width} too narrow"
    return True, ""
