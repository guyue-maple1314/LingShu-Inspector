"""1.4 窄通道路径规划：基于有效 CorridorState 生成沿中心线的路径。"""

from typing import Any, List, Tuple

from .corridor_plan_validator import validate_corridor


def plan_path(
    corridor: Any,
    robot_width: float = 0.46,
    margin: float = 0.10,
) -> List[Tuple[float, float]]:
    ok, _ = validate_corridor(corridor, robot_width, margin)
    if not ok:
        return []
    path: List[Tuple[float, float]] = []
    for point in corridor.get("centerline") or []:
        if isinstance(point, dict):
            path.append((float(point.get("x", 0.0)), float(point.get("y", 0.0))))
        else:
            path.append((float(point[0]), float(point[1])))
    return path
