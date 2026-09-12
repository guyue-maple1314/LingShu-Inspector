"""1.7 BIM 先验与实时 SLAM 关联决策。

PPT 范围：关联先验 BIM 和实时 SLAM 结果，为检测事件做语义位置映射。

输入：
- BIM 构件列表（来自 bim_loader 抽象接口）
- SLAM 观测（来自 1.6 FusionPose 周边，简化为 landmark 点列表）

输出：
- 关联结果：每个 SLAM landmark 对应的 BIM 构件（或"未关联"）

红线：
- 无法关联时标记 "unassociated"，不虚构 BIM 对应关系
- BIM 缺失（未装载）时返回全 "unassociated"
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .bim_loader import BimElement


@dataclass
class SlamLandmark:
    """SLAM 实时观测点（简化结构）。"""
    id: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    confidence: float = 0.5


@dataclass
class AlignmentResult:
    """单次 BIM-SLAM 关联结果。"""
    slam_id: str = ""
    bim_id: str = ""              # 关联到的 BIM 构件（空表示未关联）
    bim_x: float = 0.0
    bim_y: float = 0.0
    bim_z: float = 0.0
    slam_x: float = 0.0
    slam_y: float = 0.0
    slam_z: float = 0.0
    distance_m: float = 0.0      # 关联距离
    associated: bool = False      # 是否成功关联

    @property
    def coordinate_source(self) -> str:
        """坐标来源标识（与 C++ PositionMatchResult.coordinate_source 对齐）。"""
        return "bim" if self.associated and self.bim_id else "slam"


class BimSlamAlignment:
    """BIM 先验与实时 SLAM 关联决策器。

    匹配策略：
    1. 对每个 SLAM landmark，在 BIM 构件中查找距离最近者
    2. 距离 < association_radius_m 且 BIM 构件存在 → 关联
    3. 否则标记 "unassociated"（不虚构对应关系）
    """

    def __init__(self, association_radius_m: float = 0.30) -> None:
        # 30cm 关联半径（与 C++ position_matcher 默认搜索半径一致）
        self._radius: float = association_radius_m if association_radius_m > 0 else 0.30

    def associate(self, bim_elements: List[BimElement],
                  slam_landmarks: List[SlamLandmark]) -> List[AlignmentResult]:
        """关联 BIM 与 SLAM。"""
        results: List[AlignmentResult] = []

        for lm in slam_landmarks:
            result = AlignmentResult(
                slam_id=lm.id,
                bim_id="",
                slam_x=lm.x,
                slam_y=lm.y,
                slam_z=lm.z,
            )

            # BIM 缺失 → 全部未关联（不虚构）
            if not bim_elements:
                results.append(result)
                continue

            # 最近邻匹配
            best_bim: Optional[BimElement] = None
            best_dist = float("inf")
            for bim in bim_elements:
                dx = bim.center_x - lm.x
                dy = bim.center_y - lm.y
                dz = bim.center_z - lm.z
                d = (dx * dx + dy * dy + dz * dz) ** 0.5
                if d < best_dist:
                    best_dist = d
                    best_bim = bim

            if best_bim is not None and best_dist <= self._radius:
                result.bim_id = best_bim.id
                result.bim_x = best_bim.center_x
                result.bim_y = best_bim.center_y
                result.bim_z = best_bim.center_z
                result.distance_m = best_dist
                result.associated = True
            # 否则保持未关联（不虚构）

            results.append(result)

        return results

    def summary(self, results: List[AlignmentResult]) -> str:
        """关联统计摘要。"""
        if not results:
            return "No SLAM landmarks to align."
        associated = sum(1 for r in results if r.associated)
        total = len(results)
        rate = associated / total if total else 0.0
        return (
            f"BIM-SLAM alignment: {associated}/{total} associated "
            f"({rate:.1%}), radius={self._radius * 100:.0f}cm"
        )
