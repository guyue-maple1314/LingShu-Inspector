"""1.6 定位状态评估器。

读取 FusionPose 的 valid_sources 和 localization_state，
评估定位健康度、退化时长、有效数据源覆盖率。

不虚构定位结果：仅基于实际反馈数据统计（红线）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class LocalizationStatusSnapshot:
    """定位状态快照。"""
    localization_state: str = "unknown"  # healthy/degraded/lost
    valid_source_count: int = 0
    valid_sources: List[str] = field(default_factory=list)
    is_healthy: bool = False
    is_degraded: bool = False
    is_lost: bool = False
    # 退化告警：激光是否失效
    lidar_failed: bool = False
    # 有效源覆盖率（相对全源 5 个）
    source_coverage: float = 0.0


class LocalizationStatusEvaluator:
    """定位状态评估器。

    从 FusionPose 消息提取定位状态和有效数据源，
    评估健康度并检测退化条件。
    """

    # 全部数据源（用于覆盖率计算）
    ALL_SOURCES = frozenset(
        {"lidar", "camera_left", "camera_right", "imu", "foot_force"}
    )

    def __init__(self) -> None:
        self._snapshots: List[LocalizationStatusSnapshot] = []

    def evaluate(
        self,
        localization_state: str,
        valid_sources: List[str],
    ) -> LocalizationStatusSnapshot:
        """评估单次定位状态。"""
        snap = LocalizationStatusSnapshot()
        snap.localization_state = localization_state
        snap.valid_sources = list(valid_sources)
        snap.valid_source_count = len(valid_sources)
        snap.is_healthy = (localization_state == "healthy")
        snap.is_degraded = (localization_state == "degraded")
        snap.is_lost = (localization_state == "lost")

        # 激光是否失效（不在有效源中）
        snap.lidar_failed = "lidar" not in valid_sources

        # 有效源覆盖率
        total = len(self.ALL_SOURCES)
        valid = sum(
            1 for s in valid_sources if s in self.ALL_SOURCES
        )
        snap.source_coverage = valid / total if total > 0 else 0.0

        self._snapshots.append(snap)
        return snap

    def healthy_ratio(self) -> float:
        """健康状态比例。"""
        if not self._snapshots:
            return 0.0
        return sum(1 for s in self._snapshots if s.is_healthy) / len(
            self._snapshots
        )

    def degraded_ratio(self) -> float:
        """退化状态比例。"""
        if not self._snapshots:
            return 0.0
        return sum(1 for s in self._snapshots if s.is_degraded) / len(
            self._snapshots
        )

    def lost_ratio(self) -> float:
        """丢失状态比例。"""
        if not self._snapshots:
            return 0.0
        return sum(1 for s in self._snapshots if s.is_lost) / len(
            self._snapshots
        )

    def lidar_failure_ratio(self) -> float:
        """激光失效比例（钢格网楼梯感知盲区指标）。"""
        if not self._snapshots:
            return 0.0
        return sum(1 for s in self._snapshots if s.lidar_failed) / len(
            self._snapshots
        )

    def reset(self) -> None:
        """重置统计。"""
        self._snapshots.clear()

    def sample_count(self) -> int:
        return len(self._snapshots)
