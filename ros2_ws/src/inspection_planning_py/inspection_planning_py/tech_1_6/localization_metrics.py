"""1.6 融合定位指标评估。

PPT 指标：
- 定位精度 ±2cm（0.02m）
- 退化条件下仍维持定位（不伪造激光有效状态）

本模块仅做任务级统计，不进入 C++ 高频定位环。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


# PPT 阈值
PPT_POSITION_ACCURACY_M = 0.02  # ±2cm
PPT_DEGRADED_MAINTAIN_RATIO = 0.9  # 退化时维持定位成功率 ≥ 90%


@dataclass
class LocalizationSample:
    """单次定位采样（来自 C++ 节点 /fusion_pose 反馈）。"""
    timestamp_ns: int = 0
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    localization_state: str = "unknown"  # healthy/degraded/lost
    valid_sources: List[str] = field(default_factory=list)
    # 参考真值（离线评估时注入，在线时为 0）
    ref_x: float = 0.0
    ref_y: float = 0.0
    ref_z: float = 0.0


@dataclass
class LocalizationRunResult:
    """单次融合定位运行评估结果。"""
    run_id: str = ""
    sample_count: int = 0
    position_error_max_m: float = 0.0
    position_error_mean_m: float = 0.0
    degraded_ratio: float = 0.0
    lost_ratio: float = 0.0
    healthy_ratio: float = 0.0
    accuracy_pass: bool = False      # 最大误差 ≤ 2cm
    degraded_maintain_pass: bool = False  # 退化时定位未丢失率 ≥ 90%

    @property
    def all_pass(self) -> bool:
        return self.accuracy_pass and self.degraded_maintain_pass

    def summary(self) -> str:
        status = "PASS" if self.all_pass else "FAIL"
        return (
            f"[{status}] {self.run_id}: "
            f"samples={self.sample_count} "
            f"err_max={self.position_error_max_m * 100:.2f}cm "
            f"(target ≤{PPT_POSITION_ACCURACY_M * 100:.1f}cm) "
            f"err_mean={self.position_error_mean_m * 100:.2f}cm "
            f"healthy={self.healthy_ratio:.1%} "
            f"degraded={self.degraded_ratio:.1%} "
            f"lost={self.lost_ratio:.1%}"
        )


class LocalizationMetrics:
    """融合定位指标聚合器。

    累积多次定位采样，评估 PPT 阈值是否达标。
    """

    def __init__(self) -> None:
        self._samples: List[LocalizationSample] = []
        self._runs: List[LocalizationRunResult] = []
        self._run_id: str = ""

    def start_run(self, run_id: str = "") -> None:
        """开始一次评估运行。"""
        self._samples.clear()
        if run_id:
            self._run_id = run_id
        else:
            self._run_id = f"run_{len(self._runs) + 1}"

    def record_sample(self, sample: LocalizationSample) -> None:
        """记录一次定位采样。"""
        self._samples.append(sample)

    def finish_run(self) -> LocalizationRunResult:
        """结束当前运行并评估。"""
        if not self._samples:
            result = LocalizationRunResult(run_id=self._run_id)
            self._runs.append(result)
            return result

        errors: List[float] = []
        degraded_count = 0
        lost_count = 0
        healthy_count = 0

        for s in self._samples:
            # 定位误差（仅在有参考真值时计算）
            if s.ref_x != 0.0 or s.ref_y != 0.0 or s.ref_z != 0.0:
                dx = s.position_x - s.ref_x
                dy = s.position_y - s.ref_y
                dz = s.position_z - s.ref_z
                err = (dx * dx + dy * dy + dz * dz) ** 0.5
                errors.append(err)

            if s.localization_state == "healthy":
                healthy_count += 1
            elif s.localization_state == "degraded":
                degraded_count += 1
            elif s.localization_state == "lost":
                lost_count += 1

        total = len(self._samples)
        err_max = max(errors) if errors else 0.0
        err_mean = sum(errors) / len(errors) if errors else 0.0

        # 退化维持率：退化状态下未丢失的比例
        degraded_total = degraded_count + lost_count
        if degraded_total > 0:
            degraded_maintain = degraded_count / degraded_total
        else:
            # 无退化发生，视为维持率 1.0
            degraded_maintain = 1.0

        result = LocalizationRunResult(
            run_id=self._run_id,
            sample_count=total,
            position_error_max_m=err_max,
            position_error_mean_m=err_mean,
            degraded_ratio=degraded_count / total if total else 0.0,
            lost_ratio=lost_count / total if total else 0.0,
            healthy_ratio=healthy_count / total if total else 0.0,
            accuracy_pass=err_max <= PPT_POSITION_ACCURACY_M,
            degraded_maintain_pass=degraded_maintain >= PPT_DEGRADED_MAINTAIN_RATIO,
        )
        self._runs.append(result)
        self._samples.clear()
        return result

    def summary_text(self) -> str:
        """汇总所有运行。"""
        if not self._runs:
            return "No runs recorded."
        lines = [r.summary() for r in self._runs]
        total = len(self._runs)
        passed = sum(1 for r in self._runs if r.all_pass)
        lines.append(f"--- {passed}/{total} runs passed PPT thresholds ---")
        return "\n".join(lines)
