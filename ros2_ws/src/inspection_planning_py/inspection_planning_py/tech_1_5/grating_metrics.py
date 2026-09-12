"""钢格网巡检指标评估。

PPT 指标（acceptance_metrics.md）：
- 步态异常率降低 85% 以上 → 目标异常率 ≤ 15%
- 平均速度 ≥ 0.8 m/s
- 单次完成 2 km 巡检 → ≥ 2000 m

本模块仅做任务级统计，不进入高频控制环。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


# PPT 阈值
PPT_ANOMALY_RATE_TARGET = 0.15  # ≤15%（即降低 85%）
PPT_MIN_AVG_SPEED = 0.8        # ≥0.8 m/s
PPT_MIN_DISTANCE = 2000.0      # ≥2 km = 2000 m


@dataclass
class GratingRunSample:
    """单次钢格网运行采样。"""
    timestamp_ns: int = 0
    anomaly_count: int = 0
    total_steps: int = 0
    avg_speed: float = 0.0
    distance: float = 0.0
    max_vibration_rms: float = 0.0
    resonance_detected: bool = False


@dataclass
class GratingRunResult:
    """单次钢格网运行评估结果。"""
    run_id: str = ""
    has_data: bool = False        # 是否拿到实测步数/速度/里程；false 时不做阈值判定
    anomaly_rate: float = 0.0
    avg_speed: float = 0.0
    distance: float = 0.0
    max_vibration_rms: float = 0.0
    anomaly_rate_pass: bool = False
    avg_speed_pass: bool = False
    distance_pass: bool = False

    @property
    def all_pass(self) -> bool:
        return (self.has_data and self.anomaly_rate_pass
                and self.avg_speed_pass and self.distance_pass)

    def summary(self) -> str:
        if not self.has_data:
            return (f"[NO DATA] {self.run_id}: 未收到实测步数/速度/里程，"
                    f"不做阈值判定（不虚构指标）")
        status = "PASS" if self.all_pass else "FAIL"
        return (
            f"[{status}] {self.run_id}: "
            f"anomaly_rate={self.anomaly_rate:.1%} "
            f"(target ≤{PPT_ANOMALY_RATE_TARGET:.0%}) "
            f"avg_speed={self.avg_speed:.2f} m/s "
            f"(target ≥{PPT_MIN_AVG_SPEED}) "
            f"distance={self.distance:.0f} m "
            f"(target ≥{PPT_MIN_DISTANCE:.0f} m) "
            f"max_vib={self.max_vibration_rms:.2f}"
        )


class GratingMetrics:
    """钢格网巡检指标聚合器。

    累积多次运行采样，评估 PPT 阈值是否达标。
    """

    def __init__(self) -> None:
        self._runs: List[GratingRunResult] = []
        self._current: Optional[GratingRunSample] = None

    def start_run(self, run_id: str = "") -> None:
        """开始一次新运行。"""
        self._current = GratingRunSample()
        self._current_run_id = run_id or f"run_{len(self._runs) + 1}"

    def record_sample(self, sample: GratingRunSample) -> None:
        """记录一次采样（来自 C++ 节点的 /grating_status 实测反馈）。"""
        if self._current is None:
            self.start_run()
        # 取最大值/累加
        self._current.anomaly_count += sample.anomaly_count
        self._current.total_steps += sample.total_steps
        self._current.distance = max(self._current.distance, sample.distance)
        self._current.max_vibration_rms = max(
            self._current.max_vibration_rms, sample.max_vibration_rms
        )
        self._current.resonance_detected = (
            self._current.resonance_detected or sample.resonance_detected
        )
        # avg_speed 取最新值
        if sample.avg_speed > 0:
            self._current.avg_speed = sample.avg_speed
        self._current.timestamp_ns = sample.timestamp_ns

    def finish_run(self) -> GratingRunResult:
        """结束当前运行并评估。"""
        if self._current is None:
            return GratingRunResult()

        s = self._current
        has_data = s.total_steps > 0
        anomaly_rate = (
            s.anomaly_count / s.total_steps if has_data else 0.0
        )

        result = GratingRunResult(
            run_id=self._current_run_id,
            has_data=has_data,
            anomaly_rate=anomaly_rate,
            avg_speed=s.avg_speed,
            distance=s.distance,
            max_vibration_rms=s.max_vibration_rms,
            anomaly_rate_pass=has_data and anomaly_rate <= PPT_ANOMALY_RATE_TARGET,
            avg_speed_pass=has_data and s.avg_speed >= PPT_MIN_AVG_SPEED,
            distance_pass=has_data and s.distance >= PPT_MIN_DISTANCE,
        )
        self._runs.append(result)
        self._current = None
        return result

    @property
    def runs(self) -> List[GratingRunResult]:
        return list(self._runs)

    def summary_text(self) -> str:
        """汇总所有运行。"""
        if not self._runs:
            return "No runs recorded."
        lines = [r.summary() for r in self._runs]
        total = len(self._runs)
        passed = sum(1 for r in self._runs if r.all_pass)
        lines.append(f"--- {passed}/{total} runs passed PPT thresholds ---")
        return "\n".join(lines)

    def thresholds_pass(self) -> bool:
        """所有运行是否全部通过 PPT 阈值。"""
        return bool(self._runs) and all(r.all_pass for r in self._runs)
