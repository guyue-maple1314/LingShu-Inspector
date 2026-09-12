"""1.8 动态红外测温指标评估。

PPT 验收指标（acceptance_metrics.md）：
- 范围内偏差 ≤ 0.2 ℃
- 动态精度 ± 0.5 ℃
- 效率提升 ≥ 10 倍

本模块仅做任务级统计，不进入 C++ 高频稳像/补偿环。
温度稳像与补偿在 C++ 执行，Python 不重复实现（红线）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

# PPT 阈值
PPT_STATIC_DEVIATION_C = 0.2    # 范围内偏差 ≤ 0.2 ℃
PPT_DYNAMIC_ACCURACY_C = 0.5    # 动态精度 ± 0.5 ℃
PPT_EFFICIENCY_GAIN = 10.0      # 效率提升 ≥ 10 倍


@dataclass
class ThermalMeasurementSample:
    """单次红外测温采样。"""
    raw_temperature: float = 0.0
    compensated_temperature: float = 0.0
    angle: float = 0.0
    distance: float = 0.0
    emissivity: float = 1.0
    correction_factor: float = 1.0
    error_state: str = "ok"          # "ok" / "xxx_out_of_range" / "invalid"
    in_range: bool = True             # error_state == "ok"
    # 参考真值（离线评估时注入，在线时为 0）
    ref_temperature: float = 0.0
    is_dynamic: bool = False         # 是否动态行进中（0.8 m/s）
    # 效率统计
    processing_fps: float = 0.0       # 本帧处理帧率
    baseline_fps: float = 0.0        # 基线帧率


@dataclass
class ThermalRunResult:
    """单次测温运行评估结果。"""
    run_id: str = ""
    sample_count: int = 0
    in_range_count: int = 0
    out_of_range_count: int = 0
    static_deviation_max_c: float = 0.0
    static_deviation_mean_c: float = 0.0
    dynamic_accuracy_max_c: float = 0.0
    efficiency_gain: float = 0.0
    static_pass: bool = False        # 范围内偏差 ≤ 0.2 ℃
    dynamic_pass: bool = False       # 动态精度 ≤ 0.5 ℃
    efficiency_pass: bool = False     # 效率 ≥ 10 倍

    @property
    def all_pass(self) -> bool:
        return self.static_pass and self.dynamic_pass and self.efficiency_pass

    def summary(self) -> str:
        status = "PASS" if self.all_pass else "FAIL"
        return (
            f"[{status}] {self.run_id}: "
            f"samples={self.sample_count} "
            f"in_range={self.in_range_count} "
            f"out_of_range={self.out_of_range_count} "
            f"static_dev_max={self.static_deviation_max_c:.3f}℃ "
            f"(target ≤{PPT_STATIC_DEVIATION_C}) "
            f"dyn_acc_max={self.dynamic_accuracy_max_c:.3f}℃ "
            f"(target ≤{PPT_DYNAMIC_ACCURACY_C}) "
            f"eff_gain={self.efficiency_gain:.1f}x "
            f"(target ≥{PPT_EFFICIENCY_GAIN})"
        )


class ThermalMetrics:
    """红外测温指标聚合器。

    累积多次测温采样，评估 PPT 阈值是否达标。

    红线：超范围样本不计入精度统计（只统计 in_range 的），
          不按范围内精度发布。
    """

    def __init__(self) -> None:
        self._samples: List[ThermalMeasurementSample] = []
        self._runs: List[ThermalRunResult] = []
        self._run_id: str = ""

    def start_run(self, run_id: str = "") -> None:
        self._samples.clear()
        if run_id:
            self._run_id = run_id
        else:
            self._run_id = f"run_{len(self._runs) + 1}"

    def record_sample(self, sample: ThermalMeasurementSample) -> None:
        self._samples.append(sample)

    def finish_run(self) -> ThermalRunResult:
        if not self._samples:
            result = ThermalRunResult(run_id=self._run_id)
            self._runs.append(result)
            return result

        in_range = 0
        out_of_range = 0
        static_errors: List[float] = []
        dynamic_errors: List[float] = []
        gains: List[float] = []

        for s in self._samples:
            if s.in_range:
                in_range += 1
                # 红线：仅范围内核算精度
                if s.ref_temperature != 0.0:
                    err = abs(s.compensated_temperature - s.ref_temperature)
                    if s.is_dynamic:
                        dynamic_errors.append(err)
                    else:
                        static_errors.append(err)
            else:
                out_of_range += 1

            # 效率统计（与 in_range 无关）
            if s.baseline_fps > 0.0 and s.processing_fps > 0.0:
                gains.append(s.processing_fps / s.baseline_fps)

        total = len(self._samples)
        static_max = max(static_errors) if static_errors else 0.0
        static_mean = (sum(static_errors) / len(static_errors)
                       if static_errors else 0.0)
        dyn_max = max(dynamic_errors) if dynamic_errors else 0.0
        eff_gain = (sum(gains) / len(gains)) if gains else 0.0

        result = ThermalRunResult(
            run_id=self._run_id,
            sample_count=total,
            in_range_count=in_range,
            out_of_range_count=out_of_range,
            static_deviation_max_c=static_max,
            static_deviation_mean_c=static_mean,
            dynamic_accuracy_max_c=dyn_max,
            efficiency_gain=eff_gain,
            static_pass=static_max <= PPT_STATIC_DEVIATION_C,
            dynamic_pass=dyn_max <= PPT_DYNAMIC_ACCURACY_C,
            efficiency_pass=eff_gain >= PPT_EFFICIENCY_GAIN,
        )
        self._runs.append(result)
        self._samples.clear()
        return result

    def summary_text(self) -> str:
        if not self._runs:
            return "No runs recorded."
        lines = [r.summary() for r in self._runs]
        total = len(self._runs)
        passed = sum(1 for r in self._runs if r.all_pass)
        lines.append(f"--- {passed}/{total} runs passed PPT thresholds ---")
        return "\n".join(lines)
