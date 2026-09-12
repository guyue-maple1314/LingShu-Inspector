"""1.7 语义位置指标评估。

PPT 验收指标（acceptance_metrics.md L358）：
- 检测事件与物理位置厘米级精准映射
- 故障位置报告准确率 ≥ 98%

本模块仅做任务级统计，不进入 C++ 高频定位/匹配环。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

# PPT 阈值
PPT_MAPPING_ACCURACY_M = 0.05  # 厘米级映射（5cm 栅格分辨率）
PPT_LOCATION_REPORT_ACCURACY = 0.98  # 位置报告准确率 ≥ 98%


@dataclass
class LocationMappingSample:
    """单次检测事件—位置映射采样。"""
    event_id: str = ""
    matched_x: float = 0.0
    matched_y: float = 0.0
    matched_z: float = 0.0
    bim_id: str = ""
    slam_id: str = ""
    coordinate_source: str = "unlocalized"  # bim/slam/cloud/unlocalized
    confidence: float = 0.0
    localized: bool = False
    # 参考真值（离线评估时注入，在线时为 0）
    ref_x: float = 0.0
    ref_y: float = 0.0
    ref_z: float = 0.0
    ref_localized: bool = True  # 真值是否应被定位


@dataclass
class LocationMappingRunResult:
    """单次语义位置映射运行评估结果。"""
    run_id: str = ""
    sample_count: int = 0
    localized_count: int = 0
    unlocalized_count: int = 0
    mapping_error_max_m: float = 0.0
    mapping_error_mean_m: float = 0.0
    # 位置报告准确率：正确定位/未定位的样本比例
    #   - 真值应定位且成功定位 → 正确
    #   - 真值不应定位且未定位 → 正确
    #   - 其他 → 错误
    location_report_accuracy: float = 0.0
    mapping_accuracy_pass: bool = False  # 最大映射误差 ≤ 5cm
    report_accuracy_pass: bool = False    # 位置报告准确率 ≥ 98%

    @property
    def all_pass(self) -> bool:
        return self.mapping_accuracy_pass and self.report_accuracy_pass

    def summary(self) -> str:
        status = "PASS" if self.all_pass else "FAIL"
        return (
            f"[{status}] {self.run_id}: "
            f"samples={self.sample_count} "
            f"localized={self.localized_count} "
            f"unlocalized={self.unlocalized_count} "
            f"map_err_max={self.mapping_error_max_m * 100:.2f}cm "
            f"(target ≤{PPT_MAPPING_ACCURACY_M * 100:.1f}cm) "
            f"report_acc={self.location_report_accuracy:.2%} "
            f"(target ≥{PPT_LOCATION_REPORT_ACCURACY:.0%})"
        )


class SemanticLocationMetrics:
    """语义位置指标聚合器。

    累积多次映射采样，评估 PPT 阈值是否达标。
    """

    def __init__(self) -> None:
        self._samples: List[LocationMappingSample] = []
        self._runs: List[LocationMappingRunResult] = []
        self._run_id: str = ""

    def start_run(self, run_id: str = "") -> None:
        self._samples.clear()
        if run_id:
            self._run_id = run_id
        else:
            self._run_id = f"run_{len(self._runs) + 1}"

    def record_sample(self, sample: LocationMappingSample) -> None:
        self._samples.append(sample)

    def finish_run(self) -> LocationMappingRunResult:
        if not self._samples:
            result = LocationMappingRunResult(run_id=self._run_id)
            self._runs.append(result)
            return result

        errors: List[float] = []
        localized = 0
        unlocalized = 0
        correct_reports = 0

        for s in self._samples:
            if s.localized:
                localized += 1
            else:
                unlocalized += 1

            # 位置报告准确率
            # 真值应定位 + 实际定位 → 正确
            # 真值不应定位 + 实际未定位 → 正确
            if s.ref_localized and s.localized:
                correct_reports += 1
            elif not s.ref_localized and not s.localized:
                correct_reports += 1

            # 映射误差（仅真值应定位且实际已定位时计算）
            if s.ref_localized and s.localized:
                if s.ref_x != 0.0 or s.ref_y != 0.0 or s.ref_z != 0.0:
                    dx = s.matched_x - s.ref_x
                    dy = s.matched_y - s.ref_y
                    dz = s.matched_z - s.ref_z
                    err = (dx * dx + dy * dy + dz * dz) ** 0.5
                    errors.append(err)

        total = len(self._samples)
        err_max = max(errors) if errors else 0.0
        err_mean = sum(errors) / len(errors) if errors else 0.0
        report_acc = correct_reports / total if total else 0.0

        result = LocationMappingRunResult(
            run_id=self._run_id,
            sample_count=total,
            localized_count=localized,
            unlocalized_count=unlocalized,
            mapping_error_max_m=err_max,
            mapping_error_mean_m=err_mean,
            location_report_accuracy=report_acc,
            mapping_accuracy_pass=err_max <= PPT_MAPPING_ACCURACY_M,
            report_accuracy_pass=report_acc >= PPT_LOCATION_REPORT_ACCURACY,
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
