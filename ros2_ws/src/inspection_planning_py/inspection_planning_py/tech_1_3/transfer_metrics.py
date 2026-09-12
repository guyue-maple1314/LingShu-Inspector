"""tech_1_3 迁移指标统计

PPT 1.3 指标：
- 复杂地形通行率 >90%
- 实机迁移成功率 96.8%

纯离线统计模块，不调用实机接口。接受训练过程 / 实机回放的结构化事件
记录，输出 summary 与是否达标。
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass, field
from typing import Dict, List, Optional


PPT_PASS_RATE_THRESHOLD = 0.90    # 复杂地形通行率 >90%
PPT_TRANSFER_SUCCESS_THRESHOLD = 0.968  # 实机迁移成功率 96.8%

PASS_RATE_THRESHOLD_NAME = "复杂地形通行率阈值"
TRANSFER_SUCCESS_THRESHOLD_NAME = "实机迁移成功率阈值"


@dataclass
class EpisodeRecord:
    episode_id: str
    terrain_id: str
    terrain_type: str
    passed: bool
    sim_to_real: bool = False       # True=实机；False=仿真
    transfer_success: Optional[bool] = None  # 仅实机记录有意义：sim策略在实机上是否成功
    distance_m: Optional[float] = None
    duration_s: Optional[float] = None
    failure_reason: Optional[str] = None


class TransferMetricsCollector:
    """通行率 + 迁移成功率 双重指标收集器。"""

    def __init__(self) -> None:
        self._records: List[EpisodeRecord] = []
        # 仿真通行率历史（训练过程多次 checkpoint 或 rollout）
        self._sim_pass_rate_history: List[float] = []

    # ---------- 写入 ----------

    def record(self, rec: EpisodeRecord) -> None:
        self._records.append(rec)

    def record_pass_rate(self, pass_rate: float) -> None:
        self._sim_pass_rate_history.append(float(pass_rate))

    def record_batch(self, records) -> None:
        for r in records:
            self.record(r)

    # ---------- 仿真侧：复杂地形通行率 ----------

    def simulation_pass_rate(self, terrain_types: Optional[List[str]] = None) -> float:
        """复杂地形 = 非 flat 的其它地形（钢格网/坡道/楼梯/窄道）。"""
        sim = [r for r in self._records if not r.sim_to_real]
        if terrain_types:
            sim = [r for r in sim if r.terrain_type in terrain_types]
        else:
            sim = [r for r in sim if r.terrain_type != "flat"]
        if not sim:
            return 0.0
        return sum(1 for r in sim if r.passed) / len(sim)

    def sim_pass_rate_history(self) -> List[float]:
        return list(self._sim_pass_rate_history)

    def pass_rate_passed(self) -> bool:
        return self.simulation_pass_rate() > PPT_PASS_RATE_THRESHOLD

    # ---------- 实机侧：Sim-to-Real 迁移成功率 ----------

    def transfer_success_rate(self) -> float:
        real = [r for r in self._records if r.sim_to_real and r.transfer_success is not None]
        if not real:
            return 0.0
        return sum(1 for r in real if r.transfer_success) / len(real)

    def transfer_passed(self) -> bool:
        return self.transfer_success_rate() >= PPT_TRANSFER_SUCCESS_THRESHOLD

    # ---------- 汇总 ----------

    def summary_dict(self) -> Dict[str, object]:
        total = len(self._records)
        sim_count = sum(1 for r in self._records if not r.sim_to_real)
        real_count = total - sim_count
        mean_pass_hist = (
            statistics.mean(self._sim_pass_rate_history)
            if self._sim_pass_rate_history else 0.0
        )
        transfer_sr = self.transfer_success_rate()
        pass_sr = self.simulation_pass_rate()
        return {
            "total_episodes": total,
            "simulation_episodes": sim_count,
            "real_world_episodes": real_count,
            "simulation_pass_rate": pass_sr,
            "simulation_pass_rate_history_mean": mean_pass_hist,
            "simulation_pass_rate_history_count": len(self._sim_pass_rate_history),
            "transfer_success_rate": transfer_sr,
            "ppt_thresholds": {
                PASS_RATE_THRESHOLD_NAME: PPT_PASS_RATE_THRESHOLD,
                TRANSFER_SUCCESS_THRESHOLD_NAME: PPT_TRANSFER_SUCCESS_THRESHOLD,
            },
            "pass_rate_met": pass_sr > PPT_PASS_RATE_THRESHOLD,
            "transfer_rate_met": transfer_sr >= PPT_TRANSFER_SUCCESS_THRESHOLD,
            "failure_reasons_top": _top_failures(self._records, k=5),
        }


def _top_failures(records: List[EpisodeRecord], k: int) -> Dict[str, int]:
    counter: Dict[str, int] = {}
    for r in records:
        if r.failure_reason:
            counter[r.failure_reason] = counter.get(r.failure_reason, 0) + 1
    return dict(sorted(counter.items(), key=lambda kv: kv[1], reverse=True)[:k])
