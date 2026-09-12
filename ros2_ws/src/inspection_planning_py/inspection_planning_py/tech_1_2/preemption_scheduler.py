"""1.2 纯调度逻辑：执行 / 抢占 / 恢复 / 充电 / 等待。

CHARGE 分支改用 BatterySafetyPolicy：
- safe_soc = max(30%, 预计返航耗电 + 10pp)，SOC <= safe_soc 即保存进度返航；
- 启动新任务（含抢占切换）前必须通过新任务准入公式
  current_soc - task_soc - return_soc >= 10pp。
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

from .battery_safety_policy import BatterySafetyPolicy
from .task_value_evaluator import TaskValueEvaluator

#: 未显式给出预计返航耗电时的默认单位距离耗电（%/m），与 battery_safety.yaml 一致
_DEFAULT_CONSUMPTION_PER_METER = 0.01


@dataclass
class SchedulingDecision:
    decision: str  # EXECUTE / PREEMPT / RESUME / CHARGE / WAIT
    current_task_id: Optional[str]
    ranked_task_ids: List[str]
    reason: str


def decide(
    tasks: List[Dict[str, float]],
    current_task_id: Optional[str],
    battery_percent: float,
    return_distance_m: float,
    resumable_task_id: Optional[str] = None,
    value_evaluator: Optional[TaskValueEvaluator] = None,
    preemption_margin: float = 0.0,
    estimated_return_soc: Optional[float] = None,
    battery_policy: Optional[BatterySafetyPolicy] = None,
) -> SchedulingDecision:
    evaluator = value_evaluator or TaskValueEvaluator()
    policy = battery_policy or BatterySafetyPolicy()
    if estimated_return_soc is None:
        estimated_return_soc = return_distance_m * _DEFAULT_CONSUMPTION_PER_METER

    ranked = sorted(tasks, key=evaluator.evaluate, reverse=True)
    ranked_ids = [t["task_id"] for t in ranked]

    # 规则 3：SOC <= 动态安全阈值 → 保存进度、禁止新任务、返航充电
    safety = policy.evaluate(battery_percent, 0.0, estimated_return_soc)
    if safety.must_return:
        return SchedulingDecision(
            "CHARGE", current_task_id, ranked_ids, safety.reason
        )

    if not ranked:
        if resumable_task_id:
            return SchedulingDecision(
                "RESUME", resumable_task_id, [], "no candidate tasks; resume saved task"
            )
        return SchedulingDecision("WAIT", None, [], "no candidate tasks")

    best = ranked[0]
    best_value = evaluator.evaluate(best)
    best_task_soc = float(best.get("estimated_consumption", 0.0))

    if current_task_id is not None:
        current = next((t for t in tasks if t["task_id"] == current_task_id), None)
        if current is not None:
            current_value = evaluator.evaluate(current)
            if best["task_id"] != current_task_id and best_value > current_value + preemption_margin:
                # 规则 4：抢占切换等价于启动新任务，必须过准入
                if policy.can_admit_new_task(
                    battery_percent, best_task_soc, estimated_return_soc
                ):
                    return SchedulingDecision(
                        "PREEMPT", best["task_id"], ranked_ids,
                        f"higher-value task {best['task_id']} arrived",
                    )
                return SchedulingDecision(
                    "EXECUTE", current_task_id, ranked_ids,
                    "preemption blocked: battery admission margin insufficient",
                )
            return SchedulingDecision(
                "EXECUTE", current_task_id, ranked_ids, "current task still highest value"
            )

    # 规则 4：无在执行任务，启动最高分新任务前先验准入
    if not policy.can_admit_new_task(
        battery_percent, best_task_soc, estimated_return_soc
    ):
        return SchedulingDecision(
            "WAIT", None, ranked_ids,
            "new task blocked: battery margin after task+return below reserve",
        )

    return SchedulingDecision("EXECUTE", best["task_id"], ranked_ids, "execute highest-value task")
