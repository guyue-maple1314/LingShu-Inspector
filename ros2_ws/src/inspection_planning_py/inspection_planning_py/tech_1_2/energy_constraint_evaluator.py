"""1.2 电量、返航距离、任务进度与充电安排约束。"""

from dataclasses import dataclass


@dataclass
class EnergyResult:
    feasible: bool
    required_percent: float
    remaining_after_task: float
    reason: str


def evaluate_energy(
    battery_percent: float,
    return_distance_m: float,
    estimated_consumption_percent: float,
    consumption_per_meter_percent: float = 0.01,
    safety_margin_percent: float = 5.0,
) -> EnergyResult:
    return_cost = return_distance_m * consumption_per_meter_percent
    required = estimated_consumption_percent + return_cost
    remaining = battery_percent - required
    feasible = remaining >= safety_margin_percent
    reason = "" if feasible else "energy insufficient for task plus return"
    return EnergyResult(
        feasible=feasible,
        required_percent=required,
        remaining_after_task=remaining,
        reason=reason,
    )
