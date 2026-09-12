"""1.2 只按风险、收益、紧急度做多维价值评估。"""

from typing import Dict


def evaluate_value(
    benefit: float,
    risk: float,
    urgency: float,
    benefit_weight: float = 1.0,
    risk_weight: float = 1.0,
    urgency_weight: float = 1.0,
) -> float:
    return benefit_weight * benefit - risk_weight * risk + urgency_weight * urgency


class TaskValueEvaluator:
    def __init__(
        self,
        benefit_weight: float = 1.0,
        risk_weight: float = 1.0,
        urgency_weight: float = 1.0,
    ) -> None:
        self.benefit_weight = float(benefit_weight)
        self.risk_weight = float(risk_weight)
        self.urgency_weight = float(urgency_weight)

    def evaluate(self, task: Dict[str, float]) -> float:
        return evaluate_value(
            float(task["benefit"]),
            float(task["risk"]),
            float(task["urgency"]),
            self.benefit_weight,
            self.risk_weight,
            self.urgency_weight,
        )
