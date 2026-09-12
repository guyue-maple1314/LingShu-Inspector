"""1.2 安全电量策略（纯逻辑，不依赖 rclpy）。

规则：

1. 默认安全电量基准为 30%。
2. 动态安全阈值：``safe_soc = max(30%, estimated_return_soc + 10 个百分点)``。
3. ``current_soc <= safe_soc`` 时：保存当前任务进度，停止启动新的普通
   巡检/采集任务，进入返航充电调度。
4. 新任务准入条件：
   ``current_soc - estimated_task_soc - estimated_return_soc >= 10 个百分点``。
5. 30% 是任务调度层临时运行阈值，不是 BMS 硬件欠压保护值；实机标定完成前
   不得调低。
"""

from dataclasses import dataclass

#: 固定安全电量基准（%）
DEFAULT_BASE_SAFE_SOC_PERCENT = 30.0
#: 返航额外余量（10 个百分点）
DEFAULT_RETURN_RESERVE_PERCENT_POINTS = 10.0
#: 策略状态：临时候选值，实机标定后才转正式
POLICY_STATUS = "provisional_until_real_robot_calibration"


@dataclass(frozen=True)
class BatterySafetyDecision:
    """安全电量评估结果。"""

    current_soc: float
    estimated_task_soc: float
    estimated_return_soc: float
    base_safe_soc_percent: float
    safe_soc: float
    must_return: bool          # True = 保存进度并返航充电
    admit_new_task: bool       # True = 允许启动新普通任务
    save_progress: bool        # True = 调度器须先保存当前任务进度
    reason: str


class BatterySafetyPolicy:
    """30% 固定基准 + 动态返航阈值 + 新任务准入判断。"""

    def __init__(
        self,
        base_safe_soc_percent: float = DEFAULT_BASE_SAFE_SOC_PERCENT,
        return_reserve_percent_points: float = DEFAULT_RETURN_RESERVE_PERCENT_POINTS,
    ) -> None:
        if not 0.0 <= base_safe_soc_percent <= 100.0:
            raise ValueError("base_safe_soc_percent must be within [0, 100]")
        if return_reserve_percent_points < 0.0:
            raise ValueError("return_reserve_percent_points must be >= 0")
        self._base = float(base_safe_soc_percent)
        self._reserve = float(return_reserve_percent_points)

    @property
    def base_safe_soc_percent(self) -> float:
        return self._base

    @property
    def return_reserve_percent_points(self) -> float:
        return self._reserve

    def safe_soc(self, estimated_return_soc: float) -> float:
        """动态安全阈值：``max(30%, 预计返航耗电 + 10pp)``。"""
        return max(self._base, float(estimated_return_soc) + self._reserve)

    def must_return_to_charge(
        self, current_soc: float, estimated_return_soc: float
    ) -> bool:
        """SOC 触及动态阈值（含等号）即须返航。"""
        return float(current_soc) <= self.safe_soc(estimated_return_soc)

    def can_admit_new_task(
        self,
        current_soc: float,
        estimated_task_soc: float,
        estimated_return_soc: float,
    ) -> bool:
        """新任务准入：任务后返航仍保留 >= 10 个百分点。"""
        return (
            float(current_soc)
            - float(estimated_task_soc)
            - float(estimated_return_soc)
            >= self._reserve
        )

    def evaluate(
        self,
        current_soc: float,
        estimated_task_soc: float = 0.0,
        estimated_return_soc: float = 0.0,
    ) -> BatterySafetyDecision:
        """综合评估：先判返航，再判新任务准入。"""
        current_soc = float(current_soc)
        estimated_task_soc = float(estimated_task_soc)
        estimated_return_soc = float(estimated_return_soc)
        safe = self.safe_soc(estimated_return_soc)

        if current_soc <= safe:
            return BatterySafetyDecision(
                current_soc=current_soc,
                estimated_task_soc=estimated_task_soc,
                estimated_return_soc=estimated_return_soc,
                base_safe_soc_percent=self._base,
                safe_soc=safe,
                must_return=True,
                admit_new_task=False,
                save_progress=True,
                reason=(
                    f"SOC {current_soc:.1f}% <= safe_soc {safe:.1f}%; "
                    "save progress, block new tasks and return to charge"
                ),
            )

        admitted = self.can_admit_new_task(
            current_soc, estimated_task_soc, estimated_return_soc
        )
        if admitted:
            reason = "above dynamic safe soc; task admission margin sufficient"
        else:
            reason = (
                "above safe_soc but remaining margin after task+return "
                f"< {self._reserve:.1f}pp; new task not admitted"
            )
        return BatterySafetyDecision(
            current_soc=current_soc,
            estimated_task_soc=estimated_task_soc,
            estimated_return_soc=estimated_return_soc,
            base_safe_soc_percent=self._base,
            safe_soc=safe,
            must_return=False,
            admit_new_task=admitted,
            save_progress=False,
            reason=reason,
        )
