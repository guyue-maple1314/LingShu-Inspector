"""1.4 统计 75cm 通道试验结果。"""

PPT_CORRIDOR_WIDTH_M = 0.75


class CorridorMetrics:
    def __init__(self, target_width_m: float = PPT_CORRIDOR_WIDTH_M) -> None:
        self.target_width_m = float(target_width_m)
        self._trials = 0
        self._passes = 0

    def record_trial(self, passed: bool) -> None:
        self._trials += 1
        if passed:
            self._passes += 1

    def pass_rate(self) -> float:
        return self._passes / self._trials if self._trials else 0.0
