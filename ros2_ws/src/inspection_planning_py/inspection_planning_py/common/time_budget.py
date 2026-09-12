"""统计 500 ms 决策与抢占时间预算。"""

import time


class TimeBudget:
    def __init__(self, budget_ms: float):
        self.budget_ms = float(budget_ms)
        self._start = None

    def start(self) -> None:
        self._start = time.monotonic()

    def elapsed_ms(self) -> float:
        if self._start is None:
            return 0.0
        return (time.monotonic() - self._start) * 1000.0

    def is_exceeded(self) -> bool:
        return self.elapsed_ms() > self.budget_ms
