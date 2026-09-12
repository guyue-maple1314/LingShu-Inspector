"""1.1 统计结构化指令成功率和决策耗时。"""

from typing import List


class DecisionMetrics:
    def __init__(self) -> None:
        self._total = 0
        self._success = 0
        self._latency_ms: List[float] = []

    def record(self, success: bool, latency_ms: float) -> None:
        self._total += 1
        if success:
            self._success += 1
        self._latency_ms.append(float(latency_ms))

    def success_rate(self) -> float:
        if self._total == 0:
            return 0.0
        return self._success / self._total

    def latency_percentile(self, p: float) -> float:
        if not self._latency_ms:
            return 0.0
        ordered = sorted(self._latency_ms)
        k = (len(ordered) - 1) * (p / 100.0)
        lo = int(k)
        hi = min(lo + 1, len(ordered) - 1)
        frac = k - lo
        return ordered[lo] * (1.0 - frac) + ordered[hi] * frac

    def p95_ms(self) -> float:
        return self.latency_percentile(95.0)
