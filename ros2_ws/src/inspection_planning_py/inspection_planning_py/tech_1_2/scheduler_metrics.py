"""1.2 统计任务成功率和抢占响应时间。"""

from typing import List


class SchedulerMetrics:
    def __init__(self) -> None:
        self._tasks_total = 0
        self._tasks_success = 0
        self._preemption_latency_ms: List[float] = []

    def record_task(self, success: bool) -> None:
        self._tasks_total += 1
        if success:
            self._tasks_success += 1

    def record_preemption(self, latency_ms: float) -> None:
        self._preemption_latency_ms.append(float(latency_ms))

    def task_success_rate(self) -> float:
        if self._tasks_total == 0:
            return 0.0
        return self._tasks_success / self._tasks_total

    def max_preemption_ms(self) -> float:
        return max(self._preemption_latency_ms, default=0.0)
