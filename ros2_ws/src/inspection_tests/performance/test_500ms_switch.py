"""1.1/1.2 500ms 任务切换性能测试（需 ROS 2 环境）。"""

import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from inspection_interfaces.msg import EnergyConstraint, Task, TaskDecision
from inspection_planning_py.common.topic_names import TASK, TASK_DECISION, ENERGY_CONSTRAINT
from inspection_planning_py.tech_1_2.preemption_scheduler_node import PreemptionSchedulerNode


@pytest.fixture(scope="module")
def scheduler():
    rclpy.init()
    node = PreemptionSchedulerNode()
    test = rclpy.create_node("switch_perf_test")
    decisions = []
    test.create_subscription(
        TaskDecision, TASK_DECISION, lambda m: decisions.append((m, time.monotonic())), 10
    )
    task_pub = test.create_publisher(Task, TASK, 10)
    energy_pub = test.create_publisher(EnergyConstraint, ENERGY_CONSTRAINT, 10)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(test)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield task_pub, energy_pub, decisions

    executor.shutdown()
    rclpy.shutdown()


def test_switch_within_500ms(scheduler):
    task_pub, energy_pub, decisions = scheduler

    energy = EnergyConstraint()
    energy.battery_level = 100.0
    energy.return_distance = 0.0
    energy_pub.publish(energy)

    low = Task()
    low.task_id = "low"
    low.benefit = 1.0
    low.risk = 0.0
    low.urgency = 0.0
    task_pub.publish(low)
    time.sleep(0.3)

    start = time.monotonic()
    high = Task()
    high.task_id = "high"
    high.benefit = 10.0
    high.risk = 0.0
    high.urgency = 0.0
    task_pub.publish(high)

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        for decision, timestamp in list(decisions):
            if decision.decision == "PREEMPT":
                latency_ms = (timestamp - start) * 1000.0
                assert latency_ms <= 500.0, f"switch latency {latency_ms:.1f}ms > 500ms"
                return
        time.sleep(0.02)

    pytest.fail("no PREEMPT decision observed within timeout")
