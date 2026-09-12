"""1.2 任务抢占集成测试（需 ROS 2 环境，并已启动 preemption_scheduler_node 依赖）。"""

import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from inspection_interfaces.msg import EnergyConstraint, Task, TaskDecision
from inspection_planning_py.common.topic_names import TASK, TASK_DECISION, ENERGY_CONSTRAINT
from inspection_planning_py.tech_1_2.preemption_scheduler_node import PreemptionSchedulerNode


def _make_task(task_id, benefit, risk, urgency):
    task = Task()
    task.task_id = task_id
    task.task_type = "inspection"
    task.benefit = benefit
    task.risk = risk
    task.urgency = urgency
    return task


def _make_energy(battery=100.0, distance=0.0):
    energy = EnergyConstraint()
    energy.battery_level = battery
    energy.return_distance = distance
    energy.estimated_consumption = 0.0
    return energy


@pytest.fixture(scope="module")
def scheduler():
    rclpy.init()
    scheduler_node = PreemptionSchedulerNode()
    test_node = rclpy.create_node("preemption_integration_test")
    decisions = []
    test_node.create_subscription(
        TaskDecision, TASK_DECISION, lambda m: decisions.append(m), 10
    )
    task_pub = test_node.create_publisher(Task, TASK, 10)
    energy_pub = test_node.create_publisher(EnergyConstraint, ENERGY_CONSTRAINT, 10)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(scheduler_node)
    executor.add_node(test_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    yield task_pub, energy_pub, decisions

    executor.shutdown()
    rclpy.shutdown()


def _wait_for_decision(decisions, target, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for d in list(decisions):
            if d.decision == target:
                return d
        time.sleep(0.05)
    return None


def test_preemption_on_high_value_task(scheduler):
    task_pub, energy_pub, decisions = scheduler
    energy_pub.publish(_make_energy())
    task_pub.publish(_make_task("low", benefit=1.0, risk=0.0, urgency=0.0))

    exec_decision = _wait_for_decision(decisions, "EXECUTE")
    assert exec_decision is not None
    assert exec_decision.current_task_id == "low"

    task_pub.publish(_make_task("high", benefit=10.0, risk=0.0, urgency=0.0))
    preempt_decision = _wait_for_decision(decisions, "PREEMPT")
    assert preempt_decision is not None
    assert preempt_decision.current_task_id == "high"
