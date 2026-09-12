"""低电量自动返航集成测试（1.2）。

需 ROS 2 环境，启动 preemption_scheduler_node 依赖后运行：
    pytest test_low_battery_return.py -v

验证规则（设计方案 §6.2 / §9.5）：
1. SOC 低于 30% 固定基准 → 调度决定 CHARGE（保存进度、返航充电）；
2. SOC 恰好等于 30%（等号边界）→ CHARGE；
3. 预计返航耗电抬高动态阈值（safe_soc=35）时，33% 也须 CHARGE；
4. 电量恢复到安全区后，任务可正常 EXECUTE。
"""

import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

from inspection_interfaces.msg import EnergyConstraint, Task, TaskDecision
from inspection_planning_py.common.topic_names import TASK, TASK_DECISION, ENERGY_CONSTRAINT
from inspection_planning_py.tech_1_2.preemption_scheduler_node import PreemptionSchedulerNode


def _make_task(task_id="patrol-1", benefit=5.0):
    task = Task()
    task.task_id = task_id
    task.task_type = "inspection"
    task.benefit = benefit
    task.risk = 0.0
    task.urgency = 0.0
    return task


def _make_energy(battery, return_consumption=0.0):
    energy = EnergyConstraint()
    energy.battery_level = battery
    energy.return_distance = 0.0
    energy.estimated_consumption = 0.0
    energy.estimated_return_consumption = return_consumption
    energy.base_safe_soc_percent = 30.0
    energy.safe_soc = max(30.0, return_consumption + 10.0)
    return energy


@pytest.fixture(scope="module")
def scheduler():
    rclpy.init()
    scheduler_node = PreemptionSchedulerNode()
    test_node = rclpy.create_node("low_battery_return_integration_test")
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


def test_charge_below_30_percent(scheduler):
    task_pub, energy_pub, decisions = scheduler
    task_pub.publish(_make_task())
    energy_pub.publish(_make_energy(battery=25.0))

    decision = _wait_for_decision(decisions, "CHARGE")
    assert decision is not None
    # 返航决定保留当前任务标识，供进度保存/恢复链路使用
    assert decision.current_task_id in ("patrol-1", "")


def test_charge_at_exact_30_percent_boundary(scheduler):
    task_pub, energy_pub, decisions = scheduler
    task_pub.publish(_make_task())
    energy_pub.publish(_make_energy(battery=30.0))

    decision = _wait_for_decision(decisions, "CHARGE")
    assert decision is not None


def test_charge_when_dynamic_threshold_raised(scheduler):
    # 预计返航耗电 25% → safe_soc=35%，SOC=33% 必须返航
    task_pub, energy_pub, decisions = scheduler
    task_pub.publish(_make_task())
    energy_pub.publish(_make_energy(battery=33.0, return_consumption=25.0))

    decision = _wait_for_decision(decisions, "CHARGE")
    assert decision is not None


def test_resume_execution_when_battery_recovers(scheduler):
    task_pub, energy_pub, decisions = scheduler
    task_pub.publish(_make_task())
    energy_pub.publish(_make_energy(battery=80.0))

    decision = _wait_for_decision(decisions, "EXECUTE")
    assert decision is not None
    assert decision.current_task_id == "patrol-1"
