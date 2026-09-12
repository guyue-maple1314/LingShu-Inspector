"""1.2 抢占调度节点：按价值/能耗决定执行、抢占、恢复、充电，并作为 ExecuteTask 客户端下发目标。"""

import time
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from inspection_interfaces.action import ExecuteTask
from inspection_interfaces.msg import EnergyConstraint, StructuredGoal, Task, TaskDecision
from inspection_interfaces.srv import ValidateGoal

from inspection_planning_py.common.node_names import PREEMPTION_SCHEDULER_NODE
from inspection_planning_py.common.topic_names import (
    ENERGY_CONSTRAINT,
    EXECUTE_TASK_ACTION,
    STRUCTURED_GOAL,
    TASK,
    TASK_DECISION,
    VALIDATE_GOAL_SERVICE,
)
from inspection_planning_py.tech_1_2.battery_safety_policy import (
    BatterySafetyPolicy,
    DEFAULT_BASE_SAFE_SOC_PERCENT,
    DEFAULT_RETURN_RESERVE_PERCENT_POINTS,
)
from inspection_planning_py.tech_1_2.preemption_scheduler import decide
from inspection_planning_py.tech_1_2.scheduler_metrics import SchedulerMetrics


class PreemptionSchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__(PREEMPTION_SCHEDULER_NODE)
        self.declare_parameter("scheduler_period_sec", 0.1)
        # 安全电量参数（默认值 = config/battery_safety.yaml）
        self.declare_parameter(
            "base_safe_soc_percent", DEFAULT_BASE_SAFE_SOC_PERCENT
        )
        self.declare_parameter(
            "return_reserve_percent_points",
            DEFAULT_RETURN_RESERVE_PERCENT_POINTS,
        )
        self._tasks: Dict[str, dict] = {}
        self._goals: Dict[str, StructuredGoal] = {}
        self._current_task_id: Optional[str] = None
        self._battery_percent = 100.0
        self._return_distance_m = 0.0
        self._estimated_return_soc = 0.0
        self._returning_to_charge = False
        self._battery_policy = BatterySafetyPolicy(
            base_safe_soc_percent=float(
                self.get_parameter("base_safe_soc_percent").value
            ),
            return_reserve_percent_points=float(
                self.get_parameter("return_reserve_percent_points").value
            ),
        )
        self._goal_handle = None
        self._pending_preempt_start: Optional[float] = None
        self._metrics = SchedulerMetrics()

        self._publisher = self.create_publisher(TaskDecision, TASK_DECISION, 10)
        self.create_subscription(Task, TASK, self._on_task, 10)
        self.create_subscription(StructuredGoal, STRUCTURED_GOAL, self._on_goal, 10)
        self.create_subscription(EnergyConstraint, ENERGY_CONSTRAINT, self._on_energy, 10)
        period = self.get_parameter("scheduler_period_sec").value
        self.create_timer(period, self._run_scheduler)

        self._action_client = ActionClient(self, ExecuteTask, EXECUTE_TASK_ACTION)
        self._validate_client = self.create_client(ValidateGoal, VALIDATE_GOAL_SERVICE)

    def _on_task(self, task: Task) -> None:
        self._tasks[task.task_id] = {
            "task_id": task.task_id,
            "benefit": float(task.benefit),
            "risk": float(task.risk),
            "urgency": float(task.urgency),
            "estimated_consumption": float(task.estimated_consumption),
        }

    def _on_goal(self, goal: StructuredGoal) -> None:
        self._goals[goal.goal_id] = goal

    def _on_energy(self, energy: EnergyConstraint) -> None:
        self._battery_percent = float(energy.battery_level)
        self._return_distance_m = float(energy.return_distance)
        # 预计返航耗电由 energy_constraint_node 按距离×单位耗电给出
        self._estimated_return_soc = float(
            energy.estimated_return_consumption
        )

    def _run_scheduler(self) -> None:
        decision = decide(
            list(self._tasks.values()),
            self._current_task_id,
            self._battery_percent,
            self._return_distance_m,
            estimated_return_soc=self._estimated_return_soc,
            battery_policy=self._battery_policy,
        )
        self._apply_decision(decision)
        self._publish_decision(decision)
        self.get_logger().info(f"scheduler success_rate={self._metrics.task_success_rate():.2f}")

    def _apply_decision(self, decision) -> None:
        if decision.decision != "CHARGE":
            self._returning_to_charge = False
        if decision.decision == "PREEMPT":
            self._cancel_current()
            self._current_task_id = decision.current_task_id
            self._pending_preempt_start = time.monotonic()
            self._send_goal(decision.current_task_id)
        elif decision.decision == "EXECUTE":
            if decision.current_task_id != self._current_task_id:
                self._current_task_id = decision.current_task_id
                self._send_goal(decision.current_task_id)
        elif decision.decision == "RESUME":
            self._current_task_id = decision.current_task_id
            self._send_goal(decision.current_task_id)
        elif decision.decision == "CHARGE":
            # SOC<=safe_soc：取消当前 Action（C++ 端负责保存可恢复进度），
            # 停止新任务并进入返航充电调度；返航终点由导航执行层按充电桩位姿下发。
            if not self._returning_to_charge:
                self._returning_to_charge = True
                self.get_logger().warning(
                    f"low battery return-to-charge: {decision.reason}"
                )
            self._cancel_current()
            self._current_task_id = None

    def _send_goal(self, task_id: Optional[str]) -> None:
        if not task_id or task_id not in self._goals:
            self.get_logger().warning(f"no structured goal available for task {task_id}")
            return
        goal_msg = self._goals[task_id]

        if not self._validate_client.service_is_ready():
            self._dispatch_goal(goal_msg)
            return
        request = ValidateGoal.Request()
        request.goal = goal_msg
        future = self._validate_client.call_async(request)
        future.add_done_callback(lambda f: self._on_validate(f, goal_msg))

    def _on_validate(self, future, goal_msg: StructuredGoal) -> None:
        try:
            response = future.result()
        except Exception:
            self.get_logger().warning("validate_goal service call failed; dispatching anyway")
            self._dispatch_goal(goal_msg)
            return
        if response.allowed:
            self._dispatch_goal(goal_msg)
        else:
            self.get_logger().warning(f"goal rejected: {response.rejection_reason}")

    def _dispatch_goal(self, goal_msg: StructuredGoal) -> None:
        if not self._action_client.server_is_ready():
            self.get_logger().warning("execute_task action server not ready")
            return
        goal = ExecuteTask.Goal()
        goal.goal = goal_msg
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("execute_task goal rejected")
            return
        if self._pending_preempt_start is not None:
            latency_ms = (time.monotonic() - self._pending_preempt_start) * 1000.0
            self._metrics.record_preemption(latency_ms)
            self._pending_preempt_start = None
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        result = future.result().result
        self._metrics.record_task(result.success)
        self.get_logger().info(
            f"execute_task finished: success={result.success} message={result.message}"
        )

    def _cancel_current(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _publish_decision(self, decision) -> None:
        msg = TaskDecision()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_task_id = decision.current_task_id or ""
        msg.ranked_task_ids = list(decision.ranked_task_ids)
        msg.decision = decision.decision
        msg.decision_time = self.get_clock().now().to_msg()
        self._publisher.publish(msg)
        self.get_logger().info(
            f"decision={decision.decision} current={decision.current_task_id}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PreemptionSchedulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
