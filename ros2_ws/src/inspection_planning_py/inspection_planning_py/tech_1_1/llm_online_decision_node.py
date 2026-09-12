"""1.1 大模型在线决策节点：多源状态理解，生成结构化目标。"""

import time
import threading
from collections import deque
from typing import Any, Dict

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from inspection_interfaces.msg import (
    InspectionAlert,
    OperatorInstruction,
    RobotState,
    StructuredGoal,
)

from inspection_planning_py.common.node_names import LLM_ONLINE_DECISION_NODE
from inspection_planning_py.common.topic_names import (
    INSPECTION_ALERT,
    OPERATOR_INSTRUCTION,
    ROBOT_STATE,
    STRUCTURED_GOAL,
)
from inspection_planning_py.tech_1_1.decision_context_builder import build_decision_context
from inspection_planning_py.tech_1_1.decision_metrics import DecisionMetrics
from inspection_planning_py.tech_1_1.goal_schema import validate_goal_schema
from inspection_planning_py.tech_1_1.llm_client import DeepseekLlmClient, LlmClient, StubLlmClient
from inspection_planning_py.common.structured_output_validator import (
    validate_goal_schema as precheck_goal,
)
from inspection_planning_py.common.time_budget import TimeBudget


class LlmOnlineDecisionNode(Node):
    def __init__(self) -> None:
        super().__init__(LLM_ONLINE_DECISION_NODE)
        self.declare_parameter("llm_backend", "deepseek")
        self.declare_parameter("llm_model", "deepseek-chat")
        self.declare_parameter("llm_timeout_sec", 30.0)
        self.declare_parameter("decision_time_budget_ms", 500.0)
        # 500 ms 决策时间预算（1.1 验收口径），超预算记警告，不做静默降级
        self._time_budget = TimeBudget(
            float(self.get_parameter("decision_time_budget_ms").value)
        )

        backend = self.get_parameter("llm_backend").value
        self._llm: LlmClient = self._make_llm(backend)

        self._latest_robot_state = None
        # 告警只保留最近 N 条：避免长期运行内存与提示词无界增长
        self._alerts = deque(maxlen=50)
        self._latest_instruction = None
        self._metrics = DecisionMetrics()
        # MultiThreadedExecutor 下告警/指令回调可能并发进入决策，串行化
        self._decision_lock = threading.Lock()

        self.create_subscription(RobotState, ROBOT_STATE, self._on_robot_state, 10)
        self.create_subscription(InspectionAlert, INSPECTION_ALERT, self._on_alert, 10)
        self.create_subscription(OperatorInstruction, OPERATOR_INSTRUCTION, self._on_instruction, 10)
        self._publisher = self.create_publisher(StructuredGoal, STRUCTURED_GOAL, 10)

    def _make_llm(self, backend: str) -> LlmClient:
        if backend == "deepseek":
            try:
                client = DeepseekLlmClient(
                    model=self.get_parameter("llm_model").value,
                    timeout=self.get_parameter("llm_timeout_sec").value,
                )
                self.get_logger().info("using DeepSeek LLM backend")
                return client
            except ValueError as exc:
                self.get_logger().warning(f"DeepSeek unavailable ({exc}); falling back to stub")
        return StubLlmClient()

    def _on_robot_state(self, msg: RobotState) -> None:
        self._latest_robot_state = msg

    def _on_alert(self, msg: InspectionAlert) -> None:
        self._alerts.append(msg)
        self._maybe_decide("alert")

    def _on_instruction(self, msg: OperatorInstruction) -> None:
        self._latest_instruction = msg
        self._maybe_decide("instruction")

    def _maybe_decide(self, trigger: str) -> None:
        with self._decision_lock:
            self._time_budget.start()
            start = time.monotonic()
            context = build_decision_context(
                self._latest_robot_state, list(self._alerts), self._latest_instruction
            )
            goal = self._llm.generate_goal(context)
            ok, _ = validate_goal_schema(goal) if goal is not None else (False, "no goal")
            latency_ms = (time.monotonic() - start) * 1000.0
            self._metrics.record(ok, latency_ms)
            if self._time_budget.is_exceeded():
                self.get_logger().warning(
                    f"decision took {latency_ms:.1f} ms, exceeding budget "
                    f"{self._time_budget.budget_ms:.0f} ms"
                )
            if not ok:
                self.get_logger().warning(f"LLM produced invalid goal: {goal}")
                return
            self._publish_goal(goal)

    def _publish_goal(self, goal: Dict[str, Any]) -> None:
        # Python 端结构化目标预校验：字段不全或类型不符时不下发
        # （最终执行权仍属于 C++ 行为树的安全校验）
        payload = {
            "goal_id": goal["goal_id"],
            "task_type": goal["task_type"],
            "target_pose": goal["target_pose"],
            "valid_until": goal["valid_until_sec"],
        }
        ok, error = precheck_goal(payload)
        if not ok:
            self.get_logger().warning(f"structured goal pre-check failed: {error}")
            return

        msg = StructuredGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.goal_id = goal["goal_id"]
        msg.task_type = goal["task_type"]
        msg.target_pose.position.x = float(goal["target_pose"]["x"])
        msg.target_pose.position.y = float(goal["target_pose"]["y"])
        msg.target_pose.position.z = float(goal["target_pose"]["z"])
        msg.constraints = goal["constraints"]
        msg.valid_until.sec = int(goal["valid_until_sec"])
        msg.valid_until.nanosec = 0
        self._publisher.publish(msg)
        self.get_logger().info(f"published structured goal {msg.goal_id}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LlmOnlineDecisionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
