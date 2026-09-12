"""1.2 候选任务池节点：汇聚结构化目标为统一任务。"""

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import StructuredGoal, Task

from inspection_planning_py.common.node_names import CANDIDATE_TASK_POOL_NODE
from inspection_planning_py.common.topic_names import STRUCTURED_GOAL, TASK


class CandidateTaskPoolNode(Node):
    def __init__(self) -> None:
        super().__init__(CANDIDATE_TASK_POOL_NODE)
        # 预计任务耗电（%）：由规划层或参数给出，未配置时按 0 处理
        self.declare_parameter("estimated_consumption_percent", 0.0)
        self._estimated_consumption_percent = float(
            self.get_parameter("estimated_consumption_percent").value
        )
        self._publisher = self.create_publisher(Task, TASK, 10)
        self.create_subscription(StructuredGoal, STRUCTURED_GOAL, self._on_goal, 10)

    def _on_goal(self, goal: StructuredGoal) -> None:
        task = Task()
        task.header.stamp = self.get_clock().now().to_msg()
        task.task_id = goal.goal_id
        task.task_type = goal.task_type
        task.status = "candidate"
        task.risk = 1.0
        task.benefit = 1.0
        task.urgency = 2.0 if goal.task_type == "abnormal" else 1.0
        task.progress = 0.0
        task.estimated_consumption = self._estimated_consumption_percent
        if self._estimated_consumption_percent <= 0.0:
            self.get_logger().debug(
                "estimated_consumption_percent is 0; the admission margin "
                "check will only account for the return leg"
            )
        self._publisher.publish(task)
        self.get_logger().info(f"added candidate task {task.task_id}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CandidateTaskPoolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
