"""1.1 异常触发后的目标更新与路径重规划节点。"""

import time

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import InspectionAlert, StructuredGoal

from inspection_planning_py.common.node_names import GOAL_REPLANNER_NODE
from inspection_planning_py.common.topic_names import INSPECTION_ALERT, STRUCTURED_GOAL


class GoalReplannerNode(Node):
    def __init__(self) -> None:
        super().__init__(GOAL_REPLANNER_NODE)
        self._seq = 0
        self._publisher = self.create_publisher(StructuredGoal, STRUCTURED_GOAL, 10)
        self.create_subscription(InspectionAlert, INSPECTION_ALERT, self._on_alert, 10)

    def _on_alert(self, alert: InspectionAlert) -> None:
        self._seq += 1
        msg = StructuredGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.goal_id = f"replan-{self._seq}"
        msg.task_type = "abnormal"
        msg.target_pose = alert.detection_pose
        msg.constraints = f"replan on alert {alert.alert_type}"
        msg.valid_until.sec = int(time.time()) + 30
        msg.valid_until.nanosec = 0
        self._publisher.publish(msg)
        self.get_logger().info(f"replanned goal {msg.goal_id} for alert {alert.alert_type}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalReplannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
