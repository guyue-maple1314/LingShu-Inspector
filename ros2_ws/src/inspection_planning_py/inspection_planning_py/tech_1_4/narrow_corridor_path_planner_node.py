"""1.4 窄通道路径规划节点：订阅 CorridorState，校验并生成中心线路径。"""

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import CorridorState

from inspection_planning_py.common.node_names import NARROW_CORRIDOR_PATH_PLANNER_NODE
from inspection_planning_py.common.topic_names import CORRIDOR_STATE
from inspection_planning_py.tech_1_4.corridor_metrics import CorridorMetrics
from inspection_planning_py.tech_1_4.narrow_corridor_path_planner import plan_path


class NarrowCorridorPathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__(NARROW_CORRIDOR_PATH_PLANNER_NODE)
        self.declare_parameter("robot_width", 0.46)
        self.declare_parameter("safety_margin", 0.10)
        self._metrics = CorridorMetrics()
        self.create_subscription(CorridorState, CORRIDOR_STATE, self._on_corridor, 10)

    def _on_corridor(self, msg: CorridorState) -> None:
        corridor = {
            "width": float(msg.corridor_width),
            "centerline": [{"x": p.x, "y": p.y} for p in msg.centerline],
            "visual_validation_passed": bool(msg.visual_validation_passed),
            "confidence": float(msg.confidence),
        }
        robot_width = float(self.get_parameter("robot_width").value)
        margin = float(self.get_parameter("safety_margin").value)
        path = plan_path(corridor, robot_width, margin)
        passed = len(path) > 0
        self._metrics.record_trial(passed)
        self.get_logger().info(
            f"corridor width={corridor['width']:.2f}m passed={passed} "
            f"path_points={len(path)} pass_rate={self._metrics.pass_rate():.2f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NarrowCorridorPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
