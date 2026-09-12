"""1.2 能耗约束节点：从本体电量与参数生成并发布 EnergyConstraint。

同时发布预计返航耗电、30% 基准与动态安全阈值
``safe_soc = max(30%, 预计返航耗电 + 10pp)``，参数默认值与
config/battery_safety.yaml 保持一致，可被 launch 参数覆盖。
"""

import rclpy
from rclpy.node import Node

from inspection_interfaces.msg import EnergyConstraint, RobotState

from inspection_planning_py.common.node_names import ENERGY_CONSTRAINT_NODE
from inspection_planning_py.common.topic_names import ENERGY_CONSTRAINT, ROBOT_STATE
from inspection_planning_py.tech_1_2.battery_safety_policy import (
    DEFAULT_BASE_SAFE_SOC_PERCENT,
    DEFAULT_RETURN_RESERVE_PERCENT_POINTS,
)


class EnergyConstraintNode(Node):
    def __init__(self) -> None:
        super().__init__(ENERGY_CONSTRAINT_NODE)
        self.declare_parameter("return_distance_m", 0.0)
        self.declare_parameter("estimated_consumption_percent", 0.0)
        # 安全电量参数（默认值 = battery_safety.yaml）
        self.declare_parameter(
            "base_safe_soc_percent", DEFAULT_BASE_SAFE_SOC_PERCENT
        )
        self.declare_parameter(
            "return_reserve_percent_points",
            DEFAULT_RETURN_RESERVE_PERCENT_POINTS,
        )
        self.declare_parameter("consumption_per_meter_percent", 0.01)

        self._battery_percent = 100.0
        self._publisher = self.create_publisher(EnergyConstraint, ENERGY_CONSTRAINT, 10)
        self.create_subscription(RobotState, ROBOT_STATE, self._on_robot_state, 10)
        self.create_timer(1.0, self._publish)

    def _on_robot_state(self, msg: RobotState) -> None:
        self._battery_percent = float(msg.battery_level)

    def _publish(self) -> None:
        return_distance = float(self.get_parameter("return_distance_m").value)
        per_meter = float(self.get_parameter("consumption_per_meter_percent").value)
        base_safe_soc = float(self.get_parameter("base_safe_soc_percent").value)
        reserve = float(
            self.get_parameter("return_reserve_percent_points").value
        )

        # 预计返航耗电与动态安全阈值
        estimated_return_consumption = return_distance * per_meter
        safe_soc = max(base_safe_soc, estimated_return_consumption + reserve)

        msg = EnergyConstraint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.battery_level = self._battery_percent
        msg.return_distance = return_distance
        msg.estimated_consumption = float(
            self.get_parameter("estimated_consumption_percent").value
        )
        msg.estimated_return_consumption = estimated_return_consumption
        msg.base_safe_soc_percent = base_safe_soc
        msg.safe_soc = safe_soc
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnergyConstraintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
