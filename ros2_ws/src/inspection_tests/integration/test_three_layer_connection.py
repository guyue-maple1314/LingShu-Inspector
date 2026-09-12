"""三层连通性集成测试。

前置条件：已通过 system.launch.py（或同时）启动 C++ / Python 心跳节点。
验证 C++ 层发布 RobotState、Python 层发布 OperatorInstruction。
"""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from inspection_interfaces.msg import OperatorInstruction, RobotState
from inspection_planning_py.common.topic_names import ROBOT_STATE, OPERATOR_INSTRUCTION


class Listener(Node):
    def __init__(self):
        super().__init__("three_layer_connection_listener")
        self.robot_states = []
        self.instructions = []
        self.create_subscription(
            RobotState, ROBOT_STATE, lambda m: self.robot_states.append(m), 10
        )
        self.create_subscription(
            OperatorInstruction,
            OPERATOR_INSTRUCTION,
            lambda m: self.instructions.append(m),
            10,
        )


@pytest.fixture(scope="module")
def listener():
    rclpy.init()
    node = Listener()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.5)
        if node.robot_states and node.instructions:
            break

    yield node

    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


def test_cpp_layer_publishes_robot_state(listener):
    assert listener.robot_states, "未收到 C++ 层发布的 RobotState"


def test_python_layer_publishes_instruction(listener):
    assert listener.instructions, "未收到 Python 层发布的 OperatorInstruction"
