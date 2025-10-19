"""ROS2 bridge that forwards crane state messages to the broker."""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .. import msg_broker

ROS_TOPIC = "ros.robot.state"


class RobotStateBridge(Node):
    """Bridge ROS messages on /rcan/state to the internal broker."""

    def __init__(self) -> None:
        super().__init__("rcan_state_bridge")
        self.subscription = self.create_subscription(  # type: ignore[assignment]
            String,
            "/rcan/state",
            self._handle_state,
            10,
        )

    def _handle_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:  # pragma: no cover - logging only
            self.get_logger().error("Failed to decode state message: %s", exc)
            return
        msg_broker.publish(ROS_TOPIC, payload)


def main() -> None:
    rclpy.init()
    node = RobotStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


__all__ = ["RobotStateBridge", "main", "ROS_TOPIC"]
