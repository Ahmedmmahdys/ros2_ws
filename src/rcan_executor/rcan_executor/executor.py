"""Basic RCAN executor node.

This initial version simply subscribes to the panel task feed and reports the
incoming work items. The intent is to provide a clean starting point for more
sophisticated task execution logic.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from crane_interfaces.msg import PanelTask


class RCANExecutor(Node):
    """Listen for :class:`~crane_interfaces.msg.PanelTask` messages and log them."""

    def __init__(self) -> None:
        super().__init__("rcan_executor")

        self.declare_parameter("panel_task_topic", "panel_task")
        panel_task_topic = (
            self.get_parameter("panel_task_topic").get_parameter_value().string_value
        )

        self._subscription = self.create_subscription(
            PanelTask,
            panel_task_topic,
            self._handle_panel_task,
            10,
        )

        self._last_task: Optional[PanelTask] = None

        self.get_logger().info(
            "RCAN executor ready; waiting for panel tasks on '%s'.", panel_task_topic
        )

    def _handle_panel_task(self, task: PanelTask) -> None:
        """Report each task and remember the most recent one."""

        self._last_task = task

        hook = task.hook_point
        panel = task.panel_position
        target = task.target_position

        self.get_logger().info(
            (
                "Received panel task %s -> %s\n"
                "  hook:   (%.3f, %.3f, %.3f)\n"
                "  panel:  (%.3f, %.3f, %.3f)\n"
                "  target: (%.3f, %.3f, %.3f)"
            ),
            task.ifc_guid or "<unknown>",
            task.next_ifc_guid or "<end>",
            hook.x,
            hook.y,
            hook.z,
            panel.x,
            panel.y,
            panel.z,
            target.x,
            target.y,
            target.z,
        )

    @property
    def last_task(self) -> Optional[PanelTask]:
        """Expose the most recent panel task handled by the node."""

        return self._last_task


def main() -> None:
    rclpy.init()
    node = RCANExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RCAN executor interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
