"""RCAN executor node that simulates crane phases for each panel task."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from crane_interfaces.msg import PanelTask, TaskStatus


@dataclass
class PhaseResult:
    name: str
    success: bool
    detail: str


class RCANExecutor(Node):
    """Simulated RCAN executor that drives phases from panel task input."""

    def __init__(self) -> None:
        super().__init__("rcan_executor")

        self.declare_parameter("approach_clearance", 0.5)
        self.declare_parameter("lift_clearance", 1.5)

        self._approach_clearance = (
            self.get_parameter("approach_clearance").get_parameter_value().double_value
        )
        self._lift_clearance = (
            self.get_parameter("lift_clearance").get_parameter_value().double_value
        )

        if self._approach_clearance <= 0:
            raise ValueError("approach_clearance must be positive")
        if self._lift_clearance <= self._approach_clearance:
            raise ValueError(
                "lift_clearance must be greater than approach_clearance to maintain safe clearance",
            )

        self._status_publisher = self.create_publisher(TaskStatus, "/rcan/activity/status", 10)
        self._subscription = self.create_subscription(
            PanelTask, "/panel_task", self._process_task, 10
        )

        self.get_logger().info(
            "RCAN executor ready. Listening on '/panel_task' and publishing phase updates to '/rcan/activity/status'."
        )

    # ------------------------------------------------------------------
    def _process_task(self, task: PanelTask) -> None:
        """Handle a new panel task message."""

        if not task.ifc_guid:
            self.get_logger().error("Received panel task without an IFC GUID; ignoring task.")
            return

        self.get_logger().info(
            (
                "Received panel task %s: hook (%.3f, %.3f, %.3f), target (%.3f, %.3f, %.3f)"
            )
            % (
                task.ifc_guid,
                task.hook_point.x,
                task.hook_point.y,
                task.hook_point.z,
                task.target_position.x,
                task.target_position.y,
                task.target_position.z,
            )
        )

        phase_functions: Tuple[Tuple[str, Callable[[PanelTask], PhaseResult]], ...] = (
            ("MoveToComponent", self._phase_move_to_component),
            ("PositionForPick", self._phase_position_for_pick),
            ("PickUp", self._phase_pick_up),
            ("Transport", self._phase_transport),
            ("PositionForInstall", self._phase_position_for_install),
            ("Install", self._phase_install),
            ("Release", self._phase_release),
        )

        failure_triggered = False
        for index, (phase_name, phase_callable) in enumerate(phase_functions):
            if failure_triggered:
                skipped_detail = f"Skipped after {phase_functions[index - 1][0]} failure"
                self._publish_status(task.ifc_guid, phase_name, False, skipped_detail)
                continue

            result = phase_callable(task)
            self._publish_status(task.ifc_guid, result.name, result.success, result.detail)
            if not result.success:
                failure_triggered = True

    # ------------------------------------------------------------------
    def _phase_move_to_component(self, task: PanelTask) -> PhaseResult:
        return self._travel_above(
            task.hook_point,
            phase_name="MoveToComponent",
            description="hook point",
        )

    def _phase_position_for_pick(self, task: PanelTask) -> PhaseResult:
        return self._approach(
            task.hook_point,
            phase_name="PositionForPick",
            description="hook point",
        )

    def _phase_pick_up(self, task: PanelTask) -> PhaseResult:
        if not self._validate_point(task.hook_point):
            detail = "Invalid hook point coordinates"
            self.get_logger().error(f"[{task.ifc_guid}] {detail}")
            return PhaseResult("PickUp", False, detail)

        self.get_logger().info(
            f"[{task.ifc_guid}] PickUp: lowering to hook point at {task.hook_point.z:.3f} m and attaching load"
        )

        if not task.ifc_guid:
            return PhaseResult("PickUp", False, "Missing IFC GUID for attachment")

        self.get_logger().info(
            f"[{task.ifc_guid}] PickUp: attached, hoisting {self._lift_clearance:.3f} m"
        )
        return PhaseResult("PickUp", True, "Ok")

    def _phase_transport(self, task: PanelTask) -> PhaseResult:
        return self._travel_above(
            task.target_position,
            phase_name="Transport",
            description="target position",
        )

    def _phase_position_for_install(self, task: PanelTask) -> PhaseResult:
        return self._approach(
            task.target_position,
            phase_name="PositionForInstall",
            description="target position",
        )

    def _phase_install(self, task: PanelTask) -> PhaseResult:
        if not self._validate_point(task.target_position):
            detail = "Invalid target position coordinates"
            self.get_logger().error(f"[{task.ifc_guid}] {detail}")
            return PhaseResult("Install", False, detail)

        self.get_logger().info(
            f"[{task.ifc_guid}] Install: lowering panel to {task.target_position.z:.3f} m and releasing"
        )
        self.get_logger().info(
            f"[{task.ifc_guid}] Install: released, retreating by {self._approach_clearance:.3f} m"
        )
        return PhaseResult("Install", True, "Ok")

    def _phase_release(self, task: PanelTask) -> PhaseResult:
        if not self._validate_point(task.target_position):
            detail = "Invalid target position coordinates"
            self.get_logger().error(f"[{task.ifc_guid}] {detail}")
            return PhaseResult("Release", False, detail)

        self.get_logger().info(
            f"[{task.ifc_guid}] Release: confirming detachment and hoisting {self._lift_clearance:.3f} m"
        )
        return PhaseResult("Release", True, "Ok")

    # ------------------------------------------------------------------
    def _travel_above(self, point: Point, *, phase_name: str, description: str) -> PhaseResult:
        if not self._validate_point(point):
            detail = f"Invalid {description} coordinates"
            self.get_logger().error(detail)
            return PhaseResult(phase_name, False, detail)

        target_height = point.z + self._lift_clearance
        self.get_logger().info(
            f"{phase_name}: travelling above {description} to safe height {target_height:.3f} m"
        )
        return PhaseResult(phase_name, True, "Ok")

    def _approach(self, point: Point, *, phase_name: str, description: str) -> PhaseResult:
        if not self._validate_point(point):
            detail = f"Invalid {description} coordinates"
            self.get_logger().error(detail)
            return PhaseResult(phase_name, False, detail)

        approach_height = point.z + self._approach_clearance
        self.get_logger().info(
            f"{phase_name}: approaching {description} to clearance {approach_height:.3f} m"
        )
        return PhaseResult(phase_name, True, "Ok")

    def _validate_point(self, point: Point) -> bool:
        return all(math.isfinite(value) for value in (point.x, point.y, point.z))

    def _publish_status(self, ifc_guid: str, phase: str, success: bool, detail: str) -> None:
        message = TaskStatus()
        message.ifc_guid = ifc_guid
        message.phase = phase
        message.success = success
        message.detail = detail
        message.stamp = self.get_clock().now().to_msg()
        self._status_publisher.publish(message)
        outcome = "SUCCESS" if success else "FAILURE"
        self.get_logger().info(
            f"[{ifc_guid}] Phase {phase} result: {outcome} ({detail})"
        )


# ---------------------------------------------------------------------------
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


__all__ = ["main", "RCANExecutor"]
