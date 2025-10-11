from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from crane_interfaces.msg import PanelTask
import yaml

from .panel_chain_utils import PanelLink, point_from_value, resolve_head


def load_panel_chain(chain_path: Path) -> tuple[Dict[str, PanelLink], Optional[str]]:
    if not chain_path.exists():
        raise FileNotFoundError(f"Panel chain file not found: {chain_path}")

    with chain_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    panels_raw = data.get("panels")
    if not isinstance(panels_raw, list) or not panels_raw:
        raise ValueError("The panel chain file must define a non-empty 'panels' list.")

    allowed_keys = {"ifc_guid", "panel_position", "target_position", "next"}
    panels: Dict[str, PanelLink] = {}
    for entry in panels_raw:
        if not isinstance(entry, dict):
            raise ValueError("Each panel entry must be a mapping.")

        extra_keys = set(entry.keys()) - allowed_keys
        if extra_keys:
            raise ValueError(
                f"Panel entry for {entry.get('ifc_guid', '<unknown>')} contains unsupported keys: {sorted(extra_keys)}"
            )

        missing = allowed_keys - set(entry.keys())
        if missing:
            raise ValueError(
                f"Panel entry for {entry.get('ifc_guid', '<unknown>')} is missing keys: {sorted(missing)}"
            )

        ifc_guid = str(entry["ifc_guid"]).strip()
        if not ifc_guid:
            raise ValueError("Panel entries must define a non-empty ifc_guid.")

        if ifc_guid in panels:
            raise ValueError(f"Duplicate panel definition for ifc_guid {ifc_guid} detected.")

        panel_position = point_from_value(entry["panel_position"], field_name="panel_position")
        target_position = point_from_value(entry["target_position"], field_name="target_position")

        next_ifc_guid = entry.get("next")
        if next_ifc_guid is not None:
            next_ifc_guid = str(next_ifc_guid).strip()
            if not next_ifc_guid:
                next_ifc_guid = None

        panels[ifc_guid] = PanelLink(
            ifc_guid=ifc_guid,
            panel_position=panel_position,
            target_position=target_position,
            next_ifc_guid=next_ifc_guid,
        )

    head = resolve_head(panels)

    # Validate that every declared next exists.
    for link in panels.values():
        if link.next_ifc_guid and link.next_ifc_guid not in panels:
            raise ValueError(
                f"Panel {link.ifc_guid} references unknown next GUID {link.next_ifc_guid}."
            )

    return panels, head


class PanelChainExecutor(Node):
    """Publishes panel tasks in the exact order of the NEXT chain."""

    def __init__(self) -> None:
        super().__init__("panel_chain_executor")
        default_chain = Path(
            get_package_share_directory("crane_builder")
        ) / "config" / "example_panels.yaml"
        self.declare_parameter("panel_chain_file", str(default_chain))

        chain_path = Path(
            self.get_parameter("panel_chain_file").get_parameter_value().string_value
        )

        try:
            self._panels, self._current_guid = load_panel_chain(chain_path)
        except Exception as exc:  # noqa: BLE001 - allow ROS to report specific error
            self.get_logger().error(f"Failed to load panel chain: {exc}")
            raise

        self._publisher = self.create_publisher(PanelTask, "panel_task", 10)
        self._timer = self.create_timer(1.0, self._publish_next)
        self.get_logger().info(
            "Panel chain executor ready. Publishing tasks to 'panel_task' with identity orientation."
        )

    def _publish_next(self) -> None:
        if self._current_guid is None:
            self.get_logger().info("Panel chain complete; stopping publication timer.")
            self._timer.cancel()
            return

        link = self._panels[self._current_guid]
        message = PanelTask()
        message.ifc_guid = link.ifc_guid
        message.panel_position = link.panel_position
        message.target_position = link.target_position
        message.next_ifc_guid = link.next_ifc_guid or ""

        self._publisher.publish(message)
        self.get_logger().info(
            f"Dispatched panel {link.ifc_guid} -> next {link.next_ifc_guid or '<end>'}"
        )

        self._current_guid = link.next_ifc_guid


def main() -> None:
    rclpy.init()
    node = PanelChainExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Panel chain executor interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
