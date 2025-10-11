from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from crane_interfaces.msg import PanelTask
from geometry_msgs.msg import Point
from neo4j import GraphDatabase
from neo4j.exceptions import Neo4jError

from .panel_chain_utils import PanelLink, point_from_value, resolve_head


def _validate_identifier(name: str, *, kind: str) -> str:
    if not name:
        raise ValueError(f"{kind} cannot be empty.")
    if not all(char.isalnum() or char == '_' for char in name):
        raise ValueError(
            f"{kind} '{name}' may only contain alphanumeric characters or underscores."
        )
    return name


class Neo4jPanelChainExecutor(Node):
    """Loads the panel chain directly from Neo4j and publishes ``PanelTask`` messages."""

    def __init__(self) -> None:
        super().__init__("neo4j_panel_chain_executor")

        self.declare_parameter("uri", "neo4j+s://ae1083a1.databases.neo4j.io")
        self.declare_parameter("user", "neo4j")
        self.declare_parameter(
            "password", "aVbfVqk-XZ17tl0UOjRLwpfmbV-tCY7JUe7RjnaBbVc"
        )
        self.declare_parameter("database", "")
        self.declare_parameter("panel_label", "FormworkPanel")
        self.declare_parameter("next_relationship", "NEXT")
        self.declare_parameter("publish_period", 1.0)

        uri = self.get_parameter("uri").get_parameter_value().string_value
        user = self.get_parameter("user").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        database = self.get_parameter("database").get_parameter_value().string_value or None
        panel_label = _validate_identifier(
            self.get_parameter("panel_label").get_parameter_value().string_value,
            kind="Panel label",
        )
        relationship = _validate_identifier(
            self.get_parameter("next_relationship").get_parameter_value().string_value,
            kind="Relationship type",
        )
        period = self.get_parameter("publish_period").get_parameter_value().double_value
        if period <= 0:
            raise ValueError("publish_period must be positive.")

        auth: Optional[Tuple[str, str]]
        if user or password:
            auth = (user, password)
        else:
            auth = None

        try:
            self._driver = GraphDatabase.driver(uri, auth=auth)
            self._driver.verify_connectivity()
        except Exception as exc:  # noqa: BLE001 - surface connection issues clearly
            self.get_logger().error(f"Failed to connect to Neo4j at {uri}: {exc}")
            raise

        try:
            (
                self._panels,
                self._current_guid,
                self._sequence_order,
                self._panel_centers,
                self._sequence_indices,
                active_label,
            ) = self._load_panel_chain(panel_label, relationship, database)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load panel chain from Neo4j: {exc}")
            self._driver.close()
            raise

        self._publisher = self.create_publisher(PanelTask, "panel_task", 10)
        self._timer = self.create_timer(period, self._publish_next)

        sequence_report = ", ".join(
            f"{self._sequence_indices[guid]}:{guid}" for guid in self._sequence_order
        )
        self.get_logger().info(
            "Loaded %d panels from Neo4j using label '%s' (%s). Publishing to 'panel_task' with identity orientation.",
            len(self._sequence_order),
            active_label,
            sequence_report,
        )

    def _load_panel_chain(
        self, panel_label: str, relationship: str, database: Optional[str]
    ) -> Tuple[Dict[str, PanelLink], Optional[str], List[str], Dict[str, Point], Dict[str, int], str]:
        def run_query(label: str) -> List[Dict[str, object]]:
            query = f"""
            MATCH (panel:{label})
            WHERE panel.ifcGuid IS NOT NULL AND panel.SequenceIndex IS NOT NULL
            OPTIONAL MATCH (panel)-[:{relationship}]->(next_panel)
            RETURN panel.ifcGuid AS ifc_guid,
                   panel.HookPoint AS hook_point,
                   panel.PanelPosition AS panel_position,
                   panel.TargetPosition AS target_position,
                   panel.SequenceIndex AS sequence_index,
                   next_panel.ifcGuid AS next_ifc_guid
            """

            with self._driver.session(database=database) as session:
                return list(session.run(query))

        panels: Dict[str, PanelLink] = {}
        panel_centers: Dict[str, Point] = {}
        sequence_indices: Dict[str, int] = {}
        used_indices: set[int] = set()

        labels_to_try: List[str] = [panel_label]
        # Always try the Aura dataset's canonical label in addition to whatever
        # the user configured. This covers deployments that still rely on the
        # older :Panel label as well as the more recent :FormworkPanel schema
        # generated by the provided IFC import script.
        for fallback_label in ("FormworkPanel", "Panel"):
            if fallback_label not in labels_to_try:
                labels_to_try.append(fallback_label)

        records: List[Dict[str, object]] = []
        active_label = panel_label

        try:
            for label in labels_to_try:
                records = run_query(label)
                if records:
                    active_label = label
                    if label != panel_label:
                        self.get_logger().warn(
                            "No results found with label '%s'; using fallback label '%s'.",
                            panel_label,
                            label,
                        )
                    break
        except Neo4jError as exc:
            raise RuntimeError(f"Neo4j query failed: {exc}") from exc

        if not records:
            raise ValueError(
                "No panels were returned from Neo4j. Ensure nodes are labelled correctly and include required properties."
                f" Checked labels {labels_to_try!r} and relationship '{relationship}'."
            )

        for record in records:
            ifc_guid_raw = record.get("ifc_guid")
            if ifc_guid_raw is None:
                raise ValueError("A panel without an ifcGuid property was encountered.")
            ifc_guid = str(ifc_guid_raw).strip()
            if not ifc_guid:
                raise ValueError("Panel entries must define a non-empty ifcGuid.")

            if ifc_guid in panels:
                raise ValueError(f"Duplicate panel with ifcGuid {ifc_guid} encountered in Neo4j result.")

            hook_value = record.get("hook_point")
            pick_position = point_from_value(hook_value, field_name=f"{ifc_guid}.HookPoint")

            target_value = record.get("target_position")
            target_position = point_from_value(
                target_value, field_name=f"{ifc_guid}.TargetPosition"
            )

            panel_center_value = record.get("panel_position")
            panel_centers[ifc_guid] = point_from_value(
                panel_center_value, field_name=f"{ifc_guid}.PanelPosition"
            )

            next_guid = record.get("next_ifc_guid")
            if next_guid is not None:
                next_guid = str(next_guid).strip() or None

            sequence_raw = record.get("sequence_index")
            if sequence_raw is None:
                raise ValueError(f"Panel {ifc_guid} is missing SequenceIndex.")
            try:
                sequence_index = int(sequence_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"Panel {ifc_guid} SequenceIndex must be an integer; received {sequence_raw!r}."
                ) from exc

            if sequence_index in used_indices:
                raise ValueError(
                    f"Duplicate SequenceIndex {sequence_index} detected in Neo4j data."
                )

            panels[ifc_guid] = PanelLink(
                ifc_guid=ifc_guid,
                panel_position=pick_position,
                target_position=target_position,
                next_ifc_guid=next_guid,
            )
            sequence_indices[ifc_guid] = sequence_index
            used_indices.add(sequence_index)

        ordered = [guid for guid, _ in sorted(sequence_indices.items(), key=lambda item: item[1])]
        if not ordered:
            raise ValueError("No panels with valid SequenceIndex values were discovered.")

        head = resolve_head(panels)
        if head != ordered[0]:
            raise ValueError(
                "SequenceIndex ordering does not align with the NEXT chain head. Check the Neo4j data consistency."
            )

        for idx, guid in enumerate(ordered):
            expected_next = ordered[idx + 1] if idx + 1 < len(ordered) else None
            actual_next = panels[guid].next_ifc_guid
            if actual_next != expected_next:
                raise ValueError(
                    f"Panel {guid} NEXT chain points to {actual_next or '<end>'} but SequenceIndex expects {expected_next or '<end>'}."
                )

        return panels, head, ordered, panel_centers, sequence_indices, active_label

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

        center = self._panel_centers.get(link.ifc_guid)
        if center:
            self.get_logger().info(
                "Dispatched panel %s (center: %.3f, %.3f, %.3f) -> next %s",
                link.ifc_guid,
                center.x,
                center.y,
                center.z,
                link.next_ifc_guid or "<end>",
            )
        else:
            self.get_logger().info(
                "Dispatched panel %s -> next %s",
                link.ifc_guid,
                link.next_ifc_guid or "<end>",
            )

        self._current_guid = link.next_ifc_guid

    def destroy_node(self) -> bool:
        if hasattr(self, "_driver"):
            self._driver.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = Neo4jPanelChainExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Neo4j panel chain executor interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

