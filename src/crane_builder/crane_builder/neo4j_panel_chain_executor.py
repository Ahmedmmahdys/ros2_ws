from __future__ import annotations

import math
import re

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from crane_interfaces.msg import PanelTask
from neo4j import GraphDatabase

from .panel_chain_utils import PanelLink, point_from_value


class Neo4jPanelChainExecutor(Node):
    """Load panel chains from Neo4j using NEXT_* relation selection."""

    def __init__(self) -> None:
        super().__init__("neo4j_panel_chain_executor")

        self.declare_parameter("uri", "neo4j+s://ae1083a1.databases.neo4j.io")
        self.declare_parameter("user", "neo4j")
        self.declare_parameter(
            "password", "aVbfVqk-XZ17tl0UOjRLwpfmbV-tCY7JUe7RjnaBbVc"
        )
        self.declare_parameter("database", "")
        self.declare_parameter("chain_relation", "NEXT_1")
        self.declare_parameter("publish_period", 1.0)

        uri = self.get_parameter("uri").get_parameter_value().string_value
        user = self.get_parameter("user").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        database = self.get_parameter("database").get_parameter_value().string_value or None

        relation_param = (
            self.get_parameter("chain_relation").get_parameter_value().string_value.strip()
        )
        if not relation_param:
            raise ValueError("chain_relation parameter must be provided and non-empty.")
        relation_name = self._normalise_relation_name(relation_param)

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
            self._panels, self._sequence_order = self._load_panel_chain(
                relation_name, database
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load panel chain from Neo4j: {exc}")
            self._driver.close()
            raise

        self._relation = relation_name
        self._next_index = 0

        self._publisher = self.create_publisher(PanelTask, "/panel_task", 10)
        self._timer = self.create_timer(period, self._publish_next)

        ordered_ifc_guids = ", ".join(self._sequence_order)
        self.get_logger().info(
            "Loaded %d panels from Neo4j using relation '%s'. Publishing to '/panel_task' with identity orientation. Order: %s"
            % (len(self._sequence_order), self._relation, ordered_ifc_guids)
        )

    def _normalise_relation_name(self, relation: str) -> str:
        """Validate the requested relation name and return it in Neo4j format."""

        candidate = relation.strip()
        if not candidate:
            raise ValueError("Relation name must be non-empty after stripping whitespace.")

        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", candidate):
            raise ValueError(
                "Relation names may only contain letters, digits, and underscores and cannot start with a digit."
            )

        return candidate

    def _load_panel_chain(
        self, relation: str, database: Optional[str]
    ) -> Tuple[Dict[str, PanelLink], List[str]]:
        query = f"""
        MATCH (head:FormworkPanel)
        WHERE NOT ()-[:{relation}]->(head)
        WITH DISTINCT head
        MATCH path=(head)-[:{relation}*0..]->(tail:FormworkPanel)
        WHERE NOT (tail)-[:{relation}]->(:FormworkPanel)
        WITH head, nodes(path) AS node_list
        RETURN head.ifcGuid AS head_guid,
               [n IN node_list | n.ifcGuid] AS chain_id,
               [n IN node_list | {{
                   ifcGuid: n.ifcGuid,
                   HookPoint: n.HookPoint,
                   PanelPosition: n.PanelPosition,
                   TargetPosition: n.TargetPosition
               }}] AS panels
        ORDER BY head_guid
        """

        with self._driver.session(database=database) as session:
            available_relations = self._fetch_available_relations(session)
            if available_relations and relation not in available_relations:
                raise ValueError(
                    "Relation '%s' does not exist in Neo4j. Available relations include: %s"
                    % (relation, ", ".join(sorted(available_relations)))
                )
            records = list(session.run(query))

        if not records:
            raise ValueError(
                f"Neo4j returned no panels connected by relation '{relation}'. Verify the relation name and NEXT_* links."
            )

        panels: Dict[str, PanelLink] = {}
        sequence_order: List[str] = []

        for record in records:
            chain_values = record.get("chain_id")
            panel_entries = record.get("panels")
            head_guid = record.get("head_guid")

            if not chain_values or not panel_entries:
                raise ValueError(
                    "Neo4j returned an incomplete chain (missing identifiers or panel data)."
                )

            if len(chain_values) != len(panel_entries):
                raise ValueError("Mismatch between chain identifiers and panel data from Neo4j.")

            chain_ids = [str(guid).strip() for guid in chain_values]

            if head_guid and head_guid != chain_ids[0]:
                raise ValueError("Head GUID does not match the first element in the returned chain.")

            for idx, guid in enumerate(chain_ids):
                if not guid:
                    raise ValueError("Encountered a panel with an empty ifcGuid in Neo4j data.")
                if guid in panels:
                    raise ValueError(
                        f"Duplicate panel {guid} detected across returned chains. Ensure NEXT chains do not overlap."
                    )

                panel_data = panel_entries[idx]
                if not isinstance(panel_data, dict):
                    raise TypeError("Unexpected panel data format returned by Neo4j.")

                hook_point = point_from_value(
                    panel_data.get("HookPoint"), field_name=f"{guid}.HookPoint"
                )
                panel_center = point_from_value(
                    panel_data.get("PanelPosition"), field_name=f"{guid}.PanelPosition"
                )
                target_point = point_from_value(
                    panel_data.get("TargetPosition"), field_name=f"{guid}.TargetPosition"
                )

                next_guid = chain_ids[idx + 1] if idx + 1 < len(chain_ids) else None

                panels[guid] = PanelLink(
                    ifc_guid=guid,
                    hook_point=hook_point,
                    panel_position=panel_center,
                    target_position=target_point,
                    next_ifc_guid=next_guid,
                )

            sequence_order.extend(chain_ids)

        if not sequence_order:
            raise ValueError("No panels were assembled from Neo4j records.")

        return panels, sequence_order

    def _fetch_available_relations(self, session) -> List[str]:
        """Return the list of relationship types available in the database."""

        try:
            result = list(session.run("CALL db.relationshipTypes()"))
        except Exception as exc:  # noqa: BLE001 - log and continue without the hint
            self.get_logger().warning(
                f"Unable to list relationship types from Neo4j: {exc}"
            )
            return []

        relations: List[str] = []
        for record in result:
            value = record.get("relationshipType")
            if isinstance(value, str):
                relations.append(value)

        return relations

    def _publish_next(self) -> None:
        if self._next_index >= len(self._sequence_order):
            self.get_logger().info("Panel chain complete; stopping publication timer.")
            self._timer.cancel()
            return

        guid = self._sequence_order[self._next_index]
        link = self._panels[guid]

        if not self._valid_point(link.hook_point) or not self._valid_point(link.target_position):
            self.get_logger().error(
                f"Skipping panel {guid} due to invalid hook or target coordinates."
            )
            self._next_index += 1
            return

        message = PanelTask()
        message.ifc_guid = link.ifc_guid
        message.hook_point = link.hook_point
        message.panel_position = link.panel_position
        message.target_position = link.target_position
        message.next_ifc_guid = link.next_ifc_guid or ""

        self._publisher.publish(message)

        center = link.panel_position
        self.get_logger().info(
            "Dispatched panel %s (center: %.3f, %.3f, %.3f) -> next %s"
            % (
                link.ifc_guid,
                center.x,
                center.y,
                center.z,
                link.next_ifc_guid or "<end>",
            )
        )

        self._next_index += 1
        if self._next_index >= len(self._sequence_order):
            self.get_logger().info("Panel chain complete; stopping publication timer.")
            self._timer.cancel()

    def _valid_point(self, point) -> bool:
        return all(math.isfinite(value) for value in (point.x, point.y, point.z))

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
