from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from crane_interfaces.msg import PanelTask
from geometry_msgs.msg import Point
from neo4j import GraphDatabase

from .panel_chain_utils import PanelLink, point_from_value


class Neo4jPanelChainExecutor(Node):
    """Load panel chains from Neo4j using level + mode specific queries."""

    def __init__(self) -> None:
        super().__init__("neo4j_panel_chain_executor")

        self.declare_parameter("uri", "neo4j+s://ae1083a1.databases.neo4j.io")
        self.declare_parameter("user", "neo4j")
        self.declare_parameter(
            "password", "aVbfVqk-XZ17tl0UOjRLwpfmbV-tCY7JUe7RjnaBbVc"
        )
        self.declare_parameter("database", "")
        self.declare_parameter("level_name", "")
        self.declare_parameter("mode", "Wall")
        self.declare_parameter("publish_period", 1.0)

        uri = self.get_parameter("uri").get_parameter_value().string_value
        user = self.get_parameter("user").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        database = self.get_parameter("database").get_parameter_value().string_value or None

        requested_level = (
            self.get_parameter("level_name").get_parameter_value().string_value.strip()
        )

        mode_param = self.get_parameter("mode").get_parameter_value().string_value.strip()
        if not mode_param:
            raise ValueError("mode parameter must be provided.")
        mode = mode_param.lower()
        if mode not in {"wall", "column"}:
            raise ValueError("mode must be either 'Wall' or 'Column'.")

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
            level_name = requested_level or self._select_default_level(mode, database)
        except Exception as exc:  # noqa: BLE001 - propagate discovery issues clearly
            self.get_logger().error(f"Failed to determine a level to query: {exc}")
            self._driver.close()
            raise

        if not level_name:
            self._driver.close()
            raise ValueError(
                "Neo4j did not return any levels containing panels for the requested mode."
            )

        if not requested_level:
            self.get_logger().info(
                f"No level_name parameter supplied; defaulting to level '{level_name}' discovered in Neo4j."
            )

        try:
            (
                self._panels,
                self._sequence_order,
                self._panel_centers,
            ) = self._load_panel_chain(level_name, mode, database)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load panel chain from Neo4j: {exc}")
            self._driver.close()
            raise

        self._mode = "Wall" if mode == "wall" else "Column"
        self._level_name = level_name
        self._next_index = 0

        self._publisher = self.create_publisher(PanelTask, "panel_task", 10)
        self._timer = self.create_timer(period, self._publish_next)

        ordered_ifc_guids = ", ".join(self._sequence_order)
        self.get_logger().info(
            "Loaded %d panels from Neo4j for level '%s' (%s mode). Publishing to 'panel_task' with identity orientation. Order: %s"
            % (len(self._sequence_order), self._level_name, self._mode, ordered_ifc_guids)
        )

    def _select_default_level(self, mode: str, database: Optional[str]) -> str:
        """Return the first level that contains panels for the requested mode."""

        if mode == "wall":
            query = """
            MATCH (lvl:Level)-[:HAS_WALL]->(:Wall)-[:HAS_PART]->(:WallPart)
            MATCH (:FormworkPanel)-[:HOSTED_BY]->(:WallPart)
            RETURN DISTINCT lvl.name AS level_name
            ORDER BY level_name
            LIMIT 1
            """
        else:
            query = """
            MATCH (lvl:Level)-[:HAS_COLUMN]->(:Column)
            MATCH (:FormworkPanel)-[:HOSTED_BY]->(:Column)
            RETURN DISTINCT lvl.name AS level_name
            ORDER BY level_name
            LIMIT 1
            """

        with self._driver.session(database=database) as session:
            record = session.run(query).single()

        if not record:
            raise ValueError(
                "Neo4j contains no levels with FormworkPanel nodes linked via the required relationships."
            )

        level_name = (record.get("level_name") or "").strip()
        if not level_name:
            raise ValueError(
                "Neo4j returned a level without a name while selecting a default level."
            )

        return level_name

    def _load_panel_chain(
        self, level_name: str, mode: str, database: Optional[str]
    ) -> Tuple[Dict[str, PanelLink], List[str], Dict[str, Point]]:
        wall_query = """
        MATCH (lvl:Level {name: $level_name})-[:HAS_WALL]->(:Wall)-[:HAS_PART]->(part:WallPart)
        MATCH (head:FormworkPanel)-[:HOSTED_BY]->(part)
        WHERE NOT ()-[:NEXT]->(head)
        WITH DISTINCT head
        MATCH path=(head)-[:NEXT*0..]->(tail:FormworkPanel)
        WHERE NOT (tail)-[:NEXT]->(:FormworkPanel)
        WITH head, nodes(path) AS node_list
        RETURN head.ifcGuid AS head_guid,
               [n IN node_list | n.ifcGuid] AS chain_id,
               [n IN node_list | {
                   ifcGuid: n.ifcGuid,
                   HookPoint: n.HookPoint,
                   PanelPosition: n.PanelPosition,
                   TargetPosition: n.TargetPosition
               }] AS panels
        ORDER BY head_guid
        """

        column_query = """
        MATCH (lvl:Level {name: $level_name})-[:HAS_COLUMN]->(host:Column)
        MATCH (head:FormworkPanel)-[:HOSTED_BY]->(host)
        WHERE NOT ()-[:NEXT]->(head)
        WITH DISTINCT head
        MATCH path=(head)-[:NEXT*0..]->(tail:FormworkPanel)
        WHERE NOT (tail)-[:NEXT]->(:FormworkPanel)
        WITH head, nodes(path) AS node_list
        RETURN head.ifcGuid AS head_guid,
               [n IN node_list | n.ifcGuid] AS chain_id,
               [n IN node_list | {
                   ifcGuid: n.ifcGuid,
                   HookPoint: n.HookPoint,
                   PanelPosition: n.PanelPosition,
                   TargetPosition: n.TargetPosition
               }] AS panels
        ORDER BY head_guid
        """

        query = wall_query if mode == "wall" else column_query

        with self._driver.session(database=database) as session:
            records = list(session.run(query, level_name=level_name))

        if not records:
            raise ValueError(
                "Neo4j returned no panels for the requested level and mode. Verify the level name and relationships."
            )

        panels: Dict[str, PanelLink] = {}
        panel_centers: Dict[str, Point] = {}
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
                target_point = point_from_value(
                    panel_data.get("TargetPosition"), field_name=f"{guid}.TargetPosition"
                )
                panel_center = point_from_value(
                    panel_data.get("PanelPosition"), field_name=f"{guid}.PanelPosition"
                )

                next_guid = chain_ids[idx + 1] if idx + 1 < len(chain_ids) else None

                panels[guid] = PanelLink(
                    ifc_guid=guid,
                    panel_position=hook_point,
                    target_position=target_point,
                    next_ifc_guid=next_guid,
                )
                panel_centers[guid] = panel_center

            sequence_order.extend(chain_ids)

        if not sequence_order:
            raise ValueError("No panels were assembled from Neo4j records.")

        return panels, sequence_order, panel_centers

    def _publish_next(self) -> None:
        if self._next_index >= len(self._sequence_order):
            self.get_logger().info("Panel chain complete; stopping publication timer.")
            self._timer.cancel()
            return

        guid = self._sequence_order[self._next_index]
        link = self._panels[guid]

        message = PanelTask()
        message.ifc_guid = link.ifc_guid
        message.panel_position = link.panel_position
        message.target_position = link.target_position
        message.next_ifc_guid = link.next_ifc_guid or ""

        self._publisher.publish(message)

        center = self._panel_centers.get(link.ifc_guid)
        if center:
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
        else:
            self.get_logger().info(
                "Dispatched panel %s -> next %s"
                % (link.ifc_guid, link.next_ifc_guid or "<end>")
            )

        self._next_index += 1
        if self._next_index >= len(self._sequence_order):
            self.get_logger().info("Panel chain complete; stopping publication timer.")
            self._timer.cancel()

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
