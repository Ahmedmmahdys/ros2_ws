"""Neo4j database access helpers."""
from __future__ import annotations

import ast
import logging
from typing import Any, Dict, Optional

try:  # pragma: no cover - import guard for test environments
    from neo4j import GraphDatabase
    from neo4j import Driver
except ImportError:  # pragma: no cover - driver optional for tests
    GraphDatabase = None  # type: ignore[assignment]
    Driver = Any  # type: ignore[assignment]

from . import config
from .models import Panel, Vec3

LOGGER = logging.getLogger(__name__)

def _panel_return(alias: str) -> str:
    return (
        f"{alias}.ifcguid AS ifcguid, "
        f"{alias}.HookPoint AS hook, "
        f"{alias}.TargetPosition AS target, "
        f"{alias}.date AS date"
    )


_FETCH_PANEL_BY_GUID = (
    "MATCH (p:Panel {ifcguid:$ifcguid})\n"
    "RETURN "
    f"{_panel_return('p')}"
    ";"
)
_FETCH_NEXT_READY_PANEL = (
    "MATCH (h:Host {id:$host_id})-[:HAS_PANEL]->(start:Panel)\n"
    "WHERE NOT EXISTS { (:Panel)-[incoming]->(start) WHERE type(incoming) STARTS WITH 'NEXT_' }\n"
    "CALL {\n"
    "    WITH start\n"
    "    MATCH path = (start)-[rels*0..]->(candidate:Panel)\n"
    "    WHERE coalesce(candidate.state,'') <> 'Installed'\n"
    "      AND all(rel IN rels WHERE type(rel) STARTS WITH 'NEXT_')\n"
    "      AND CASE\n"
    "            WHEN size(rels) = 0 THEN true\n"
    "            ELSE all(idx IN range(0, size(rels) - 1) WHERE\n"
    "                type(rels[idx]) = 'NEXT_' + toString(idx + 1)\n"
    "            )\n"
    "          END\n"
    "    RETURN candidate, size(rels) AS depth\n"
    "    ORDER BY depth ASC\n"
    "    LIMIT 1\n"
    "}\n"
    "WITH candidate AS p, depth\n"
    "RETURN "
    f"{_panel_return('p')}"
    ", depth\n"
    "ORDER BY depth ASC\n"
    "LIMIT 1;"
)
_MARK_PANEL_STATE = (
    "MATCH (p:Panel {ifcguid:$ifcguid}) SET p.state = $state;"
)

_driver: Optional[Driver] = None


def get_driver() -> Driver:
    """Return a cached Neo4j driver instance."""
    global _driver
    if _driver is None:
        if GraphDatabase is None:
            raise RuntimeError("The neo4j Python driver is required to connect to the database")
        LOGGER.debug("Creating Neo4j driver for %s", config.NEO4J.uri)
        _driver = GraphDatabase.driver(
            config.NEO4J.uri,
            auth=(config.NEO4J.user, config.NEO4J.password),
        )
    return _driver


def close_driver() -> None:
    global _driver
    if _driver is not None:
        _driver.close()
        _driver = None


def _parse_vec3(value: Any) -> Vec3:
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return float(value[0]), float(value[1]), float(value[2])
    if isinstance(value, str):
        try:
            parsed = ast.literal_eval(value)
        except (ValueError, SyntaxError) as exc:
            raise ValueError(f"Invalid vector string: {value!r}") from exc
        return _parse_vec3(parsed)
    raise ValueError(f"Unsupported vector format: {value!r}")


def _record_to_panel(record: Dict[str, Any]) -> Panel:
    date_value = record.get("date")
    panel_date = str(date_value) if date_value is not None else None
    return Panel(
        ifcguid=str(record["ifcguid"]),
        hook=_parse_vec3(record["hook"]),
        target=_parse_vec3(record["target"]),
        date=panel_date,
    )


def _run_single(query: str, parameters: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    driver = get_driver()
    with driver.session() as session:
        result = session.run(query, **parameters)
        return result.single()


def get_panel_by_guid(ifcguid: str) -> Panel:
    record = _run_single(_FETCH_PANEL_BY_GUID, {"ifcguid": ifcguid})
    if record is None:
        raise LookupError(f"Panel {ifcguid} not found")
    return _record_to_panel(record)


def get_next_ready_panel(host_id: Optional[str]) -> Panel:
    if host_id is None:
        raise LookupError("Host identifier is required to select the next ready panel")
    record = _run_single(_FETCH_NEXT_READY_PANEL, {"host_id": host_id})
    if record is None:
        raise LookupError("No ready panel found")
    return _record_to_panel(record)


def mark_panel_state(ifcguid: str, state: str) -> None:
    driver = get_driver()
    with driver.session() as session:
        session.run(_MARK_PANEL_STATE, ifcguid=ifcguid, state=state)
    LOGGER.info("Panel %s state set to %s", ifcguid, state)


__all__ = [
    "close_driver",
    "get_driver",
    "get_next_ready_panel",
    "get_panel_by_guid",
    "mark_panel_state",
]
