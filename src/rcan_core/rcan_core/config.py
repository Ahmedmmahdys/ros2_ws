"""Configuration for the RCAN core services."""
from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Neo4jConfig:
    uri: str
    user: str
    password: str


NEO4J = Neo4jConfig(
    uri=os.getenv("RCAN_NEO4J_URI", "bolt://localhost:7687"),
    user=os.getenv("RCAN_NEO4J_USER", "neo4j"),
    password=os.getenv("RCAN_NEO4J_PASSWORD", "neo4j"),
)

ROBOT_NAME = os.getenv("RCAN_ROBOT_NAME", "tower_crane_1")

__all__ = ["NEO4J", "Neo4jConfig", "ROBOT_NAME"]
