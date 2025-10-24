"""Thin wrapper around the database layer for services."""
from __future__ import annotations

from typing import Optional

from .. import db_api
from ..models import Panel


def get_panel_by_guid(ifcguid: str) -> Panel:
    return db_api.get_panel_by_guid(ifcguid)


def get_next_ready_panel(host_id: Optional[str]) -> Panel:
    return db_api.get_next_ready_panel(host_id)


def mark_panel_state(ifcguid: str, state: str) -> None:
    db_api.mark_panel_state(ifcguid, state)


__all__ = ["get_panel_by_guid", "get_next_ready_panel", "mark_panel_state"]
