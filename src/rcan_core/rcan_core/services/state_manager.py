"""State manager that updates panel state based on robot feedback."""
from __future__ import annotations

import logging
from typing import Dict

from .. import msg_broker
from ..api import database_api

ROS_STATE_TOPIC = "ros.robot.state"
COMPONENT_STATE_TOPIC = "component.panel.state"

LOGGER = logging.getLogger(__name__)


class StateManager:
    """Handle state transitions for panels based on robot feedback."""

    def __init__(self) -> None:
        msg_broker.subscribe(ROS_STATE_TOPIC, self._handle_robot_state)

    def _handle_robot_state(self, payload: Dict[str, object]) -> None:
        ifcguid = payload.get("ifcguid")
        status = payload.get("status")
        if not ifcguid:
            LOGGER.error("Robot state missing ifcguid: %s", payload)
            return

        if status == "success":
            database_api.mark_panel_state(ifcguid, "Installed")
            component_state = "Installed"
        elif status == "failed":
            database_api.mark_panel_state(ifcguid, "Failed")
            component_state = "Failed"
        else:
            LOGGER.warning("Unknown status '%s' for panel %s", status, ifcguid)
            return

        msg_broker.publish(COMPONENT_STATE_TOPIC, {"ifcguid": ifcguid, "state": component_state})


__all__ = ["COMPONENT_STATE_TOPIC", "ROS_STATE_TOPIC", "StateManager"]
