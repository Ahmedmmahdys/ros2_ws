"""Task allocation service for panel installation requests."""
from __future__ import annotations

import logging
from typing import Dict, Optional

from .. import msg_broker
from .. import config as rcan_config
from ..api import database_api
from ..models import Panel

LOGGER = logging.getLogger(__name__)

REQUEST_TOPIC = "smart.install_panel.request"
READY_TOPIC = "services.activity.ready"


class TaskAllocator:
    """Subscribe to install requests and select panels to install."""

    def __init__(self) -> None:
        msg_broker.subscribe(REQUEST_TOPIC, self._handle_request)

    def _select_panel(self, host_id: Optional[str], ifcguid: Optional[str]) -> Panel:
        if ifcguid:
            return database_api.get_panel_by_guid(ifcguid)
        return database_api.get_next_ready_panel(host_id)

    def _handle_request(self, payload: Dict[str, object]) -> None:
        host_id = payload.get("host_id")
        ifcguid = payload.get("ifcguid")
        LOGGER.info("Received install request host_id=%s ifcguid=%s", host_id, ifcguid)
        try:
            panel = self._select_panel(host_id, ifcguid)
        except LookupError as exc:
            LOGGER.error("No panel available: %s", exc)
            return

        database_api.mark_panel_state(panel.ifcguid, "Requested")
        message = {
            "panel": {
                "ifcguid": panel.ifcguid,
                "hook_pose": list(panel.hook),
                "target_pose": list(panel.target),
            },
            "host_id": host_id or rcan_config.ROBOT_NAME,
        }
        if panel.date is not None:
            message["panel"]["date"] = panel.date
        msg_broker.publish(READY_TOPIC, message)


__all__ = ["READY_TOPIC", "REQUEST_TOPIC", "TaskAllocator"]
