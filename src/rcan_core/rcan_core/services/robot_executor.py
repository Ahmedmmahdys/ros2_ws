"""Robot executor launching generated ROS nodes."""
from __future__ import annotations

import os
import subprocess
import logging
from typing import Dict

from .. import msg_broker

GENERATED_TOPIC = "services.activity.generated"
RUNNING_TOPIC = "services.activity.running"
ROS_STATE_TOPIC = "ros.robot.state"

LOGGER = logging.getLogger(__name__)


class RobotExecutor:
    """Launch generated ROS2 nodes to execute panel installations."""

    def __init__(self) -> None:
        msg_broker.subscribe(GENERATED_TOPIC, self._handle_generated)

    def _handle_generated(self, payload: Dict[str, object]) -> None:
        ifcguid = str(payload["ifcguid"])
        activity_id = str(payload["activity_id"])
        msg_broker.publish(RUNNING_TOPIC, {"activity_id": activity_id, "ifcguid": ifcguid})
        if os.getenv("RCAN_SIMULATE_ROS_RUN") == "1":
            LOGGER.info("Simulating ROS execution for %s", ifcguid)
            msg_broker.publish(
                ROS_STATE_TOPIC,
                {"activity_id": activity_id, "ifcguid": ifcguid, "status": "success"},
            )
            return

        try:
            subprocess.Popen(
                ["ros2", "run", "rcan_nodes", f"install_panel_{ifcguid}"],
                env=os.environ.copy(),
            )
        except FileNotFoundError as exc:  # pragma: no cover - depends on ros2
            LOGGER.error("Failed to launch ROS2 node for %s: %s", ifcguid, exc)
        except Exception as exc:  # pragma: no cover - safety
            LOGGER.exception("Unexpected error launching activity %s: %s", ifcguid, exc)


__all__ = ["GENERATED_TOPIC", "RobotExecutor", "ROS_STATE_TOPIC", "RUNNING_TOPIC"]
