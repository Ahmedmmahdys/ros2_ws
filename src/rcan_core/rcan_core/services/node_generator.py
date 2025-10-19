"""Generate ROS2 nodes for activities based on templates."""
from __future__ import annotations

import os
import uuid
from importlib import resources
from pathlib import Path
from typing import Dict

import logging

from .. import msg_broker

READY_TOPIC = "services.activity.ready"
GENERATED_TOPIC = "services.activity.generated"

LOGGER = logging.getLogger(__name__)


class NodeGenerator:
    """Render Python ROS2 nodes from templates when activities are ready."""

    def __init__(self) -> None:
        msg_broker.subscribe(READY_TOPIC, self._handle_ready)

    def _get_generated_dir(self) -> Path:
        override = os.getenv("RCAN_NODES_GENERATED_DIR")
        if override:
            path = Path(override)
        else:
            try:
                import rcan_nodes

                path = Path(rcan_nodes.__file__).resolve().parent / "generated"
            except ModuleNotFoundError:
                path = Path(__file__).resolve().parents[3] / "rcan_nodes" / "rcan_nodes" / "generated"
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _render_template(self, context: Dict[str, str]) -> str:
        try:
            template = resources.files("rcan_nodes").joinpath("templates/install_panel_node.py.j2").read_text()
        except ModuleNotFoundError:
            template_path = Path(__file__).resolve().parents[3] / "rcan_nodes" / "rcan_nodes" / "templates" / "install_panel_node.py.j2"
            template = template_path.read_text()
        rendered = template
        for key, value in context.items():
            rendered = rendered.replace(f"{{{{ {key} }}}}", value)
        return rendered

    def _handle_ready(self, payload: Dict[str, object]) -> None:
        panel = payload["panel"]
        ifcguid = str(panel["ifcguid"])
        activity_id = uuid.uuid4().hex
        context = {
            "activity_id": activity_id,
            "ifcguid": ifcguid,
            "hook_pose": repr(tuple(panel["hook_pose"])),
            "target_pose": repr(tuple(panel["target_pose"])),
        }
        code = self._render_template(context)
        target_path = self._get_generated_dir() / f"install_panel_{ifcguid}.py"
        target_path.write_text(code)
        LOGGER.info("Generated node for %s at %s", ifcguid, target_path)
        msg_broker.publish(
            GENERATED_TOPIC,
            {
                "activity_id": activity_id,
                "ifcguid": ifcguid,
                "node_path": str(target_path),
                "panel": panel,
            },
        )


__all__ = ["GENERATED_TOPIC", "NodeGenerator", "READY_TOPIC"]
