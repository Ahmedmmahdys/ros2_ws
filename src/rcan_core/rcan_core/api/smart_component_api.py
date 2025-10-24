"""Smart component API surface for installers."""
from __future__ import annotations

from typing import Optional

from .. import msg_broker

REQUEST_TOPIC = "smart.install_panel.request"


def receive_install_request(host_id: Optional[str], ifcguid: Optional[str]) -> None:
    """Handle an incoming install panel request."""
    payload = {"host_id": host_id, "ifcguid": ifcguid}
    msg_broker.publish(REQUEST_TOPIC, payload)


__all__ = ["receive_install_request", "REQUEST_TOPIC"]
