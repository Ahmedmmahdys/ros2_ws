from __future__ import annotations

import os
import runpy
from pathlib import Path

import pytest

from rcan_core.api import database_api
from rcan_core.models import Panel
from rcan_core import msg_broker


@pytest.fixture(autouse=True)
def reset_broker():
    msg_broker.reset()
    yield
    msg_broker.reset()


def test_main_rcan_generates_node_and_updates_state(monkeypatch, tmp_path):
    panel = Panel(ifcguid="X1", hook=(1.0, 2.0, 3.0), target=(4.0, 5.0, 6.0))

    states: list[tuple[str, str]] = []
    monkeypatch.setattr(database_api, "get_panel_by_guid", lambda ifcguid: panel)
    monkeypatch.setattr(database_api, "get_next_ready_panel", lambda host_id: panel)

    def fake_mark(ifcguid: str, state: str) -> None:
        states.append((ifcguid, state))

    monkeypatch.setattr(database_api, "mark_panel_state", fake_mark)

    published = []
    original_publish = msg_broker.publish

    def spy_publish(topic: str, payload: dict) -> None:
        if topic == "component.panel.state":
            published.append(payload)
        original_publish(topic, payload)

    monkeypatch.setattr(msg_broker, "publish", spy_publish)

    monkeypatch.setenv("RCAN_SIMULATE_ROS_RUN", "1")
    monkeypatch.setenv("RCAN_NODES_GENERATED_DIR", str(tmp_path))
    monkeypatch.setenv("RCAN_DEMO_IFCGUID", "X1")

    runpy.run_module("rcan_core.main_rcan", run_name="__main__")

    node_path = Path(tmp_path) / "install_panel_X1.py"
    assert node_path.exists()
    assert states == [("X1", "Requested"), ("X1", "Installed")]
    assert published and published[-1] == {"ifcguid": "X1", "state": "Installed"}
