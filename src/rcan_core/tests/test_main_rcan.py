from __future__ import annotations

import os
import runpy
from pathlib import Path

import pytest

from rcan_core.api import database_api
from rcan_core.models import Panel
from rcan_core import msg_broker
from rcan_core import cli


@pytest.fixture(autouse=True)
def reset_broker():
    msg_broker.reset()
    yield
    msg_broker.reset()


@pytest.fixture(autouse=True)
def reset_cli_orchestrator():
    cli._ORCHESTRATOR = None
    yield
    cli._ORCHESTRATOR = None


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


def test_cli_panel_chain_requests_panels(monkeypatch, tmp_path):
    panel_sequence = [
        Panel(ifcguid="A1", hook=(1.0, 0.0, 0.0), target=(0.0, 1.0, 0.0)),
        Panel(ifcguid="B2", hook=(2.0, 0.0, 0.0), target=(0.0, 2.0, 0.0)),
    ]
    panel_lookup = {panel.ifcguid: panel for panel in panel_sequence}

    def fake_next(host_id):
        if not panel_sequence:
            raise LookupError("No panels left")
        return panel_sequence.pop(0)

    states: list[tuple[str, str]] = []

    def fake_mark(ifcguid: str, state: str) -> None:
        states.append((ifcguid, state))

    monkeypatch.setattr(database_api, "get_next_ready_panel", fake_next)
    monkeypatch.setattr(database_api, "get_panel_by_guid", lambda ifcguid: panel_lookup[ifcguid])
    monkeypatch.setattr(database_api, "mark_panel_state", fake_mark)

    monkeypatch.setenv("RCAN_SIMULATE_ROS_RUN", "1")
    monkeypatch.setenv("RCAN_NODES_GENERATED_DIR", str(tmp_path))

    count = cli.run_panel_chain(host_id="HOST_1")

    assert count == 2
    assert states == [
        ("A1", "Requested"),
        ("A1", "Installed"),
        ("B2", "Requested"),
        ("B2", "Installed"),
    ]
