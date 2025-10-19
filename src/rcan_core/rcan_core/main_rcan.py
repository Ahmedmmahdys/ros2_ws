"""Entry point for orchestrating RCAN services."""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from typing import Optional

from . import msg_broker
from .api import smart_component_api
from .services import NodeGenerator, RobotExecutor, StateManager, TaskAllocator

LOGGER = logging.getLogger(__name__)


@dataclass
class Orchestrator:
    task_allocator: TaskAllocator
    node_generator: NodeGenerator
    robot_executor: RobotExecutor
    state_manager: StateManager


def configure_logging() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(name)s: %(message)s")


def create_orchestrator() -> Orchestrator:
    msg_broker.reset()
    return Orchestrator(
        task_allocator=TaskAllocator(),
        node_generator=NodeGenerator(),
        robot_executor=RobotExecutor(),
        state_manager=StateManager(),
    )


def run_demo(host_id: Optional[str] = None, ifcguid: str = "PUT_GUID_HERE") -> Orchestrator:
    configure_logging()
    orchestrator = create_orchestrator()
    LOGGER.info("Submitting install request for panel %s", ifcguid)
    smart_component_api.receive_install_request(host_id=host_id, ifcguid=ifcguid)
    return orchestrator


def main() -> None:
    demo_ifcguid = os.getenv("RCAN_DEMO_IFCGUID", "PUT_GUID_HERE")
    run_demo(ifcguid=demo_ifcguid)


if __name__ == "__main__":
    main()
