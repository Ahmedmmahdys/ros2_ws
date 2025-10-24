"""Entry point for orchestrating RCAN services."""
from __future__ import annotations

import logging
from dataclasses import dataclass

from . import msg_broker
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


def main() -> None:
    configure_logging()
    create_orchestrator()
    LOGGER.info(
        "RCAN orchestrator initialized. Submit install requests via the RCAN CLI or API."
    )


if __name__ == "__main__":
    main()
