"""Command line interface for RCAN orchestration."""
from __future__ import annotations

import argparse
import logging
from typing import Iterable, Optional

from .api import smart_component_api, database_api
from . import config as rcan_config
from . import main_rcan

LOGGER = logging.getLogger(__name__)

_ORCHESTRATOR = None


def _ensure_orchestrator() -> main_rcan.Orchestrator:
    """Create the orchestrator exactly once so services are wired."""
    global _ORCHESTRATOR
    if _ORCHESTRATOR is None:
        main_rcan.configure_logging()
        _ORCHESTRATOR = main_rcan.create_orchestrator()
    return _ORCHESTRATOR


def _request_panel(host_id: Optional[str], ifcguid: str) -> None:
    LOGGER.info("Submitting install request for panel %s", ifcguid)
    smart_component_api.receive_install_request(host_id=host_id, ifcguid=ifcguid)


def run_panel_chain(host_id: Optional[str], limit: Optional[int] = None) -> int:
    """Sequentially request panel installations for a host using RCAN services."""
    _ensure_orchestrator()
    processed = 0
    seen: set[str] = set()
    while True:
        if limit is not None and processed >= limit:
            break
        try:
            panel = database_api.get_next_ready_panel(host_id)
        except LookupError:
            LOGGER.info(
                "No additional panels ready for host %s",
                host_id or rcan_config.ROBOT_NAME,
            )
            break
        if panel.ifcguid in seen:
            LOGGER.warning("Panel %s already requested in this session; stopping to avoid a loop", panel.ifcguid)
            break
        _request_panel(host_id, panel.ifcguid)
        seen.add(panel.ifcguid)
        processed += 1
    return processed


def run_single_install(host_id: Optional[str], ifcguid: str) -> None:
    """Request installation of a single panel."""
    _ensure_orchestrator()
    _request_panel(host_id, ifcguid)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RCAN orchestration utilities")
    sub = parser.add_subparsers(dest="command", required=True)

    demo = sub.add_parser("demo", help="Run the demo orchestrator flow for a single panel")
    demo.add_argument("--ifcguid", required=True, help="Panel IFC GUID to install")
    demo.add_argument("--host", help="Optional host identifier")

    install = sub.add_parser("install", help="Request a single panel installation")
    install.add_argument("--ifcguid", required=True, help="Panel IFC GUID to install")
    install.add_argument("--host", help="Optional host identifier")

    chain = sub.add_parser("panel-chain", help="Sequentially install all ready panels for a host")
    chain.add_argument("--host", help="Host identifier to source panels from")
    chain.add_argument("--limit", type=int, help="Maximum number of panels to request")

    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="[%(levelname)s] %(name)s: %(message)s",
    )

    if args.command == "demo":
        main_rcan.run_demo(host_id=args.host, ifcguid=args.ifcguid)
        return 0

    if args.command == "install":
        run_single_install(args.host, args.ifcguid)
        return 0

    if args.command == "panel-chain":
        count = run_panel_chain(args.host, args.limit)
        LOGGER.info("Requested %d panel(s) via panel-chain", count)
        return 0

    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
