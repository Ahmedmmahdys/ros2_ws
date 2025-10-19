"""Utility to run a generated install panel node."""
from __future__ import annotations

import argparse
import runpy
from pathlib import Path
from typing import Optional

from .. import PACKAGE_ROOT


def resolve_node_path(node_path: Optional[str], ifcguid: Optional[str]) -> Path:
    if node_path:
        return Path(node_path)
    if ifcguid:
        return PACKAGE_ROOT / "generated" / f"install_panel_{ifcguid}.py"
    raise ValueError("Either --node or --ifcguid must be provided")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a generated install panel node")
    parser.add_argument("--node", help="Explicit path to generated node", default=None)
    parser.add_argument("--ifcguid", help="Panel GUID for node lookup", default=None)
    args = parser.parse_args()
    path = resolve_node_path(args.node, args.ifcguid)
    runpy.run_path(str(path), run_name="__main__")


if __name__ == "__main__":
    main()
