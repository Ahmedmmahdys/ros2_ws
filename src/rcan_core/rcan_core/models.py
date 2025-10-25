"""Domain models for RCAN panels and activities."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass(slots=True)
class Panel:
    """A facade panel tracked in the RCAN graph."""

    ifcguid: str
    hook: Vec3
    target: Vec3
    date: Optional[str] = None


@dataclass(slots=True)
class Activity:
    """Activity describing an installation task for a panel."""

    id: str
    name: str
    ifcguid: str
    params: Dict[str, object]


__all__ = ["Activity", "Panel", "Vec3"]
