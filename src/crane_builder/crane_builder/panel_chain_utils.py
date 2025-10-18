"""Shared helpers for panel chain processing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Tuple

from geometry_msgs.msg import Point


@dataclass
class PanelLink:
    """Represents a single panel move instruction."""

    ifc_guid: str
    hook_point: Point
    panel_position: Point
    target_position: Point
    next_ifc_guid: Optional[str]


def point_from_value(value: object, *, field_name: str) -> Point:
    """Convert a Neo4j-provided value into a ROS ``Point`` message.

    Parameters
    ----------
    value:
        A value representing a 3D coordinate. Supported formats are:

        - A length-3 iterable (list/tuple) of numeric values.
        - A mapping with the keys ``x``, ``y`` and ``z`` (case-insensitive).

    field_name:
        The name of the field used in error reporting.
    """

    if value is None:
        raise ValueError(f"Field '{field_name}' must be provided.")

    xyz: Tuple[float, float, float]

    if isinstance(value, Mapping):
        lower_keys = {str(k).lower(): k for k in value.keys()}
        missing = {axis for axis in ("x", "y", "z") if axis not in lower_keys}
        if missing:
            raise ValueError(
                f"Field '{field_name}' must define x, y and z keys; missing: {sorted(missing)}"
            )
        xyz = (
            float(value[lower_keys["x"]]),
            float(value[lower_keys["y"]]),
            float(value[lower_keys["z"]]),
        )
    elif isinstance(value, Iterable) and not isinstance(value, (str, bytes)):
        items = list(value)
        if len(items) != 3:
            raise ValueError(
                f"Field '{field_name}' must contain exactly 3 values; received {len(items)}"
            )
        xyz = tuple(float(component) for component in items)  # type: ignore[assignment]
    else:
        raise TypeError(
            f"Field '{field_name}' must be a mapping or length-3 iterable, not {type(value)!r}"
        )

    point = Point()
    point.x, point.y, point.z = xyz
    return point
