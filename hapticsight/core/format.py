"""Small display helpers shared by the CLI and the simulator UI."""

from __future__ import annotations


def format_bearing(deg: float) -> str:
    """Whole-degree bearing that never prints as 360° (359.6 -> '0°')."""
    return f"{round(deg) % 360}°"


def format_distance(cm: float | None) -> str:
    return "range ?" if cm is None else f"{cm:.0f} cm"
