"""Body-relative geometry helpers.

Coordinate system (viewed from above, wearer at the origin):

* Bearings are in degrees, 0° = straight ahead, increasing **clockwise**
  (90° = right, 180° = behind, 270° = left). Canonical range is [0, 360).
* Cartesian coordinates are in centimetres: +x points to the wearer's right,
  +y points forward. Therefore ``bearing = atan2(x, y)``.
"""

from __future__ import annotations

import math

FULL_CIRCLE_DEG = 360.0


def normalize_bearing(deg: float) -> float:
    """Wrap any angle into the canonical range [0, 360).

    Handles negative values, values above 360, and floating point results that
    would otherwise round up to exactly 360.0.
    """
    if not math.isfinite(deg):
        raise ValueError(f"bearing must be finite, got {deg!r}")
    wrapped = deg % FULL_CIRCLE_DEG
    # e.g. -1e-15 % 360 == 360.0 in floating point.
    if wrapped >= FULL_CIRCLE_DEG:
        wrapped = 0.0
    return wrapped


def signed_angle_difference(target_deg: float, reference_deg: float) -> float:
    """Shortest signed rotation from ``reference`` to ``target`` in (-180, 180].

    Positive means ``target`` is clockwise of ``reference``.
    """
    diff = normalize_bearing(target_deg - reference_deg)
    if diff > 180.0:
        diff -= FULL_CIRCLE_DEG
    return diff


def angular_distance(a_deg: float, b_deg: float) -> float:
    """Unsigned smallest angle between two bearings, in [0, 180]."""
    return abs(signed_angle_difference(a_deg, b_deg))


def bearing_within(bearing_deg: float, center_deg: float, half_width_deg: float) -> bool:
    """True if ``bearing`` lies within ``center ± half_width`` (inclusive), wrap-safe."""
    return angular_distance(bearing_deg, center_deg) <= half_width_deg


def polar_to_cartesian(bearing_deg: float, distance: float) -> tuple[float, float]:
    """Convert (bearing, distance) to body-frame (x right, y forward)."""
    rad = math.radians(bearing_deg)
    return distance * math.sin(rad), distance * math.cos(rad)


def cartesian_to_polar(x: float, y: float) -> tuple[float, float]:
    """Convert body-frame (x right, y forward) to (bearing in [0, 360), distance)."""
    distance = math.hypot(x, y)
    if distance == 0.0:
        return 0.0, 0.0
    return normalize_bearing(math.degrees(math.atan2(x, y))), distance
