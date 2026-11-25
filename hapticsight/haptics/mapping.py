"""Bearing -> motor mapping.

Motor ``k`` sits at ``45° * k`` and owns the half-open sector
``[45k - 22.5°, 45k + 22.5°)``. So 22.5° maps to M1 (not M0) and 337.5° maps to
M0 (not M7): each boundary belongs to the clockwise neighbour. All arithmetic is
done on normalised bearings, so 359° and -1° both map to M0.
"""

from __future__ import annotations

import math

from hapticsight.core.geometry import angular_distance, normalize_bearing
from hapticsight.core.layout import (
    MOTOR_COUNT,
    MOTOR_SECTOR_HALF_WIDTH_DEG,
    MOTOR_SPACING_DEG,
    MOTORS,
)


def motor_for_bearing(bearing_deg: float) -> int:
    b = normalize_bearing(bearing_deg)
    return math.floor((b + MOTOR_SECTOR_HALF_WIDTH_DEG) / MOTOR_SPACING_DEG) % MOTOR_COUNT


def motors_for_bearing(bearing_deg: float, uncertainty_deg: float = 0.0) -> tuple[int, ...]:
    """Motors whose sector overlaps ``bearing ± uncertainty``.

    The primary motor is always included first. Neighbours are added only when
    the uncertainty interval genuinely reaches into their sector, so a precise
    bearing activates one motor and an imprecise one spreads honestly.
    """
    if uncertainty_deg < 0:
        raise ValueError("uncertainty_deg must be non-negative")
    primary = motor_for_bearing(bearing_deg)
    if uncertainty_deg == 0:
        return (primary,)
    if uncertainty_deg >= 180:
        return (primary, *(m.index for m in MOTORS if m.index != primary))
    extra = [
        m.index
        for m in MOTORS
        if m.index != primary
        and angular_distance(m.bearing_deg, bearing_deg)
        < MOTOR_SECTOR_HALF_WIDTH_DEG + uncertainty_deg
    ]
    return (primary, *extra)
