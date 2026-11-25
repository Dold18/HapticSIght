"""Shared data models used by every layer (simulation, perception, risk, haptics).

The central type is :class:`Observation`: a single, source-agnostic statement
that "something is at this bearing", optionally with a distance and closing
speed. Simulated sensors and future real sensors both produce Observations, so
the risk and haptic layers never know whether data came from a simulator.

Design rules encoded here:

* A distance is only present when ``range_status is RangeStatus.VALID``.
  Camera detections use ``RangeStatus.UNKNOWN`` -- a monocular webcam cannot
  measure metric range, so we never invent one.
* "No echo" (nothing within range of that sensor's cone) is distinct from
  "invalid" (sensor fault / implausible value) and from "missing" (no reading at
  all). Missing data is represented by *absence plus staleness*, never as clear.
* Closing speed is ``None`` unless a source can actually measure or estimate it.
  Positive closing speed means the obstacle is getting closer.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from enum import StrEnum
from types import MappingProxyType
from typing import Any

from hapticsight.core.geometry import normalize_bearing


class SourceType(StrEnum):
    SIMULATION = "simulation"  # Ground-truth oracle from the simulator.
    ULTRASONIC = "ultrasonic"
    CAMERA = "camera"
    FUSED = "fused"


class RangeStatus(StrEnum):
    VALID = "valid"  # distance_cm is a usable measurement.
    NO_ECHO = "no_echo"  # Sensor fired, nothing returned within max range.
    INVALID = "invalid"  # Sensor fault, timeout, or implausible value.
    UNKNOWN = "unknown"  # Source cannot measure range (e.g. monocular camera).


class SensorStatus(StrEnum):
    OK = "ok"  # Recent valid reading with an echo.
    NO_ECHO = "no_echo"  # Recent reading, nothing in range (clear *in that cone only*).
    FAULT = "fault"  # Recent reading reported an error.
    STALE = "stale"  # No reading recently -- coverage unknown.
    NEVER_SEEN = "never_seen"  # No reading has ever been received.

    @property
    def is_degraded(self) -> bool:
        """True when this sensor's sector must not be treated as observed."""
        return self in (SensorStatus.FAULT, SensorStatus.STALE, SensorStatus.NEVER_SEEN)


@dataclass(frozen=True, slots=True)
class Observation:
    """One obstacle observation in the body-relative frame.

    Attributes:
        id: Observation or tracked-object identifier (stable across frames when
            the source tracks objects).
        source: Which kind of source produced it.
        bearing_deg: Bearing relative to the wearer, normalised to [0, 360).
        timestamp: Monotonic time (s) the observation was *measured*.
        confidence: Detection confidence in [0, 1].
        range_status: Whether ``distance_cm`` is meaningful.
        distance_cm: Distance to the obstacle's nearest surface; only when VALID.
        distance_uncertainty_cm: ± uncertainty of ``distance_cm`` if known.
        bearing_uncertainty_deg: ± half-width of the bearing estimate. For an
            ultrasonic sensor this is the beam half-angle: it cannot tell where
            inside its cone the echo came from.
        closing_speed_cm_s: Rate at which distance is decreasing (positive =
            approaching), or None when unavailable.
        sensor_id: Physical sensor identifier, e.g. "S0" or "CAM_FRONT".
        metadata: Free-form, source-specific details (read-only).
    """

    id: str
    source: SourceType
    bearing_deg: float
    timestamp: float
    confidence: float = 1.0
    range_status: RangeStatus = RangeStatus.UNKNOWN
    distance_cm: float | None = None
    distance_uncertainty_cm: float | None = None
    bearing_uncertainty_deg: float = 0.0
    closing_speed_cm_s: float | None = None
    sensor_id: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "bearing_deg", normalize_bearing(self.bearing_deg))
        object.__setattr__(self, "metadata", MappingProxyType(dict(self.metadata)))
        if not 0.0 <= self.confidence <= 1.0 or math.isnan(self.confidence):
            raise ValueError(f"confidence must be within [0, 1], got {self.confidence}")
        if self.bearing_uncertainty_deg < 0:
            raise ValueError("bearing_uncertainty_deg must be non-negative")
        if self.range_status is RangeStatus.VALID:
            if self.distance_cm is None or not math.isfinite(self.distance_cm):
                raise ValueError("a VALID range requires a finite distance_cm")
            if self.distance_cm < 0:
                raise ValueError("distance_cm must be non-negative")
        elif self.distance_cm is not None:
            raise ValueError(
                f"distance_cm must be None when range_status is {self.range_status.value}"
            )
        if self.distance_uncertainty_cm is not None and self.distance_uncertainty_cm < 0:
            raise ValueError("distance_uncertainty_cm must be non-negative")
        if self.closing_speed_cm_s is not None and not math.isfinite(self.closing_speed_cm_s):
            raise ValueError("closing_speed_cm_s must be finite when present")

    @property
    def has_range(self) -> bool:
        return self.range_status is RangeStatus.VALID

    def age(self, now: float) -> float:
        """Seconds since the observation was measured (never negative)."""
        return max(0.0, now - self.timestamp)


@dataclass(frozen=True, slots=True)
class UltrasonicReading:
    """A single raw ranging result from one ultrasonic sensor.

    ``distance_cm`` is set only when ``status is RangeStatus.VALID``. ``timestamp``
    is host monotonic time at which the reading was taken or received.
    """

    sensor_id: str
    timestamp: float
    status: RangeStatus
    distance_cm: float | None = None

    def __post_init__(self) -> None:
        if self.status is RangeStatus.UNKNOWN:
            raise ValueError("ultrasonic readings are VALID, NO_ECHO, or INVALID")
        if (self.status is RangeStatus.VALID) != (self.distance_cm is not None):
            raise ValueError("distance_cm must be provided iff status is VALID")


@dataclass(frozen=True, slots=True)
class SensorHealth:
    """Freshness and status summary for one ultrasonic sensor."""

    sensor_id: str
    bearing_deg: float
    status: SensorStatus
    last_update: float | None
    age_s: float | None
    last_distance_cm: float | None = None
