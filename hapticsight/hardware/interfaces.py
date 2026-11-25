"""Adapter interfaces for physical devices.

The pipeline only depends on these protocols, so a simulated source and a real
device are interchangeable. No implementation in this package talks to real
hardware yet (Milestone 1 runs entirely without devices).
"""

from __future__ import annotations

from typing import Protocol

from hapticsight.core.models import Observation, UltrasonicReading
from hapticsight.haptics.encoder import HapticFrame


class UltrasonicSource(Protocol):
    def poll(self) -> list[UltrasonicReading]:
        """Return readings received since the last call (non-blocking)."""
        ...


class CameraDetector(Protocol):
    """Future: webcam + object detector producing bearing-only observations.

    Implementations must set ``range_status=RangeStatus.UNKNOWN`` and
    ``closing_speed_cm_s=None``: a monocular webcam does not measure metric
    range. Bearing comes from the detection's horizontal pixel position and the
    camera's calibrated field of view.
    """

    def poll(self) -> list[Observation]: ...


class HapticSink(Protocol):
    def send(self, frame: HapticFrame) -> None:
        """Deliver a haptic frame to the actuators (non-blocking)."""
        ...


class ByteTransport(Protocol):
    """Minimal serial-port abstraction (pyserial will satisfy this later)."""

    def read_available(self) -> bytes: ...

    def write(self, data: bytes) -> None: ...

    def close(self) -> None: ...
