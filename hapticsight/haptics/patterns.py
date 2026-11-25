"""Pulse patterns for on/off ERM coin motors.

The hardware switches each motor fully on or off (74HC595 -> ULN2803A), so
urgency is encoded in *timing*, not amplitude. A pattern is a burst of
``count`` pulses (each ``on_ms`` long, separated by ``off_ms``) followed by
``pause_ms`` of silence, repeated::

    |on|off|on|off| ... |on|        pause        |on|off| ...
    <------ burst of `count` pulses ------>

All fields are small unsigned integers so the same table fits in Arduino Uno
SRAM (uint16 times, uint8 count) and can be sent with a ``$PAT`` message.

The approaching cue is a **double pulse** at the same base repetition rate as
the plain severity pattern. The degraded-coverage cue is a sparse triple blip.
These defaults are provisional and need user testing on the real vest.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntEnum
from types import MappingProxyType

MAX_TIME_MS = 65535
MAX_COUNT = 9


class PatternId(IntEnum):
    """Pattern slot numbers, shared with the serial protocol and firmware."""

    OFF = 0
    CAUTION = 1
    WARNING = 2
    CRITICAL = 3
    CAUTION_APPROACH = 4
    WARNING_APPROACH = 5
    CRITICAL_APPROACH = 6
    DEGRADED = 7


@dataclass(frozen=True, slots=True)
class PulsePattern:
    on_ms: int
    off_ms: int = 0
    count: int = 1
    pause_ms: int = 0

    def __post_init__(self) -> None:
        for name in ("on_ms", "off_ms", "pause_ms"):
            v = getattr(self, name)
            if not isinstance(v, int) or not 0 <= v <= MAX_TIME_MS:
                raise ValueError(f"{name} must be an int in [0, {MAX_TIME_MS}], got {v!r}")
        if not isinstance(self.count, int) or not 0 <= self.count <= MAX_COUNT:
            raise ValueError(f"count must be an int in [0, {MAX_COUNT}]")
        if self.count > 0 and self.on_ms == 0:
            raise ValueError("a pattern with pulses needs on_ms > 0")

    @property
    def is_silent(self) -> bool:
        return self.count == 0

    @property
    def cycle_ms(self) -> int:
        if self.count == 0:
            return 0
        return self.count * self.on_ms + (self.count - 1) * self.off_ms + self.pause_ms

    @property
    def duty_cycle(self) -> float:
        cycle = self.cycle_ms
        return 0.0 if cycle == 0 else (self.count * self.on_ms) / cycle

    def is_on(self, elapsed_ms: int) -> bool:
        """Motor state ``elapsed_ms`` after the pattern started (integer maths)."""
        if self.count == 0 or elapsed_ms < 0:
            return False
        phase = elapsed_ms % self.cycle_ms
        period = self.on_ms + self.off_ms
        burst_len = self.count * self.on_ms + (self.count - 1) * self.off_ms
        if phase >= burst_len:
            return False
        return (phase % period) < self.on_ms

    def describe(self) -> str:
        if self.count == 0:
            return "off"
        if self.pause_ms == 0 and (self.count == 1 or self.off_ms == 0):
            return "continuous"
        pulses = f"{self.count}x{self.on_ms}ms" + (f"/{self.off_ms}" if self.count > 1 else "")
        return f"{pulses} every {self.cycle_ms}ms ({self.duty_cycle:.0%})"


DEFAULT_PATTERNS: Mapping[PatternId, PulsePattern] = MappingProxyType(
    {
        PatternId.OFF: PulsePattern(on_ms=0, count=0),
        # 1 Hz single tap, 10 % duty
        PatternId.CAUTION: PulsePattern(on_ms=100, pause_ms=900),
        # 2 Hz, 30 % duty
        PatternId.WARNING: PulsePattern(on_ms=150, pause_ms=350),
        # 4 Hz, 72 % duty: near-continuous buzz
        PatternId.CRITICAL: PulsePattern(on_ms=180, pause_ms=70),
        # Approaching variants: double tap at the same repetition rate
        PatternId.CAUTION_APPROACH: PulsePattern(on_ms=80, off_ms=80, count=2, pause_ms=760),
        PatternId.WARNING_APPROACH: PulsePattern(on_ms=100, off_ms=60, count=2, pause_ms=240),
        PatternId.CRITICAL_APPROACH: PulsePattern(on_ms=100, off_ms=30, count=2, pause_ms=20),
        # Sensor coverage lost: sparse triple blip every 2 s
        PatternId.DEGRADED: PulsePattern(on_ms=50, off_ms=80, count=3, pause_ms=1690),
    }
)
