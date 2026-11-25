"""Turns haptic frames into instantaneous on/off motor states.

This is a host-side model of the planned firmware algorithm, so the simulator
shows exactly what the vest would do:

* Each channel keeps the start time of its current pattern. Re-sending the
  *same* pattern does not restart it (no stutter at the refresh rate); a
  *different* pattern restarts its phase immediately.
* Each frame carries a time-to-live. If no new frame arrives before it expires
  the channel is switched off here. (The firmware will instead switch to its
  ultrasonic-only fallback mode; see docs/PROTOCOL.md.)
* Motor state is computed from elapsed monotonic time, not from frame or render
  counts, so pulse timing is independent of the graphics frame rate.
"""

from __future__ import annotations

from hapticsight.core.layout import MOTOR_COUNT
from hapticsight.haptics.encoder import HapticFrame
from hapticsight.haptics.patterns import PatternId, PulsePattern


class PulseScheduler:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._pattern_ids: list[PatternId] = [PatternId.OFF] * MOTOR_COUNT
        self._patterns: list[PulsePattern | None] = [None] * MOTOR_COUNT
        self._started_at: list[float] = [0.0] * MOTOR_COUNT
        self._expires_at: float | None = None

    def apply(self, frame: HapticFrame, now: float) -> None:
        for ch in frame.channels:
            m = ch.motor
            if ch.pattern_id != self._pattern_ids[m] or ch.pattern != self._patterns[m]:
                self._pattern_ids[m] = ch.pattern_id
                self._patterns[m] = ch.pattern
                self._started_at[m] = now
        self._expires_at = now + frame.ttl_ms / 1000.0

    def expired(self, now: float) -> bool:
        return self._expires_at is None or now >= self._expires_at

    def active_pattern(self, motor: int, now: float) -> PatternId:
        return PatternId.OFF if self.expired(now) else self._pattern_ids[motor]

    def pattern_state(self, motor: int) -> tuple[PulsePattern | None, float]:
        """Current pattern for ``motor`` and the time it started (for display)."""
        return self._patterns[motor], self._started_at[motor]

    def outputs(self, now: float) -> tuple[bool, ...]:
        if self.expired(now):
            return (False,) * MOTOR_COUNT
        states: list[bool] = []
        for m in range(MOTOR_COUNT):
            pattern = self._patterns[m]
            if pattern is None:
                states.append(False)
                continue
            elapsed_ms = int((now - self._started_at[m]) * 1000.0)
            states.append(pattern.is_on(elapsed_ms))
        return tuple(states)

    def output_byte(self, now: float) -> int:
        """States packed as the 74HC595 byte: bit k = motor Mk."""
        return sum(1 << m for m, on in enumerate(self.outputs(now)) if on)
