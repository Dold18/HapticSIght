"""Time sources.

All scheduling, staleness, and timeout logic takes timestamps from a
:class:`Clock` in **seconds of elapsed monotonic time**. Wall-clock time is
never used for these decisions because it can jump (NTP, DST, user changes).

* :class:`MonotonicClock` wraps :func:`time.monotonic` for real hardware runs.
* :class:`ManualClock` is advanced explicitly; the simulator and tests use it so
  that runs are deterministic and independent of rendering frame rate.
"""

from __future__ import annotations

import time
from typing import Protocol


class Clock(Protocol):
    def now(self) -> float:
        """Return the current monotonic time in seconds."""
        ...


class MonotonicClock:
    """Real elapsed time from the OS monotonic clock."""

    def now(self) -> float:
        return time.monotonic()


class ManualClock:
    """A clock that only moves when :meth:`advance` is called."""

    def __init__(self, start: float = 0.0) -> None:
        self._now = float(start)

    def now(self) -> float:
        return self._now

    def advance(self, dt: float) -> float:
        if dt < 0:
            raise ValueError("ManualClock cannot move backwards")
        self._now += dt
        return self._now

    def set(self, t: float) -> None:
        if t < self._now:
            raise ValueError("ManualClock cannot move backwards")
        self._now = float(t)
