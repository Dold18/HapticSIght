"""Latest-observation buffer for sources that report per frame (e.g. cameras)."""

from __future__ import annotations

from collections.abc import Iterable

from hapticsight.core.models import Observation


class ObservationBuffer:
    """Holds the newest observation per id and forgets ids after ``expire_after_s``.

    Expired entries are removed, not reported as "clear": absence of a camera
    detection is never treated as evidence that the area is free.
    """

    def __init__(self, expire_after_s: float = 1.0) -> None:
        if expire_after_s <= 0:
            raise ValueError("expire_after_s must be positive")
        self.expire_after_s = expire_after_s
        self._latest: dict[str, Observation] = {}

    def reset(self) -> None:
        self._latest.clear()

    def update(self, observations: Iterable[Observation]) -> None:
        for obs in observations:
            prev = self._latest.get(obs.id)
            if prev is None or obs.timestamp >= prev.timestamp:
                self._latest[obs.id] = obs

    def current(self, now: float) -> list[Observation]:
        expired = [k for k, o in self._latest.items() if o.age(now) > self.expire_after_s]
        for k in expired:
            del self._latest[k]
        return sorted(self._latest.values(), key=lambda o: o.id)
