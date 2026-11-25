"""Hand-controlled *simulated* ultrasonic readings.

Used by the live vision demo and by replays when the physical HC-SR04 sensors
are not available. Everything produced here is simulated and must be labelled
as such in any UI, log, or recording.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from hapticsight.core.layout import ULTRASONIC_SENSORS
from hapticsight.core.models import RangeStatus, UltrasonicReading

SOURCE_LABEL = "SIMULATED"


class ManualMode(StrEnum):
    ECHO = "echo"
    NO_ECHO = "no_echo"
    DROPOUT = "dropout"  # sensor silent -> stale -> DEGRADED cue


@dataclass
class ManualSensor:
    sensor_id: str
    mode: ManualMode = ManualMode.NO_ECHO
    distance_cm: float = 150.0
    speed_cm_s: float = 0.0  # + = approaching (distance shrinks)
    min_distance_cm: float = 20.0
    max_distance_cm: float = 400.0

    def describe(self) -> str:
        if self.mode is ManualMode.ECHO:
            motion = f" {self.speed_cm_s:+.0f}cm/s" if self.speed_cm_s else ""
            return f"{self.distance_cm:.0f} cm{motion}"
        return self.mode.value.replace("_", " ")


@dataclass
class ManualUltrasonic:
    rate_hz: float = 10.0
    sensors: dict[str, ManualSensor] = field(
        default_factory=lambda: {s.sensor_id: ManualSensor(s.sensor_id) for s in ULTRASONIC_SENSORS}
    )
    _next_due: float | None = None
    _last_t: float | None = None

    def readings(self, now: float) -> list[UltrasonicReading]:
        """Readings due at ``now`` (at most one batch per period)."""
        if self._last_t is not None:
            dt = max(0.0, now - self._last_t)
            for s in self.sensors.values():
                if s.mode is ManualMode.ECHO and s.speed_cm_s:
                    s.distance_cm = min(
                        s.max_distance_cm,
                        max(s.min_distance_cm, s.distance_cm - s.speed_cm_s * dt),
                    )
                    if s.distance_cm in (s.min_distance_cm, s.max_distance_cm):
                        s.speed_cm_s = 0.0
        self._last_t = now
        if self._next_due is not None and now + 1e-9 < self._next_due:
            return []
        self._next_due = now + 1.0 / self.rate_hz
        out = []
        for s in self.sensors.values():
            if s.mode is ManualMode.DROPOUT:
                continue
            if s.mode is ManualMode.ECHO:
                out.append(
                    UltrasonicReading(s.sensor_id, now, RangeStatus.VALID, round(s.distance_cm))
                )
            else:
                out.append(UltrasonicReading(s.sensor_id, now, RangeStatus.NO_ECHO))
        return out

    def set(self, sensor_id: str, value: str) -> None:
        """``"80"`` -> echo at 80 cm, ``"X"`` -> no echo, ``"D"`` -> dropout."""
        s = self.sensors[sensor_id]
        v = value.strip().upper()
        s.speed_cm_s = 0.0
        if v == "X":
            s.mode = ManualMode.NO_ECHO
        elif v == "D":
            s.mode = ManualMode.DROPOUT
        else:
            s.mode = ManualMode.ECHO
            s.distance_cm = float(v)

    @classmethod
    def from_spec(cls, spec: str | None) -> ManualUltrasonic:
        """Parse ``"S0=80,S1=X,S2=D"`` (unlisted sensors: no echo)."""
        sim = cls()
        if spec:
            for part in spec.split(","):
                key, _, value = part.partition("=")
                sim.set(key.strip().upper(), value)
        return sim
