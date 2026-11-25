"""Ultrasonic front end: raw readings -> Observations + per-sensor health.

Four HC-SR04 sensors give four *sectors* of range information. Each reading is
converted into at most one Observation located at the sensor's boresight
bearing, with ``bearing_uncertainty_deg`` equal to the beam half-angle: the
sensor reports the nearest echo somewhere in its cone, not a precise bearing.

Closing speed is estimated from successive readings of the same sensor with a
least-squares fit over a short window. It is reported only when enough
consistent samples exist; otherwise it stays ``None`` (unknown).
"""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from hapticsight.core.layout import ULTRASONIC_SENSORS, UltrasonicSensorSpec
from hapticsight.core.models import (
    Observation,
    RangeStatus,
    SensorHealth,
    SensorStatus,
    SourceType,
    UltrasonicReading,
)


@dataclass(frozen=True, slots=True)
class UltrasonicConfig:
    """Sensor model parameters (provisional HC-SR04 values)."""

    beam_half_angle_deg: float = 15.0
    min_range_cm: float = 2.0
    max_range_cm: float = 400.0
    distance_uncertainty_cm: float = 2.0
    confidence: float = 0.9
    # Firmware reports each sensor at 5 Hz (one US frame per 200 ms cycle); stale
    # after 3 periods without a reading, i.e. two consecutive frames lost.
    stale_after_s: float = 0.6


@dataclass(frozen=True, slots=True)
class RangeRateConfig:
    window_s: float = 1.0  # ~5 samples at the firmware's 5 Hz per sensor
    min_samples: int = 4
    min_span_s: float = 0.4
    # A jump implying a faster change than this is treated as a different
    # object entering the beam, so the history restarts.
    max_plausible_rate_cm_s: float = 400.0


class RangeRateEstimator:
    """Per-key closing-speed estimate from a sliding window of range samples."""

    def __init__(self, config: RangeRateConfig | None = None) -> None:
        self.config = config or RangeRateConfig()
        self._history: dict[str, deque[tuple[float, float]]] = {}

    def reset(self, key: str | None = None) -> None:
        if key is None:
            self._history.clear()
        else:
            self._history.pop(key, None)

    def update(self, key: str, t: float, distance_cm: float) -> float | None:
        """Add a sample and return the closing speed (cm/s, + = approaching)."""
        cfg = self.config
        hist = self._history.setdefault(key, deque())
        if hist:
            last_t, last_d = hist[-1]
            dt = t - last_t
            if dt <= 0:
                return self.estimate(key)  # duplicate / out-of-order sample
            if abs(distance_cm - last_d) / dt > cfg.max_plausible_rate_cm_s:
                hist.clear()
        hist.append((t, distance_cm))
        while hist and t - hist[0][0] > cfg.window_s:
            hist.popleft()
        return self.estimate(key)

    def estimate(self, key: str) -> float | None:
        hist = self._history.get(key)
        cfg = self.config
        if not hist or len(hist) < cfg.min_samples:
            return None
        if hist[-1][0] - hist[0][0] < cfg.min_span_s:
            return None
        n = len(hist)
        mean_t = sum(s[0] for s in hist) / n
        mean_d = sum(s[1] for s in hist) / n
        sxx = sum((s[0] - mean_t) ** 2 for s in hist)
        if sxx == 0:
            return None
        sxy = sum((s[0] - mean_t) * (s[1] - mean_d) for s in hist)
        slope = sxy / sxx  # cm/s, negative when distance shrinks
        return -slope


class UltrasonicFrontEnd:
    """Keeps the latest reading per sensor and turns it into observations."""

    def __init__(
        self,
        config: UltrasonicConfig | None = None,
        rate_config: RangeRateConfig | None = None,
        sensors: Sequence[UltrasonicSensorSpec] = ULTRASONIC_SENSORS,
    ) -> None:
        self.config = config or UltrasonicConfig()
        self.sensors = tuple(sensors)
        self._specs = {s.sensor_id: s for s in self.sensors}
        self._rate = RangeRateEstimator(rate_config)
        self._latest: dict[str, UltrasonicReading] = {}
        self._closing: dict[str, float | None] = {}

    def reset(self) -> None:
        self._latest.clear()
        self._closing.clear()
        self._rate.reset()

    def sanitize(self, reading: UltrasonicReading) -> UltrasonicReading:
        """Reclassify physically implausible VALID readings."""
        cfg = self.config
        if reading.status is RangeStatus.VALID and reading.distance_cm is not None:
            if reading.distance_cm < cfg.min_range_cm:
                return UltrasonicReading(reading.sensor_id, reading.timestamp, RangeStatus.INVALID)
            if reading.distance_cm > cfg.max_range_cm:
                return UltrasonicReading(reading.sensor_id, reading.timestamp, RangeStatus.NO_ECHO)
        return reading

    def ingest(self, readings: Iterable[UltrasonicReading]) -> None:
        for raw in readings:
            if raw.sensor_id not in self._specs:
                raise KeyError(f"unknown ultrasonic sensor {raw.sensor_id!r}")
            reading = self.sanitize(raw)
            previous = self._latest.get(reading.sensor_id)
            if previous is not None and reading.timestamp < previous.timestamp:
                continue  # never let an older reading overwrite a newer one
            self._latest[reading.sensor_id] = reading
            if reading.status is RangeStatus.VALID and reading.distance_cm is not None:
                self._closing[reading.sensor_id] = self._rate.update(
                    reading.sensor_id, reading.timestamp, reading.distance_cm
                )
            else:
                self._rate.reset(reading.sensor_id)
                self._closing[reading.sensor_id] = None

    def latest(self, sensor_id: str) -> UltrasonicReading | None:
        return self._latest.get(sensor_id)

    def observations(self) -> list[Observation]:
        """One observation per sensor whose latest reading has an echo.

        Staleness is *not* filtered here; the risk engine judges age.
        """
        cfg = self.config
        out: list[Observation] = []
        for spec in self.sensors:
            reading = self._latest.get(spec.sensor_id)
            if reading is None or reading.status is not RangeStatus.VALID:
                continue
            out.append(
                Observation(
                    id=f"us:{spec.sensor_id}",
                    source=SourceType.ULTRASONIC,
                    bearing_deg=spec.bearing_deg,
                    timestamp=reading.timestamp,
                    confidence=cfg.confidence,
                    range_status=RangeStatus.VALID,
                    distance_cm=reading.distance_cm,
                    distance_uncertainty_cm=cfg.distance_uncertainty_cm,
                    bearing_uncertainty_deg=cfg.beam_half_angle_deg,
                    closing_speed_cm_s=self._closing.get(spec.sensor_id),
                    sensor_id=spec.sensor_id,
                )
            )
        return out

    def health(self, now: float) -> list[SensorHealth]:
        out: list[SensorHealth] = []
        for spec in self.sensors:
            reading = self._latest.get(spec.sensor_id)
            if reading is None:
                out.append(
                    SensorHealth(
                        spec.sensor_id, spec.bearing_deg, SensorStatus.NEVER_SEEN, None, None
                    )
                )
                continue
            age = max(0.0, now - reading.timestamp)
            if age > self.config.stale_after_s:
                status = SensorStatus.STALE
            elif reading.status is RangeStatus.VALID:
                status = SensorStatus.OK
            elif reading.status is RangeStatus.NO_ECHO:
                status = SensorStatus.NO_ECHO
            else:
                status = SensorStatus.FAULT
            out.append(
                SensorHealth(
                    spec.sensor_id,
                    spec.bearing_deg,
                    status,
                    reading.timestamp,
                    age,
                    reading.distance_cm,
                )
            )
        return out
