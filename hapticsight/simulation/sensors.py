"""Simulated ultrasonic sensors and cameras.

These produce exactly the data types real adapters will produce
(:class:`UltrasonicReading` and camera :class:`Observation`) so the shared
pipeline cannot tell the difference.

Ultrasonic model (simplified HC-SR04):
  * Reports the nearest obstacle whose angular extent overlaps the beam cone
    (``beam_half_angle``) and is within max range. Objects outside the cone are
    *not* seen: there are blind zones between the four sensors.
  * Gaussian range noise, quantised to whole centimetres like the firmware.
  * Each ping returns an echo with the obstacle's ``echo_probability``.
  * Modes: ENABLED, DROPOUT (stops reporting; data goes stale), FAULT (reports
    errors).

Camera model (front/rear webcam + idealised detector):
  * Detects obstacle centres inside the horizontal FOV and max range.
  * Reports bearing (with noise) and confidence only: **no range, no speed**.
  * Random misses; no occlusion modelling.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from enum import StrEnum

from hapticsight.core.geometry import angular_distance
from hapticsight.core.layout import CAMERAS, ULTRASONIC_SENSORS, CameraSpec, UltrasonicSensorSpec
from hapticsight.core.models import Observation, RangeStatus, SourceType, UltrasonicReading
from hapticsight.perception.ultrasonic import UltrasonicConfig
from hapticsight.simulation.world import World


class SensorMode(StrEnum):
    ENABLED = "enabled"
    DROPOUT = "dropout"
    FAULT = "fault"


@dataclass(frozen=True, slots=True)
class SimSensorConfig:
    ultrasonic_rate_hz: float = 5.0  # matches the firmware's staggered 4 x 50 ms cycle
    ultrasonic_noise_cm: float = 1.0
    camera_rate_hz: float = 15.0
    camera_hfov_deg: float = 60.0
    camera_max_range_cm: float = 800.0
    camera_bearing_noise_deg: float = 1.0
    camera_bearing_uncertainty_deg: float = 3.0
    camera_miss_probability: float = 0.05


class SimulatedUltrasonicArray:
    def __init__(
        self,
        us_config: UltrasonicConfig,
        sim_config: SimSensorConfig,
        rng: random.Random,
        sensors: tuple[UltrasonicSensorSpec, ...] = ULTRASONIC_SENSORS,
    ) -> None:
        self.us_config = us_config
        self.sim_config = sim_config
        self.rng = rng
        self.sensors = sensors
        self.modes: dict[str, SensorMode] = {s.sensor_id: SensorMode.ENABLED for s in sensors}
        self.last_emitted: dict[str, UltrasonicReading] = {}
        self._next_sample = 0.0

    @property
    def period_s(self) -> float:
        return 1.0 / self.sim_config.ultrasonic_rate_hz

    def reset(self) -> None:
        self.modes = {s.sensor_id: SensorMode.ENABLED for s in self.sensors}
        self.last_emitted.clear()
        self._next_sample = 0.0

    def due(self, now: float) -> bool:
        return now + 1e-9 >= self._next_sample

    def sample(self, world: World, now: float, force: bool = False) -> list[UltrasonicReading]:
        if not force:
            if not self.due(now):
                return []
            self._next_sample += self.period_s
            if self._next_sample <= now:  # fell behind (e.g. long pause) - resync
                self._next_sample = now + self.period_s
        out: list[UltrasonicReading] = []
        for spec in self.sensors:
            mode = self.modes[spec.sensor_id]
            if mode is SensorMode.DROPOUT:
                continue
            if mode is SensorMode.FAULT:
                reading = UltrasonicReading(spec.sensor_id, now, RangeStatus.INVALID)
            else:
                reading = self.measure(spec, world, now)
            self.last_emitted[spec.sensor_id] = reading
            out.append(reading)
        return out

    def measure(self, spec: UltrasonicSensorSpec, world: World, now: float) -> UltrasonicReading:
        cfg = self.us_config
        nearest: float | None = None
        for o in world.obstacles:
            gap = angular_distance(o.bearing_deg, spec.bearing_deg)
            if gap > cfg.beam_half_angle_deg + o.angular_half_extent_deg:
                continue
            d = o.clearance_cm
            if d > cfg.max_range_cm:
                continue
            if o.echo_probability < 1.0 and self.rng.random() >= o.echo_probability:
                continue
            if nearest is None or d < nearest:
                nearest = d
        if nearest is None:
            return UltrasonicReading(spec.sensor_id, now, RangeStatus.NO_ECHO)
        noisy = round(nearest + self.rng.gauss(0.0, self.sim_config.ultrasonic_noise_cm))
        if noisy < cfg.min_range_cm:
            return UltrasonicReading(spec.sensor_id, now, RangeStatus.INVALID)
        if noisy > cfg.max_range_cm:
            return UltrasonicReading(spec.sensor_id, now, RangeStatus.NO_ECHO)
        return UltrasonicReading(spec.sensor_id, now, RangeStatus.VALID, float(noisy))


class SimulatedCamera:
    def __init__(self, spec: CameraSpec, sim_config: SimSensorConfig, rng: random.Random) -> None:
        self.spec = spec
        self.config = sim_config
        self.rng = rng
        self.enabled = True
        self.last_detections: list[Observation] = []
        self._next_sample = 0.0

    def reset(self) -> None:
        self.enabled = True
        self.last_detections = []
        self._next_sample = 0.0

    def sample(self, world: World, now: float, force: bool = False) -> list[Observation]:
        if not force:
            if now + 1e-9 < self._next_sample:
                return []
            self._next_sample += 1.0 / self.config.camera_rate_hz
            if self._next_sample <= now:
                self._next_sample = now + 1.0 / self.config.camera_rate_hz
        if not self.enabled:
            self.last_detections = []
            return []
        cfg = self.config
        out: list[Observation] = []
        for o in world.obstacles:
            if not o.camera_visible:
                continue
            if angular_distance(o.bearing_deg, self.spec.bearing_deg) > cfg.camera_hfov_deg / 2:
                continue
            if o.clearance_cm > cfg.camera_max_range_cm:
                continue
            if self.rng.random() < cfg.camera_miss_probability:
                continue
            if o.camera_confidence is not None:
                confidence = o.camera_confidence
            else:
                confidence = max(0.5, 0.95 - 0.45 * o.clearance_cm / cfg.camera_max_range_cm)
            out.append(
                Observation(
                    id=f"{self.spec.camera_id}:{o.id}",
                    source=SourceType.CAMERA,
                    bearing_deg=o.bearing_deg + self.rng.gauss(0.0, cfg.camera_bearing_noise_deg),
                    timestamp=now,
                    confidence=confidence,
                    range_status=RangeStatus.UNKNOWN,
                    bearing_uncertainty_deg=cfg.camera_bearing_uncertainty_deg,
                    sensor_id=self.spec.camera_id,
                    metadata={"label": o.label or o.id},
                )
            )
        self.last_detections = out
        return out


def make_cameras(sim_config: SimSensorConfig, rng: random.Random) -> list[SimulatedCamera]:
    return [SimulatedCamera(spec, sim_config, rng) for spec in CAMERAS]
