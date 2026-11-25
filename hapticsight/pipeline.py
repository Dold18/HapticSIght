"""The shared processing pipeline.

    ultrasonic readings ─┐
                         ├─> observations ─> RiskEngine ─> HapticEncoder ─> HapticFrame
    camera detections ───┘     (fusion)

The simulator and the future hardware runtime both call
:meth:`HapticSightPipeline.process`; neither re-implements any of these steps.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from hapticsight.config import HapticSightConfig
from hapticsight.core.models import Observation, SensorHealth, UltrasonicReading
from hapticsight.haptics.encoder import HapticEncoder, HapticFrame
from hapticsight.perception.buffer import ObservationBuffer
from hapticsight.perception.fusion import fuse
from hapticsight.perception.ultrasonic import UltrasonicFrontEnd
from hapticsight.risk.engine import RiskEngine, RiskReport


@dataclass(frozen=True, slots=True)
class PipelineResult:
    timestamp: float
    observations: tuple[Observation, ...]
    sensor_health: tuple[SensorHealth, ...]
    report: RiskReport
    frame: HapticFrame


class HapticSightPipeline:
    def __init__(self, config: HapticSightConfig | None = None) -> None:
        self.config = config or HapticSightConfig()
        self.ultrasonic = UltrasonicFrontEnd(self.config.ultrasonic, self.config.range_rate)
        self.camera_buffer = ObservationBuffer(self.config.camera_expire_after_s)
        self.risk = RiskEngine(self.config.risk)
        self.encoder = HapticEncoder(self.config.haptics)

    def reset(self) -> None:
        self.ultrasonic.reset()
        self.camera_buffer.reset()
        self.encoder.reset()

    def process(
        self,
        now: float,
        ultrasonic_readings: Iterable[UltrasonicReading] = (),
        camera_observations: Iterable[Observation] = (),
    ) -> PipelineResult:
        """Normal sensor path: raw readings and camera detections in, frame out."""
        self.ultrasonic.ingest(ultrasonic_readings)
        self.camera_buffer.update(camera_observations)
        observations = fuse(
            self.ultrasonic.observations(),
            self.camera_buffer.current(now),
            self.config.fusion,
            now,
        )
        return self.process_observations(now, observations, self.ultrasonic.health(now))

    def process_observations(
        self,
        now: float,
        observations: Sequence[Observation],
        sensor_health: Sequence[SensorHealth] = (),
    ) -> PipelineResult:
        """Bypass perception (e.g. simulator ground-truth mode, replayed logs)."""
        report = self.risk.assess(observations, now, sensor_health)
        frame = self.encoder.encode(report, now)
        return PipelineResult(now, tuple(observations), tuple(sensor_health), report, frame)
