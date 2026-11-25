"""Unified live runtime: cameras + Arduino ultrasonic -> shared pipeline -> Arduino motors.

    VisionSystem.camera_observations ─┐
                                      ├─> HapticSightPipeline.process ─> HapticFrame
    UltrasonicInput.readings ─────────┘                                      │
                                         UltrasonicInput.deliver (ArduinoBridge.send) <─┘

There is no separate risk engine here: this loop only moves data between the
existing components and logs what happened. The headless loop runs at a
configurable rate (default 15 Hz); the live UI drives the same ``step``.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from hapticsight.core.clock import Clock, MonotonicClock
from hapticsight.core.format import format_bearing
from hapticsight.core.models import UltrasonicReading
from hapticsight.hardware.inputs import UltrasonicInput
from hapticsight.hardware.telemetry import TelemetryLogger
from hapticsight.pipeline import HapticSightPipeline, PipelineResult


@dataclass(frozen=True, slots=True)
class RuntimeStep:
    timestamp: float
    readings: tuple[UltrasonicReading, ...]
    result: PipelineResult
    sent: bool


class HardwareRuntime:
    def __init__(
        self,
        ultrasonic: UltrasonicInput,
        pipeline: HapticSightPipeline | None = None,
        vision: Any | None = None,  # VisionSystem (optional; imported lazily)
        clock: Clock | None = None,
        logger: TelemetryLogger | None = None,
        loop_hz: float = 15.0,
    ) -> None:
        if not 1.0 <= loop_hz <= 50.0:
            raise ValueError("loop_hz must be within [1, 50]")
        self.ultrasonic = ultrasonic
        self.pipeline = pipeline or HapticSightPipeline()
        self.vision = vision
        self.clock = clock or MonotonicClock()
        self.logger = logger
        self.loop_hz = loop_hz
        self.steps = 0
        self.last: RuntimeStep | None = None

    def step(self, now: float | None = None) -> RuntimeStep:
        now = self.clock.now() if now is None else now
        readings = self.ultrasonic.readings(now)
        cams = self.vision.camera_observations(now) if self.vision is not None else []
        result = self.pipeline.process(now, readings, cams)
        sent = False
        before = getattr(getattr(self.ultrasonic, "bridge", None), "hap_sent", None)
        self.ultrasonic.deliver(result.frame, now)
        after = getattr(getattr(self.ultrasonic, "bridge", None), "hap_sent", None)
        sent = before is not None and after != before
        step = RuntimeStep(now, tuple(readings), result, sent)
        self.steps += 1
        self.last = step
        if self.logger is not None:
            self._log(step)
        return step

    def _log(self, step: RuntimeStep) -> None:
        assert self.logger is not None
        kind = self.ultrasonic.kind.value
        if step.readings:
            self.logger.log(
                "readings",
                step.timestamp,
                source=kind,
                simulated=self.ultrasonic.kind.simulated,
                values={r.sensor_id: (r.status.value, r.distance_cm) for r in step.readings},
            )
        r = step.result
        self.logger.log(
            "step",
            step.timestamp,
            ultrasonic_source=kind,
            observations=[
                {
                    "id": o.id,
                    "source": o.source.value,
                    "bearing_deg": round(o.bearing_deg, 1),
                    "range_cm": o.distance_cm,
                }
                for o in r.observations
            ],
            assessments=[
                {
                    "id": a.id,
                    "severity": a.severity.name,
                    "bearing": format_bearing(a.bearing_deg),
                    "range_cm": a.effective_distance_cm,
                    "stale": a.is_stale,
                }
                for a in r.report.top(8)
            ],
            haptics=r.frame.pattern_string(),
            hap_sent=step.sent,
        )

    def run(self, duration_s: float | None = None, stop: threading.Event | None = None) -> int:
        """Headless loop at ``loop_hz``; returns the number of steps."""
        period = 1.0 / self.loop_hz
        start = self.clock.now()
        next_t = start
        stop = stop or threading.Event()
        while not stop.is_set():
            now = self.clock.now()
            if duration_s is not None and now - start >= duration_s:
                break
            if now >= next_t:
                self.step(now)
                next_t += period
                if next_t < now:  # fell behind: do not try to catch up in a burst
                    next_t = now + period
            stop.wait(max(0.0, min(period, next_t - self.clock.now())))
        return self.steps

    def close(self) -> None:
        self.ultrasonic.close()
        if self.vision is not None:
            self.vision.stop()
        if self.logger is not None:
            self.logger.close()
