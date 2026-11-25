"""Headless simulation engine (no graphics).

Runs the world, simulated sensors, the shared :class:`HapticSightPipeline`, and
the :class:`PulseScheduler` on a fixed time step using a :class:`ManualClock`.
Given the same scenario and seed, every run produces identical results,
regardless of how often (or whether) anything renders it.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from enum import StrEnum

from hapticsight.config import HapticSightConfig
from hapticsight.core.clock import ManualClock
from hapticsight.core.models import Observation, UltrasonicReading
from hapticsight.haptics.scheduler import PulseScheduler
from hapticsight.pipeline import HapticSightPipeline, PipelineResult
from hapticsight.simulation.scenarios import SANDBOX, Scenario, ScenarioEvent
from hapticsight.simulation.sensors import (
    SensorMode,
    SimSensorConfig,
    SimulatedCamera,
    SimulatedUltrasonicArray,
    make_cameras,
)
from hapticsight.simulation.world import SimObstacle, World, ground_truth_observations


class PerceptionMode(StrEnum):
    SENSORS = "sensors"  # realistic: simulated ultrasonic + camera -> pipeline
    GROUND_TRUTH = "ground_truth"  # oracle: perfect observations -> pipeline


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    tick_hz: float = 50.0
    max_steps_per_advance: int = 10  # avoid a spiral of death after a stall
    sensors: SimSensorConfig = field(default_factory=SimSensorConfig)


class SimulationEngine:
    def __init__(
        self,
        config: HapticSightConfig | None = None,
        sim_config: SimulationConfig | None = None,
        scenario: Scenario = SANDBOX,
        seed: int | None = None,
    ) -> None:
        self.config = config or HapticSightConfig()
        self.sim_config = sim_config or SimulationConfig()
        self.clock = ManualClock()
        self.pipeline = HapticSightPipeline(self.config)
        self.scheduler = PulseScheduler()
        self.world = World()
        self.rng = random.Random()
        self.ultrasonic = SimulatedUltrasonicArray(
            self.config.ultrasonic, self.sim_config.sensors, self.rng
        )
        self.cameras: list[SimulatedCamera] = make_cameras(self.sim_config.sensors, self.rng)
        self.mode = PerceptionMode.SENSORS
        self.paused = False
        self.event_log: list[tuple[float, str]] = []
        self._seed_override = seed
        self._accumulator = 0.0
        self.scenario = scenario
        self.last_result: PipelineResult | None = None
        self.load_scenario(scenario)

    # ------------------------------------------------------------ lifecycle

    @property
    def dt(self) -> float:
        return 1.0 / self.sim_config.tick_hz

    @property
    def time(self) -> float:
        return self.clock.now()

    @property
    def seed(self) -> int:
        return self._seed_override if self._seed_override is not None else self.scenario.seed

    def load_scenario(self, scenario: Scenario) -> None:
        self.scenario = scenario
        self.reset()

    def reset(self) -> None:
        """Restore the current scenario's initial state (deterministic)."""
        self.clock = ManualClock()
        self.rng.seed(self.seed)
        self.world.clear()
        for spec in self.scenario.obstacles:
            self.world.add(spec.build())
        self.ultrasonic.reset()
        for cam in self.cameras:
            cam.reset()
            cam.enabled = cam.spec.camera_id not in self.scenario.disabled_cameras
        self.pipeline.reset()
        self.scheduler.reset()
        self._pending_events = sorted(self.scenario.events, key=lambda e: e.time_s)
        self.event_log = []
        self._accumulator = 0.0
        self._process(self.clock.now())  # sensors are due at t=0

    # ------------------------------------------------------------ stepping

    def step(self) -> PipelineResult:
        """Advance exactly one fixed tick."""
        now = self.clock.advance(self.dt)
        self.world.step(self.dt)
        self._apply_events(now)
        return self._process(now)

    def advance(self, real_dt: float) -> int:
        """Advance by wall-time ``real_dt`` using fixed ticks. Returns ticks run."""
        if self.paused:
            return 0
        self._accumulator += real_dt
        steps = 0
        while self._accumulator >= self.dt and steps < self.sim_config.max_steps_per_advance:
            self.step()
            self._accumulator -= self.dt
            steps += 1
        if steps == self.sim_config.max_steps_per_advance:
            self._accumulator = 0.0
        return steps

    def run_for(self, seconds: float) -> list[PipelineResult]:
        n = round(seconds * self.sim_config.tick_hz)
        return [self.step() for _ in range(n)]

    def refresh(self) -> PipelineResult:
        """Re-sample sensors and re-run the pipeline without advancing time.

        Used after interactive edits while paused so the display reflects them.
        """
        return self._process(self.clock.now(), force_sample=True)

    def _apply_events(self, now: float) -> None:
        while self._pending_events and self._pending_events[0].time_s <= now + 1e-9:
            self.apply_event(self._pending_events.pop(0))

    def apply_event(self, event: ScenarioEvent) -> None:
        if event.kind == "ultrasonic_mode":
            self.ultrasonic.modes[event.target] = SensorMode(str(event.value))
        elif event.kind == "camera_enabled":
            self.camera(event.target).enabled = bool(event.value)
        self.event_log.append((self.clock.now(), event.note or f"{event.target}={event.value}"))

    def _process(self, now: float, force_sample: bool = False) -> PipelineResult:
        readings: list[UltrasonicReading] = self.ultrasonic.sample(self.world, now, force_sample)
        detections: list[Observation] = []
        for cam in self.cameras:
            detections.extend(cam.sample(self.world, now, force_sample))
        if self.mode is PerceptionMode.GROUND_TRUTH:
            result = self.pipeline.process_observations(
                now, ground_truth_observations(self.world, now)
            )
        else:
            result = self.pipeline.process(now, readings, detections)
        self.scheduler.apply(result.frame, now)
        self.last_result = result
        return result

    # ------------------------------------------------------------ queries

    @property
    def result(self) -> PipelineResult:
        assert self.last_result is not None
        return self.last_result

    def motor_outputs(self) -> tuple[bool, ...]:
        return self.scheduler.outputs(self.clock.now())

    def camera(self, camera_id: str) -> SimulatedCamera:
        return next(c for c in self.cameras if c.spec.camera_id == camera_id)

    # ------------------------------------------------------------ interaction

    def set_mode(self, mode: PerceptionMode) -> None:
        self.mode = mode
        self.pipeline.reset()
        self.refresh()

    def toggle_mode(self) -> PerceptionMode:
        self.set_mode(
            PerceptionMode.GROUND_TRUTH
            if self.mode is PerceptionMode.SENSORS
            else PerceptionMode.SENSORS
        )
        return self.mode

    def cycle_ultrasonic_mode(self, sensor_id: str) -> SensorMode:
        order = [SensorMode.ENABLED, SensorMode.DROPOUT, SensorMode.FAULT]
        current = self.ultrasonic.modes[sensor_id]
        new = order[(order.index(current) + 1) % len(order)]
        self.ultrasonic.modes[sensor_id] = new
        self.event_log.append((self.clock.now(), f"{sensor_id} -> {new.value}"))
        return new

    def toggle_camera(self, camera_id: str) -> bool:
        cam = self.camera(camera_id)
        cam.enabled = not cam.enabled
        self.event_log.append((self.clock.now(), f"{camera_id} {'on' if cam.enabled else 'off'}"))
        return cam.enabled

    def add_obstacle(self, x: float, y: float, **kwargs: object) -> SimObstacle:
        obstacle = SimObstacle(id=self.world.new_id(), x=x, y=y, **kwargs)  # type: ignore[arg-type]
        self.world.add(obstacle)
        self._after_edit()
        return obstacle

    def remove_obstacle(self, obstacle_id: str) -> None:
        self.world.remove(obstacle_id)
        self._after_edit()

    def move_obstacle(self, obstacle_id: str, x: float, y: float) -> None:
        o = self.world.get(obstacle_id)
        if o is not None:
            o.x, o.y = x, y
            self._after_edit()

    def set_velocity(self, obstacle_id: str, vx: float, vy: float) -> None:
        o = self.world.get(obstacle_id)
        if o is not None:
            o.vx, o.vy = vx, vy
            o.stop_after_s = None
            self._after_edit()

    def _after_edit(self) -> None:
        if self.paused:
            self.refresh()
