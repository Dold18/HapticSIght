"""Deterministic, reusable demonstration / regression scenarios.

Obstacles are specified by bearing and *clearance* (gap between torso surface
and obstacle surface). Each scenario lists its expected, explainable outcome;
tests/test_scenarios.py checks those outcomes automatically.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

from hapticsight.core.geometry import polar_to_cartesian
from hapticsight.simulation.world import WEARER_RADIUS_CM, SimObstacle


def toward_wearer(bearing_deg: float, speed: float) -> tuple[float, float]:
    x, y = polar_to_cartesian(bearing_deg, speed)
    return -x, -y


def away_from_wearer(bearing_deg: float, speed: float) -> tuple[float, float]:
    return polar_to_cartesian(bearing_deg, speed)


@dataclass(frozen=True, slots=True)
class ObstacleSpec:
    id: str
    bearing_deg: float
    clearance_cm: float
    velocity: tuple[float, float] = (0.0, 0.0)
    radius_cm: float = 20.0
    label: str = ""
    echo_probability: float = 1.0
    camera_visible: bool = True
    camera_confidence: float | None = None
    stop_after_s: float | None = None

    def build(self) -> SimObstacle:
        center = self.clearance_cm + self.radius_cm + WEARER_RADIUS_CM
        x, y = polar_to_cartesian(self.bearing_deg, center)
        return SimObstacle(
            id=self.id,
            x=x,
            y=y,
            vx=self.velocity[0],
            vy=self.velocity[1],
            radius_cm=self.radius_cm,
            label=self.label,
            echo_probability=self.echo_probability,
            camera_visible=self.camera_visible,
            camera_confidence=self.camera_confidence,
            stop_after_s=self.stop_after_s,
        )


@dataclass(frozen=True, slots=True)
class ScenarioEvent:
    time_s: float
    kind: Literal["ultrasonic_mode", "camera_enabled"]
    target: str
    value: str | bool
    note: str = ""


@dataclass(frozen=True, slots=True)
class Scenario:
    number: int
    key: str
    name: str
    description: str
    expected: tuple[str, ...]
    obstacles: tuple[ObstacleSpec, ...] = ()
    events: tuple[ScenarioEvent, ...] = ()
    duration_s: float = 8.0
    seed: int = 1
    disabled_cameras: tuple[str, ...] = field(default=())


def _crossing_start(y_center: float, x_start: float) -> tuple[float, float]:
    """(bearing, clearance) for a point on a straight crossing line."""
    from hapticsight.core.geometry import cartesian_to_polar

    bearing, dist = cartesian_to_polar(x_start, y_center)
    return bearing, dist - 20.0 - WEARER_RADIUS_CM


_CROSS_B, _CROSS_C = _crossing_start(130.0, -350.0)
_BOUND_B, _BOUND_C = _crossing_start(130.0, -35.0)

SANDBOX = Scenario(
    0,
    "sandbox",
    "Sandbox",
    "Empty environment. Click to place obstacles and experiment.",
    ("No obstacles: all motors off.",),
    duration_s=5.0,
)

SCENARIOS: tuple[Scenario, ...] = (
    Scenario(
        1,
        "ahead",
        "Obstacle directly ahead",
        "A static obstacle 80 cm in front of the wearer.",
        (
            "S0 measures ~80 cm; front camera fuses its bearing.",
            "M0 WARNING (2 Hz pulses); every other motor off.",
        ),
        (ObstacleSpec("A", 0.0, 80.0, label="post"),),
    ),
    Scenario(
        2,
        "approach_right",
        "Approaching from the right",
        "An obstacle walks toward the wearer from the right at 60 cm/s and stops ~42 cm away.",
        (
            "Right side is camera-blind: ultrasonic S1 only.",
            "M2 escalates CAUTION -> WARNING -> CRITICAL with the double-pulse approach cue.",
            "Time-to-contact raises WARNING before the 100 cm band is reached.",
            "After it stops, the approach cue ends; M2 stays CRITICAL (distance < 50 cm).",
        ),
        (
            ObstacleSpec(
                "A",
                90.0,
                330.0,
                velocity=toward_wearer(90.0, 60.0),
                stop_after_s=4.8,
                label="person",
            ),
        ),
        duration_s=7.0,
    ),
    Scenario(
        3,
        "behind",
        "Obstacle behind",
        "A static obstacle 75 cm behind the wearer.",
        ("S2 measures ~75 cm; rear camera fuses.", "M4 WARNING; every other motor off."),
        (ObstacleSpec("A", 180.0, 75.0, label="wall"),),
    ),
    Scenario(
        4,
        "two_directions",
        "Two obstacles, different directions",
        "One obstacle ahead at 150 cm and one on the left at 60 cm.",
        (
            "M0 CAUTION and M6 WARNING simultaneously.",
            "Left obstacle ranks first in the hazard list (higher severity).",
        ),
        (
            ObstacleSpec("A", 0.0, 150.0, label="bench"),
            ObstacleSpec("B", 270.0, 60.0, label="pillar"),
        ),
    ),
    Scenario(
        5,
        "same_motor",
        "Competing for the same motor",
        "Two obstacles in the front sector: one at 10°/150 cm, one at 352°/70 cm.",
        (
            "S0 reports only the nearest echo (~70 cm).",
            "Both camera detections share that echo (flagged shared range).",
            "M0 resolves to the most urgent request: WARNING.",
            "Ground-truth mode: A is CAUTION, B is WARNING -> M0 WARNING from B.",
        ),
        (
            ObstacleSpec("A", 10.0, 150.0, label="far"),
            ObstacleSpec("B", 352.0, 70.0, label="near"),
        ),
    ),
    Scenario(
        6,
        "moving_away",
        "Obstacle moving away",
        "An obstacle starts 35 cm ahead and walks away at 50 cm/s.",
        (
            "M0 steps down CRITICAL -> WARNING -> CAUTION -> off.",
            "Motion is RECEDING: no approach cue, no time-to-contact escalation.",
        ),
        (ObstacleSpec("A", 0.0, 35.0, velocity=away_from_wearer(0.0, 50.0), label="person"),),
        duration_s=6.0,
    ),
    Scenario(
        7,
        "crossing",
        "Crossing in front",
        "An obstacle crosses left-to-right 130 cm in front of the wearer at 80 cm/s.",
        (
            "S0 sees it only while it overlaps the front beam (~1.5 s).",
            "M0 activates (CAUTION/WARNING) only while it is in front.",
            "Fusion can briefly place it on M7 / M1 at the edges of the beam.",
            "Range rate reads approaching until the closest point, then receding.",
            "Ground-truth mode sweeps M7 -> M0 -> M1 within the 200 cm band.",
        ),
        (ObstacleSpec("A", _CROSS_B, _CROSS_C, velocity=(80.0, 0.0), label="cyclist"),),
        duration_s=9.0,
    ),
    Scenario(
        8,
        "dropout",
        "Sensor dropout and fault",
        "Obstacle ahead at 80 cm. S0 drops out at 2 s and returns at 5 s; S1 faults 6.5-8.5 s.",
        (
            "0-2 s: M0 WARNING.",
            "After dropout the last reading goes stale: the warning is NOT kept "
            "indefinitely, and M0 switches to the DEGRADED triple-blip instead of going silent.",
            "Front camera still sees the object but cannot range it: no band is invented.",
            "5 s: S0 returns -> M0 WARNING again. 6.5 s: S1 fault -> M2 DEGRADED.",
        ),
        (ObstacleSpec("A", 0.0, 80.0, label="post"),),
        events=(
            ScenarioEvent(2.0, "ultrasonic_mode", "S0", "dropout", "S0 stops reporting"),
            ScenarioEvent(5.0, "ultrasonic_mode", "S0", "enabled", "S0 restored"),
            ScenarioEvent(6.5, "ultrasonic_mode", "S1", "fault", "S1 reports errors"),
            ScenarioEvent(8.5, "ultrasonic_mode", "S1", "enabled", "S1 restored"),
        ),
        duration_s=10.0,
    ),
    Scenario(
        9,
        "boundary",
        "Near the 359°/0° boundary",
        "An obstacle drifts from 345° through 0° to 15° at ~90 cm.",
        (
            "Bearing wraps 359° -> 0° without glitches.",
            "Only M0 is driven throughout (never M7 or M1).",
        ),
        (ObstacleSpec("A", _BOUND_B, _BOUND_C, velocity=(10.0, 0.0), label="drifter"),),
        duration_s=7.0,
    ),
    Scenario(
        10,
        "uncertain",
        "Conflicting / uncertain observations",
        "Soft obstacle ahead (no echo), low-confidence camera ghost behind, "
        "intermittent echo on the right.",
        (
            "Ahead: camera sees it, ultrasonic does not -> range unknown, no band, M0 off.",
            "Behind: confidence 0.20 < 0.35 -> listed but ignored, M4 off.",
            "Right: echo returns ~50% of pings; release hysteresis keeps M2 WARNING mostly steady.",
        ),
        (
            ObstacleSpec(
                "SOFT", 0.0, 90.0, echo_probability=0.0, camera_confidence=0.85, label="curtain"
            ),
            ObstacleSpec(
                "GHOST",
                180.0,
                120.0,
                echo_probability=0.0,
                camera_confidence=0.2,
                label="reflection",
            ),
            ObstacleSpec("EDGE", 110.0, 70.0, echo_probability=0.5, label="angled panel"),
        ),
        duration_s=8.0,
    ),
)

ALL_SCENARIOS: tuple[Scenario, ...] = (SANDBOX, *SCENARIOS)


def get_scenario(key_or_number: str | int) -> Scenario:
    for s in ALL_SCENARIOS:
        if s.key == key_or_number or s.number == key_or_number or str(s.number) == key_or_number:
            return s
    raise KeyError(f"unknown scenario {key_or_number!r}")
