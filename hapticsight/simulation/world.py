"""Ground-truth simulated world (body frame, wearer stationary at the origin).

Distances reported to the pipeline are *clearances*: the gap between the
wearer's torso surface (where the sensors and motors are) and the obstacle's
surface. Obstacles are circles with constant velocity.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from hapticsight.core.geometry import cartesian_to_polar
from hapticsight.core.models import Observation, RangeStatus, SourceType

WEARER_RADIUS_CM = 20.0
CONTACT_CLEARANCE_CM = 5.0


@dataclass
class SimObstacle:
    id: str
    x: float  # cm, + right
    y: float  # cm, + forward
    vx: float = 0.0  # cm/s
    vy: float = 0.0
    radius_cm: float = 20.0
    label: str = ""
    # Probability each ultrasonic ping returns an echo (soft/angled surfaces < 1).
    echo_probability: float = 1.0
    camera_visible: bool = True
    # Override for simulated detector confidence (None = distance-based model).
    camera_confidence: float | None = None
    # Scenario time after which the obstacle stops moving.
    stop_after_s: float | None = None

    @property
    def center_distance(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing_deg(self) -> float:
        return cartesian_to_polar(self.x, self.y)[0]

    @property
    def clearance_cm(self) -> float:
        return max(0.0, self.center_distance - self.radius_cm - WEARER_RADIUS_CM)

    @property
    def closing_speed_cm_s(self) -> float:
        """Rate at which clearance decreases (positive = approaching)."""
        d = self.center_distance
        if d == 0:
            return 0.0
        return -(self.x * self.vx + self.y * self.vy) / d

    @property
    def angular_half_extent_deg(self) -> float:
        d = self.center_distance
        if d <= self.radius_cm:
            return 180.0
        return math.degrees(math.asin(self.radius_cm / d))

    @property
    def speed(self) -> float:
        return math.hypot(self.vx, self.vy)

    def copy(self) -> SimObstacle:
        return replace(self)


class World:
    def __init__(self) -> None:
        self.obstacles: list[SimObstacle] = []
        self.time = 0.0
        self._next_id = 1

    def clear(self) -> None:
        self.obstacles.clear()
        self.time = 0.0
        self._next_id = 1

    def new_id(self) -> str:
        existing = {o.id for o in self.obstacles}
        while (candidate := f"U{self._next_id}") in existing:
            self._next_id += 1
        self._next_id += 1
        return candidate

    def add(self, obstacle: SimObstacle) -> SimObstacle:
        if any(o.id == obstacle.id for o in self.obstacles):
            raise ValueError(f"duplicate obstacle id {obstacle.id!r}")
        self.obstacles.append(obstacle)
        return obstacle

    def remove(self, obstacle_id: str) -> None:
        self.obstacles = [o for o in self.obstacles if o.id != obstacle_id]

    def get(self, obstacle_id: str) -> SimObstacle | None:
        return next((o for o in self.obstacles if o.id == obstacle_id), None)

    def step(self, dt: float) -> None:
        self.time += dt
        for o in self.obstacles:
            if o.stop_after_s is not None and self.time >= o.stop_after_s:
                o.vx = o.vy = 0.0
            o.x += o.vx * dt
            o.y += o.vy * dt
            min_center = WEARER_RADIUS_CM + o.radius_cm + CONTACT_CLEARANCE_CM
            d = o.center_distance
            if d < min_center:
                # Do not let obstacles pass through the wearer: stop at contact.
                scale = min_center / d if d > 0 else 1.0
                o.x, o.y = (o.x * scale, o.y * scale) if d > 0 else (0.0, min_center)
                o.vx = o.vy = 0.0


def ground_truth_observations(world: World, now: float) -> list[Observation]:
    """Perfect observations straight from the world (comparison/oracle mode)."""
    return [
        Observation(
            id=f"gt:{o.id}",
            source=SourceType.SIMULATION,
            bearing_deg=o.bearing_deg,
            timestamp=now,
            confidence=1.0,
            range_status=RangeStatus.VALID,
            distance_cm=o.clearance_cm,
            distance_uncertainty_cm=0.0,
            closing_speed_cm_s=o.closing_speed_cm_s,
            metadata={"label": o.label},
        )
        for o in world.obstacles
    ]
