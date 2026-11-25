"""Physical layout contract shared with the CAD / hardware workstream.

**Do not change these assignments without updating docs/HARDWARE_INTERFACE.md
and coordinating with the hardware build.** Motor order here is also the
channel order on the wire (see hapticsight.protocol) and on the 74HC595 outputs
(QA = M0 ... QH = M7, see docs/HARDWARE_INTERFACE.md).

Bearings follow hapticsight.core.geometry: 0° forward, clockwise positive.
"""

from __future__ import annotations

from dataclasses import dataclass

MOTOR_COUNT = 8
MOTOR_SPACING_DEG = 360.0 / MOTOR_COUNT  # 45°
MOTOR_SECTOR_HALF_WIDTH_DEG = MOTOR_SPACING_DEG / 2.0  # 22.5°


@dataclass(frozen=True, slots=True)
class MotorSpec:
    index: int
    name: str
    bearing_deg: float
    label: str


@dataclass(frozen=True, slots=True)
class UltrasonicSensorSpec:
    index: int
    sensor_id: str
    bearing_deg: float
    label: str


@dataclass(frozen=True, slots=True)
class CameraSpec:
    camera_id: str
    bearing_deg: float
    label: str


MOTORS: tuple[MotorSpec, ...] = (
    MotorSpec(0, "M0", 0.0, "front"),
    MotorSpec(1, "M1", 45.0, "front-right"),
    MotorSpec(2, "M2", 90.0, "right"),
    MotorSpec(3, "M3", 135.0, "rear-right"),
    MotorSpec(4, "M4", 180.0, "rear"),
    MotorSpec(5, "M5", 225.0, "rear-left"),
    MotorSpec(6, "M6", 270.0, "left"),
    MotorSpec(7, "M7", 315.0, "front-left"),
)

ULTRASONIC_SENSORS: tuple[UltrasonicSensorSpec, ...] = (
    UltrasonicSensorSpec(0, "S0", 0.0, "front"),
    UltrasonicSensorSpec(1, "S1", 90.0, "right"),
    UltrasonicSensorSpec(2, "S2", 180.0, "rear"),
    UltrasonicSensorSpec(3, "S3", 270.0, "left"),
)

CAMERAS: tuple[CameraSpec, ...] = (
    CameraSpec("CAM_FRONT", 0.0, "front"),
    CameraSpec("CAM_REAR", 180.0, "rear"),
)

ULTRASONIC_BY_ID: dict[str, UltrasonicSensorSpec] = {s.sensor_id: s for s in ULTRASONIC_SENSORS}
