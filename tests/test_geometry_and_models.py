from __future__ import annotations

import math

import pytest

from hapticsight.core.clock import ManualClock, MonotonicClock
from hapticsight.core.geometry import (
    angular_distance,
    bearing_within,
    cartesian_to_polar,
    normalize_bearing,
    polar_to_cartesian,
    signed_angle_difference,
)
from hapticsight.core.layout import MOTORS, ULTRASONIC_SENSORS
from hapticsight.core.models import (
    Observation,
    RangeStatus,
    SensorStatus,
    SourceType,
    UltrasonicReading,
)


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        (0, 0),
        (360, 0),
        (720, 0),
        (-1, 359),
        (-360, 0),
        (361, 1),
        (-90, 270),
        (359.999, 359.999),
        (1e-16 - 360, 0),
    ],
)
def test_normalize_bearing(raw: float, expected: float) -> None:
    assert normalize_bearing(raw) == pytest.approx(expected)
    assert 0 <= normalize_bearing(raw) < 360


def test_normalize_never_returns_360_for_tiny_negative() -> None:
    assert normalize_bearing(-1e-15) < 360.0


def test_normalize_rejects_non_finite() -> None:
    with pytest.raises(ValueError):
        normalize_bearing(math.nan)
    with pytest.raises(ValueError):
        normalize_bearing(math.inf)


@pytest.mark.parametrize(
    ("target", "ref", "expected"),
    [
        (10, 350, 20),
        (350, 10, -20),
        (180, 0, 180),
        (0, 180, 180),
        (90, 0, 90),
        (270, 0, -90),
    ],
)
def test_signed_angle_difference_wraps(target: float, ref: float, expected: float) -> None:
    assert signed_angle_difference(target, ref) == pytest.approx(expected)


def test_angular_distance_and_within_across_zero() -> None:
    assert angular_distance(359, 1) == pytest.approx(2)
    assert bearing_within(355, 0, 5)
    assert bearing_within(5, 0, 5)
    assert not bearing_within(6, 0, 5)


@pytest.mark.parametrize("bearing", [0, 45, 90, 135, 180, 225, 270, 315, 359.5])
def test_polar_cartesian_round_trip(bearing: float) -> None:
    x, y = polar_to_cartesian(bearing, 100)
    b, d = cartesian_to_polar(x, y)
    assert d == pytest.approx(100)
    assert angular_distance(b, bearing) == pytest.approx(0, abs=1e-9)


def test_axes_convention() -> None:
    # +y forward, +x right, clockwise bearings
    assert cartesian_to_polar(0, 100)[0] == pytest.approx(0)
    assert cartesian_to_polar(100, 0)[0] == pytest.approx(90)
    assert cartesian_to_polar(0, -100)[0] == pytest.approx(180)
    assert cartesian_to_polar(-100, 0)[0] == pytest.approx(270)
    assert cartesian_to_polar(0, 0) == (0.0, 0.0)


def test_layout_contract_is_unchanged() -> None:
    """Guards the CAD/hardware contract (docs/HARDWARE_INTERFACE.md)."""
    assert [(m.name, m.bearing_deg) for m in MOTORS] == [
        ("M0", 0),
        ("M1", 45),
        ("M2", 90),
        ("M3", 135),
        ("M4", 180),
        ("M5", 225),
        ("M6", 270),
        ("M7", 315),
    ]
    assert [(s.sensor_id, s.bearing_deg) for s in ULTRASONIC_SENSORS] == [
        ("S0", 0),
        ("S1", 90),
        ("S2", 180),
        ("S3", 270),
    ]


def test_observation_normalizes_bearing() -> None:
    o = Observation("a", SourceType.CAMERA, bearing_deg=-10, timestamp=0)
    assert o.bearing_deg == pytest.approx(350)
    assert not o.has_range
    assert o.distance_cm is None


def test_camera_observation_does_not_need_distance() -> None:
    o = Observation("c", SourceType.CAMERA, 5, 0, range_status=RangeStatus.UNKNOWN)
    assert o.range_status is RangeStatus.UNKNOWN


@pytest.mark.parametrize(
    "kwargs",
    [
        {"confidence": 1.5},
        {"confidence": -0.1},
        {"confidence": math.nan},
        {"range_status": RangeStatus.VALID},  # missing distance
        {"range_status": RangeStatus.VALID, "distance_cm": -1.0},
        {"range_status": RangeStatus.VALID, "distance_cm": math.inf},
        {"range_status": RangeStatus.UNKNOWN, "distance_cm": 10.0},
        {"range_status": RangeStatus.NO_ECHO, "distance_cm": 10.0},
        {"bearing_uncertainty_deg": -1.0},
        {"closing_speed_cm_s": math.nan},
    ],
)
def test_observation_validation(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        Observation("x", SourceType.ULTRASONIC, 0, 0, **kwargs)  # type: ignore[arg-type]


def test_observation_age_and_metadata_is_readonly() -> None:
    o = Observation("a", SourceType.SIMULATION, 0, 10.0, metadata={"k": 1})
    assert o.age(10.5) == pytest.approx(0.5)
    assert o.age(9.0) == 0.0  # never negative
    with pytest.raises(TypeError):
        o.metadata["k"] = 2  # type: ignore[index]


def test_ultrasonic_reading_validation() -> None:
    UltrasonicReading("S0", 0, RangeStatus.VALID, 50.0)
    UltrasonicReading("S0", 0, RangeStatus.NO_ECHO)
    with pytest.raises(ValueError):
        UltrasonicReading("S0", 0, RangeStatus.VALID)
    with pytest.raises(ValueError):
        UltrasonicReading("S0", 0, RangeStatus.NO_ECHO, 50.0)
    with pytest.raises(ValueError):
        UltrasonicReading("S0", 0, RangeStatus.UNKNOWN)


def test_sensor_status_degraded_flags() -> None:
    assert SensorStatus.STALE.is_degraded
    assert SensorStatus.FAULT.is_degraded
    assert SensorStatus.NEVER_SEEN.is_degraded
    assert not SensorStatus.OK.is_degraded
    assert not SensorStatus.NO_ECHO.is_degraded


def test_clocks() -> None:
    c = ManualClock()
    assert c.now() == 0
    c.advance(0.25)
    assert c.now() == 0.25
    with pytest.raises(ValueError):
        c.advance(-1)
    with pytest.raises(ValueError):
        c.set(0.1)
    m = MonotonicClock()
    assert m.now() <= m.now()
