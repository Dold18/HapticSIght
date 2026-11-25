from __future__ import annotations

import pytest

from hapticsight.core.models import SensorHealth, SensorStatus
from hapticsight.risk.engine import (
    Motion,
    ProximityThresholds,
    RiskConfig,
    RiskEngine,
    Severity,
)
from tests.conftest import make_obs


@pytest.mark.parametrize(
    ("distance", "expected"),
    [
        (0, Severity.CRITICAL),
        (49.9, Severity.CRITICAL),
        (50, Severity.WARNING),
        (99.9, Severity.WARNING),
        (100, Severity.CAUTION),
        (200, Severity.CAUTION),
        (200.1, Severity.NONE),
        (1000, Severity.NONE),
    ],
)
def test_default_proximity_boundaries(distance: float, expected: Severity) -> None:
    assert ProximityThresholds().classify(distance) is expected


def test_thresholds_are_configurable_and_validated() -> None:
    t = ProximityThresholds(caution_cm=300, warning_cm=150, critical_cm=75)
    assert t.classify(250) is Severity.CAUTION
    assert t.classify(140) is Severity.WARNING
    with pytest.raises(ValueError):
        ProximityThresholds(caution_cm=100, warning_cm=150, critical_cm=50)


def test_risk_config_validation() -> None:
    with pytest.raises(ValueError):
        RiskConfig(ttc_critical_s=3, ttc_warning_s=2)
    with pytest.raises(ValueError):
        RiskConfig(stale_after_s=3, expire_after_s=2)


def test_conservative_distance_uses_uncertainty() -> None:
    engine = RiskEngine()
    a = engine.assess_one(make_obs(distance=102, distance_unc=3), now=0)
    assert a.effective_distance_cm == pytest.approx(99)
    assert a.severity is Severity.WARNING
    loose = RiskEngine(RiskConfig(conservative_distance=False))
    assert loose.assess_one(make_obs(distance=102, distance_unc=3), 0).severity is Severity.CAUTION


def test_missing_range_gets_no_band() -> None:
    a = RiskEngine().assess_one(make_obs(distance=None), now=0)
    assert a.proximity_severity is None
    assert a.severity is Severity.NONE
    assert a.effective_distance_cm is None
    assert any("no proximity band" in r for r in a.reasons)


@pytest.mark.parametrize(
    ("closing", "motion"),
    [
        (None, Motion.UNKNOWN),
        (0.0, Motion.STEADY),
        (14.9, Motion.STEADY),
        (15.0, Motion.APPROACHING),
        (-15.0, Motion.RECEDING),
        (-50.0, Motion.RECEDING),
    ],
)
def test_motion_classification(closing: float | None, motion: Motion) -> None:
    assert RiskEngine().assess_one(make_obs(closing=closing), 0).motion is motion


def test_camera_only_object_is_never_approaching() -> None:
    # Visible, high-confidence, but no closing speed: motion stays UNKNOWN.
    a = RiskEngine().assess_one(make_obs(distance=None, confidence=1.0), 0)
    assert a.motion is Motion.UNKNOWN
    assert not a.is_approaching


def test_time_to_contact_escalates_but_never_lowers() -> None:
    engine = RiskEngine()
    # 180 cm at 100 cm/s -> TTC 1.8 s -> WARNING (proximity alone: CAUTION)
    a = engine.assess_one(make_obs(distance=180, closing=100), 0)
    assert a.proximity_severity is Severity.CAUTION
    assert a.time_to_contact_s == pytest.approx(1.8)
    assert a.severity is Severity.WARNING
    # 300 cm at 150 cm/s -> TTC 2.0 s -> WARNING even outside every distance band
    b = engine.assess_one(make_obs(distance=300, closing=150), 0)
    assert b.proximity_severity is Severity.NONE
    assert b.severity is Severity.WARNING
    # 40 cm, slow approach: TTC large, proximity CRITICAL stays CRITICAL
    c = engine.assess_one(make_obs(distance=40, closing=16), 0)
    assert c.severity is Severity.CRITICAL
    # receding never escalates
    d = engine.assess_one(make_obs(distance=150, closing=-100), 0)
    assert d.severity is Severity.CAUTION
    assert d.time_to_contact_s is None


def test_stale_observation_cannot_warn() -> None:
    engine = RiskEngine(RiskConfig(stale_after_s=0.5, expire_after_s=2.0))
    obs = make_obs(distance=30, t=0.0)
    assert engine.assess_one(obs, now=0.5).severity is Severity.CRITICAL
    stale = engine.assess_one(obs, now=0.51)
    assert stale.is_stale
    assert stale.severity is Severity.NONE
    assert stale.proximity_severity is Severity.CRITICAL  # still visible in diagnostics


def test_expired_observations_are_dropped_and_counted() -> None:
    report = RiskEngine().assess([make_obs(t=0.0), make_obs(id="b", t=5.0)], now=5.0)
    assert [a.id for a in report.assessments] == ["b"]
    assert report.expired_count == 1


def test_low_confidence_cannot_warn() -> None:
    a = RiskEngine().assess_one(make_obs(distance=30, confidence=0.2), 0)
    assert a.is_low_confidence
    assert a.severity is Severity.NONE


def test_prioritisation_order() -> None:
    engine = RiskEngine()
    obs = [
        make_obs(0, 150, id="caution_ahead"),
        make_obs(180, 70, id="warning_behind"),
        make_obs(90, 70, id="warning_right_approaching", closing=20),
        make_obs(270, 40, id="critical_left"),
        make_obs(0, None, id="camera_only"),
        make_obs(0, 10, id="stale", t=-10),
    ]
    report = engine.assess(obs, now=0)
    order = [a.id for a in report.assessments]
    assert order[:4] == [
        "critical_left",
        "warning_right_approaching",
        "warning_behind",
        "caution_ahead",
    ]
    assert report.max_severity is Severity.CRITICAL
    assert {a.id for a in report.actionable} == set(order[:4])


def test_ties_broken_by_distance_then_forward_path() -> None:
    engine = RiskEngine()
    report = engine.assess(
        [make_obs(180, 80, id="behind"), make_obs(0, 80, id="ahead"), make_obs(90, 60, id="near")],
        now=0,
    )
    assert [a.id for a in report.assessments] == ["near", "ahead", "behind"]


def test_report_exposes_degraded_sensors() -> None:
    health = [
        SensorHealth("S0", 0, SensorStatus.OK, 0.0, 0.0),
        SensorHealth("S1", 90, SensorStatus.STALE, 0.0, 1.0),
    ]
    report = RiskEngine().assess([], now=1.0, sensor_health=health)
    assert [h.sensor_id for h in report.degraded_sensors] == ["S1"]
    assert report.max_severity is Severity.NONE
