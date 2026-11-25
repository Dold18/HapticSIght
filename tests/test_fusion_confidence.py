"""Regression tests for the PR #1 review finding on fusion confidence.

A low-confidence camera detection must not gain directional confidence from a
high-confidence echo, and rejecting an unreliable association must never
suppress an independent ultrasonic proximity warning.
"""

from __future__ import annotations

import pytest

from hapticsight.config import HapticSightConfig, config_from_dict
from hapticsight.core.models import RangeStatus, SourceType, UltrasonicReading
from hapticsight.haptics.patterns import PatternId as P
from hapticsight.perception.fusion import FusionConfig, fuse
from hapticsight.pipeline import HapticSightPipeline
from hapticsight.risk.engine import RiskEngine, Severity
from tests.conftest import make_obs


def us(bearing: float = 0.0, distance: float = 80.0, t: float = 0.0, conf: float = 0.9):  # type: ignore[no-untyped-def]
    sid = {0: "S0", 90: "S1", 180: "S2", 270: "S3"}[int(bearing)]
    return make_obs(
        bearing, distance, id=f"us:{sid}", t=t, confidence=conf, source=SourceType.ULTRASONIC,
        bearing_unc=15, distance_unc=2, sensor_id=sid,
    )  # fmt: skip


def cam(id: str, bearing: float, conf: float, t: float = 0.0):  # type: ignore[no-untyped-def]
    return make_obs(
        bearing, None, id=id, t=t, confidence=conf, source=SourceType.CAMERA, bearing_unc=4
    )


def by_source(obs):  # type: ignore[no-untyped-def]
    out: dict[SourceType, list] = {}  # type: ignore[type-arg]
    for o in obs:
        out.setdefault(o.source, []).append(o)
    return out


def test_high_confidence_match_fuses_with_min_confidence() -> None:
    out = fuse([us(conf=0.9)], [cam("c", 5.0, 0.7)], now=0.0)
    (f,) = out
    assert f.source is SourceType.FUSED
    assert f.confidence == pytest.approx(0.7)  # min, not max
    assert f.metadata["camera_confidence"] == 0.7
    assert f.metadata["ultrasonic_confidence"] == 0.9
    assert f.bearing_deg == 5.0 and f.distance_cm == 80.0


def test_fused_confidence_never_exceeds_either_input() -> None:
    for c, u in [(0.95, 0.6), (0.6, 0.95), (0.5, 0.5)]:
        out = fuse([us(conf=u)], [cam("c", 0.0, c)], now=0.0)
        fused = [o for o in out if o.source is SourceType.FUSED]
        assert fused and fused[0].confidence <= min(c, u) + 1e-12


def test_low_confidence_camera_is_not_fused_and_echo_survives() -> None:
    echo = us(conf=0.9)
    out = by_source(fuse([echo], [cam("ghost", 3.0, 0.2)], now=0.0))
    assert SourceType.FUSED not in out
    assert out[SourceType.ULTRASONIC] == [echo]  # untouched, still warns
    assert out[SourceType.CAMERA][0].confidence == 0.2  # no borrowed confidence
    report = RiskEngine().assess([*out[SourceType.ULTRASONIC], *out[SourceType.CAMERA]], 0.0)
    assert report.max_severity is Severity.WARNING
    assert {a.id for a in report.actionable} == {"us:S0"}


def test_unrelated_observations_stay_separate() -> None:
    out = by_source(fuse([us(0.0)], [cam("c", 90.0, 0.9)], now=0.0))
    assert SourceType.FUSED not in out
    assert out[SourceType.CAMERA][0].range_status is RangeStatus.UNKNOWN
    assert out[SourceType.CAMERA][0].distance_cm is None


def test_stale_echo_or_stale_camera_is_not_fused() -> None:
    out = by_source(fuse([us(t=0.0)], [cam("c", 0.0, 0.9, t=1.0)], now=1.0))
    assert SourceType.FUSED not in out
    out2 = by_source(fuse([us(t=1.0)], [cam("c", 0.0, 0.9, t=0.0)], now=1.0))
    assert SourceType.FUSED not in out2
    assert out2[SourceType.ULTRASONIC][0].id == "us:S0"


def test_multiple_camera_objects_in_one_beam() -> None:
    cams = [cam("a", -8.0, 0.8), cam("b", 10.0, 0.6), cam("weak", 2.0, 0.3)]
    out = by_source(fuse([us(distance=70.0)], cams, now=0.0))
    fused = {o.metadata["camera_id"]: o for o in out[SourceType.FUSED]}
    assert set(fused) == {"a", "b"}  # the weak one is not associated
    assert all(f.distance_cm == 70.0 and f.metadata["shared_range"] for f in fused.values())
    assert fused["a"].confidence == pytest.approx(0.8)
    assert fused["b"].confidence == pytest.approx(0.6)
    assert [c.id for c in out[SourceType.CAMERA]] == ["weak"]
    assert SourceType.ULTRASONIC not in out  # confident fused coverage replaces the echo


def test_echo_kept_when_fused_confidence_below_gate() -> None:
    # Echo itself is weak (0.45): fused = min(0.9, 0.45) < gate 0.5, so the
    # echo must not be suppressed even though the camera is inside the beam.
    out = by_source(fuse([us(conf=0.45)], [cam("c", 0.0, 0.9)], now=0.0))
    assert SourceType.ULTRASONIC in out


def test_echo_kept_when_camera_outside_beam_but_inside_gate() -> None:
    out = by_source(fuse([us()], [cam("c", 22.0, 0.9)], now=0.0))
    assert SourceType.FUSED in out and SourceType.ULTRASONIC in out


def test_config_invariants_prevent_hidden_warnings() -> None:
    with pytest.raises(ValueError, match="min_camera_confidence"):
        config_from_dict({"fusion": {"min_camera_confidence": 0.2}})
    with pytest.raises(ValueError, match="stale_after_s"):
        config_from_dict({"fusion": {"max_camera_age_s": 0.6}})
    with pytest.raises(ValueError):
        FusionConfig(min_camera_confidence=1.5)
    HapticSightConfig()  # defaults are consistent


def readings(t: float, s0: float | None = None, s2: float | None = None) -> list[UltrasonicReading]:
    def r(sid: str, d: float | None) -> UltrasonicReading:
        if d is None:
            return UltrasonicReading(sid, t, RangeStatus.NO_ECHO)
        return UltrasonicReading(sid, t, RangeStatus.VALID, d)

    return [r("S0", s0), r("S1", None), r("S2", s2), r("S3", None)]


def test_pipeline_low_confidence_camera_does_not_suppress_warning() -> None:
    p = HapticSightPipeline()
    result = p.process(0.0, readings(0.0, s0=80), [cam("CAM_FRONT:T1", 3.0, 0.2)])
    assert result.frame.pattern_ids[0] is P.WARNING
    assert result.frame.channels[0].sources == ("us:S0",)
