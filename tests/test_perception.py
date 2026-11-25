from __future__ import annotations

import pytest

from hapticsight.core.models import RangeStatus, SensorStatus, SourceType, UltrasonicReading
from hapticsight.perception.buffer import ObservationBuffer
from hapticsight.perception.fusion import FusionConfig, fuse
from hapticsight.perception.ultrasonic import (
    RangeRateConfig,
    RangeRateEstimator,
    UltrasonicConfig,
    UltrasonicFrontEnd,
)
from tests.conftest import make_obs

# ------------------------------------------------------------ range rate


def feed(est: RangeRateEstimator, samples: list[tuple[float, float]]) -> float | None:
    out = None
    for t, d in samples:
        out = est.update("k", t, d)
    return out


def test_closing_speed_unknown_until_enough_samples() -> None:
    est = RangeRateEstimator(RangeRateConfig(min_samples=4))
    # 5 Hz, the firmware's per-sensor rate
    assert feed(est, [(0.0, 200), (0.2, 190), (0.4, 180)]) is None
    assert est.update("k", 0.6, 170) == pytest.approx(50)


def test_closing_speed_sign_convention() -> None:
    est = RangeRateEstimator()
    approaching = feed(est, [(i * 0.1, 200 - 6 * i) for i in range(6)])
    assert approaching == pytest.approx(60)
    est.reset()
    receding = feed(est, [(i * 0.1, 100 + 3 * i) for i in range(6)])
    assert receding == pytest.approx(-30)


def test_range_jump_restarts_history() -> None:
    est = RangeRateEstimator(RangeRateConfig(max_plausible_rate_cm_s=300))
    feed(est, [(i * 0.1, 200.0) for i in range(5)])
    # Jump of 150 cm in 0.1 s = 1500 cm/s: a different object; estimate restarts.
    assert est.update("k", 0.5, 50.0) is None


def test_window_drops_old_samples() -> None:
    est = RangeRateEstimator(RangeRateConfig(window_s=0.3, min_samples=3, min_span_s=0.1))
    feed(est, [(0.0, 300), (0.1, 300), (0.2, 300), (0.3, 300)])
    # samples older than 0.3 s fall away; slope from recent samples only
    assert feed(est, [(0.4, 290), (0.5, 280), (0.6, 270)]) == pytest.approx(100)


def test_duplicate_timestamp_is_ignored() -> None:
    est = RangeRateEstimator()
    feed(est, [(i * 0.1, 200 - 5 * i) for i in range(5)])
    assert est.update("k", 0.4, 0.0) == pytest.approx(50)


# ------------------------------------------------------------ front end


def test_front_end_converts_valid_reading_to_sector_observation() -> None:
    fe = UltrasonicFrontEnd(UltrasonicConfig(beam_half_angle_deg=15))
    fe.ingest([UltrasonicReading("S1", 1.0, RangeStatus.VALID, 80.0)])
    (obs,) = fe.observations()
    assert obs.source is SourceType.ULTRASONIC
    assert obs.bearing_deg == 90
    assert obs.bearing_uncertainty_deg == 15
    assert obs.distance_cm == 80
    assert obs.closing_speed_cm_s is None  # one sample: unknown, not zero


def test_front_end_no_echo_and_fault_produce_no_observation() -> None:
    fe = UltrasonicFrontEnd()
    fe.ingest(
        [
            UltrasonicReading("S0", 1.0, RangeStatus.NO_ECHO),
            UltrasonicReading("S1", 1.0, RangeStatus.INVALID),
        ]
    )
    assert fe.observations() == []
    health = {h.sensor_id: h.status for h in fe.health(1.0)}
    assert health["S0"] is SensorStatus.NO_ECHO
    assert health["S1"] is SensorStatus.FAULT
    assert health["S2"] is SensorStatus.NEVER_SEEN


def test_front_end_sanitizes_out_of_range() -> None:
    fe = UltrasonicFrontEnd(UltrasonicConfig(min_range_cm=2, max_range_cm=400))
    fe.ingest(
        [
            UltrasonicReading("S0", 1.0, RangeStatus.VALID, 1.0),
            UltrasonicReading("S1", 1.0, RangeStatus.VALID, 450.0),
        ]
    )
    assert fe.latest("S0").status is RangeStatus.INVALID  # type: ignore[union-attr]
    assert fe.latest("S1").status is RangeStatus.NO_ECHO  # type: ignore[union-attr]
    assert fe.observations() == []


def test_front_end_staleness_and_dropout() -> None:
    fe = UltrasonicFrontEnd(UltrasonicConfig(stale_after_s=0.4))
    fe.ingest([UltrasonicReading("S0", 1.0, RangeStatus.VALID, 80.0)])
    assert fe.health(1.3)[0].status is SensorStatus.OK
    h = fe.health(1.5)[0]
    assert h.status is SensorStatus.STALE
    assert h.age_s == pytest.approx(0.5)
    # The observation is still reported, carrying its original timestamp, so
    # the risk engine can see that it is stale.
    assert fe.observations()[0].timestamp == 1.0


def test_front_end_ignores_out_of_order_reading() -> None:
    fe = UltrasonicFrontEnd()
    fe.ingest([UltrasonicReading("S0", 2.0, RangeStatus.VALID, 80.0)])
    fe.ingest([UltrasonicReading("S0", 1.0, RangeStatus.VALID, 20.0)])
    assert fe.latest("S0").distance_cm == 80.0  # type: ignore[union-attr]


def test_front_end_estimates_closing_speed_and_resets_on_no_echo() -> None:
    fe = UltrasonicFrontEnd()
    fe.ingest(UltrasonicReading("S0", i * 0.1, RangeStatus.VALID, 200.0 - 5 * i) for i in range(6))
    assert fe.observations()[0].closing_speed_cm_s == pytest.approx(50)
    fe.ingest([UltrasonicReading("S0", 0.6, RangeStatus.NO_ECHO)])
    fe.ingest([UltrasonicReading("S0", 0.7, RangeStatus.VALID, 170.0)])
    assert fe.observations()[0].closing_speed_cm_s is None


def test_front_end_rejects_unknown_sensor() -> None:
    with pytest.raises(KeyError):
        UltrasonicFrontEnd().ingest([UltrasonicReading("S9", 0, RangeStatus.NO_ECHO)])


# ------------------------------------------------------------ buffer


def test_buffer_expires_old_observations() -> None:
    buf = ObservationBuffer(expire_after_s=0.5)
    buf.update([make_obs(id="a", t=0.0), make_obs(id="b", t=0.4)])
    assert [o.id for o in buf.current(0.45)] == ["a", "b"]
    assert [o.id for o in buf.current(0.6)] == ["b"]
    buf.update([make_obs(id="b", t=0.2)])  # older update must not replace newer
    assert buf.current(0.6)[0].timestamp == 0.4


# ------------------------------------------------------------ fusion


def us(sensor: str, bearing: float, distance: float, t: float = 0.0, closing: float | None = None):  # type: ignore[no-untyped-def]
    return make_obs(
        bearing,
        distance,
        id=f"us:{sensor}",
        t=t,
        confidence=0.9,
        closing=closing,
        source=SourceType.ULTRASONIC,
        bearing_unc=15,
        distance_unc=2,
        sensor_id=sensor,
    )


def cam(id: str, bearing: float, t: float = 0.0, confidence: float = 0.8):  # type: ignore[no-untyped-def]
    return make_obs(
        bearing, None, id=id, t=t, confidence=confidence, source=SourceType.CAMERA, bearing_unc=3
    )


def test_fusion_takes_bearing_from_camera_and_range_from_ultrasonic() -> None:
    out = fuse([us("S0", 0, 80, closing=30)], [cam("c1", 8)])
    (f,) = out
    assert f.source is SourceType.FUSED
    assert f.bearing_deg == 8
    assert f.distance_cm == 80
    assert f.closing_speed_cm_s == 30
    assert f.metadata["shared_range"] is False


def test_fusion_keeps_camera_only_detection_without_range() -> None:
    out = fuse([us("S1", 90, 80)], [cam("c1", 0)])
    sources = sorted(o.source.value for o in out)
    assert sources == ["camera", "ultrasonic"]
    camera_obs = next(o for o in out if o.source is SourceType.CAMERA)
    assert camera_obs.distance_cm is None


def test_fusion_keeps_raw_ultrasonic_when_camera_is_outside_beam() -> None:
    # 22° is inside the gate (15 + 10) but outside the 15° beam
    out = fuse([us("S0", 0, 80)], [cam("c1", 22)])
    assert {o.source for o in out} == {SourceType.FUSED, SourceType.ULTRASONIC}


def test_fusion_flags_shared_echo_between_two_camera_objects() -> None:
    out = fuse([us("S0", 0, 70)], [cam("a", -8), cam("b", 10)])
    fused = [o for o in out if o.source is SourceType.FUSED]
    assert len(fused) == 2
    assert all(o.distance_cm == 70 and o.metadata["shared_range"] for o in fused)


def test_fusion_uses_oldest_timestamp() -> None:
    (f,) = fuse([us("S0", 0, 80, t=1.0)], [cam("c1", 0, t=2.0)])
    assert f.timestamp == 1.0


def test_fusion_disabled_passes_through() -> None:
    out = fuse([us("S0", 0, 80)], [cam("c1", 0)], FusionConfig(enabled=False))
    assert {o.source for o in out} == {SourceType.ULTRASONIC, SourceType.CAMERA}


def test_fusion_ignores_stale_echo_when_now_given() -> None:
    stale_echo = us("S0", 0, 80, t=0.0)
    out = fuse([stale_echo], [cam("c1", 0, t=1.0)], now=1.0)
    # Camera stays bearing-only; the stale echo is passed through untouched.
    assert {o.source for o in out} == {SourceType.ULTRASONIC, SourceType.CAMERA}
    assert next(o for o in out if o.source is SourceType.CAMERA).distance_cm is None
