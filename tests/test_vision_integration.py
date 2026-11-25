"""Camera observations through the *shared* pipeline: fusion, risk, haptics, advisory cue."""

from __future__ import annotations

import time
from dataclasses import replace

import pytest

from hapticsight.config import HapticSightConfig
from hapticsight.core.models import RangeStatus, SourceType, UltrasonicReading
from hapticsight.haptics.encoder import HapticConfig, HapticEncoder
from hapticsight.haptics.patterns import DEFAULT_PATTERNS
from hapticsight.haptics.patterns import PatternId as P
from hapticsight.pipeline import HapticSightPipeline
from hapticsight.protocol.codec import decode, encode
from hapticsight.protocol.messages import HapticCommandMessage
from hapticsight.risk.engine import RiskEngine
from hapticsight.vision.camera import ThreadedCapture
from hapticsight.vision.config import VisionConfig
from hapticsight.vision.detector import ScriptedDetector
from hapticsight.vision.factory import SyntheticCapture
from hapticsight.vision.geometry import CameraCalibration
from hapticsight.vision.processor import CameraProcessor, VisionSystem
from hapticsight.vision.synthetic import MarkerDetector
from tests.conftest import make_obs
from tests.vision_helpers import FakeCapture, OpenerLog, frame, raw, wait_until

W = 640


def us(t: float, **ranges: float | None) -> list[UltrasonicReading]:
    out = []
    for sid in ("S0", "S1", "S2", "S3"):
        d = ranges.get(sid)
        out.append(
            UltrasonicReading(sid, t, RangeStatus.VALID, d)
            if d is not None
            else UltrasonicReading(sid, t, RangeStatus.NO_ECHO)
        )
    return out


def processor(camera_id: str = "CAM_FRONT", hfov: float = 60.0, script=None):  # type: ignore[no-untyped-def]
    cfg = VisionConfig().camera(camera_id)
    cfg = replace(cfg, calibration=CameraCalibration(hfov_deg=hfov))
    return CameraProcessor(cfg, ScriptedDetector(script or {}))


def box_at_bearing(geo, bearing: float, half: float = 25.0):  # type: ignore[no-untyped-def]
    u = geo.column_for_body_bearing(bearing, W)
    return (u - half, 60.0, u + half, 300.0)


def camera_obs(proc: CameraProcessor, bearing: float, label: str = "person", n: int = 3):  # type: ignore[no-untyped-def]
    x1, y1, x2, y2 = box_at_bearing(proc.geometry, bearing)
    proc.detector = ScriptedDetector({i: [raw(label, x1, y1, x2, y2)] for i in range(n)})
    result = None
    for i in range(n):
        result = proc.process(frame(proc.camera.camera_id, t=i / 15, seq=i + 1), now=i / 15)
    assert result is not None
    return result


def test_processor_produces_valid_camera_observations() -> None:
    proc = processor()
    result = camera_obs(proc, 10.0)
    (obs,) = result.observations
    assert obs.source is SourceType.CAMERA and obs.range_status is RangeStatus.UNKNOWN
    assert obs.distance_cm is None and obs.closing_speed_cm_s is None
    assert obs.bearing_deg == pytest.approx(10.0, abs=0.01)
    assert obs.sensor_id == "CAM_FRONT" and obs.id == "CAM_FRONT:T1"
    assert obs.timestamp == pytest.approx(2 / 15)
    assert result.detections[0].bearing_deg == pytest.approx(10.0, abs=0.01)


def test_stale_frames_are_not_processed() -> None:
    proc = processor(script={0: [raw("person", 0, 0, 50, 50)]})
    result = proc.process(frame(t=0.0), now=1.0)  # 1 s old > max_frame_age_s 0.5
    assert result.stale and result.detections == () and proc.detector.calls == 0  # type: ignore[attr-defined]


@pytest.mark.parametrize(
    ("camera_id", "bearing", "echo", "motor"),
    [
        ("CAM_FRONT", 0.0, {"S0": 80.0}, 0),
        ("CAM_FRONT", 20.0, {"S0": 80.0}, 0),  # fused, still inside M0's sector
        ("CAM_REAR", 195.0, {"S2": 80.0}, 4),
        ("CAM_REAR", 165.0, {"S2": 80.0}, 4),
    ],
)
def test_haptic_mapping_after_fusion(
    camera_id: str, bearing: float, echo: dict[str, float], motor: int
) -> None:
    proc = processor(camera_id)
    result = camera_obs(proc, bearing)
    out = HapticSightPipeline().process(result.capture_ts, us(result.capture_ts, **echo),
                                        result.observations)  # fmt: skip
    fused = [a for a in out.report.assessments if a.source is SourceType.FUSED]
    assert fused and fused[0].bearing_deg == pytest.approx(bearing % 360, abs=0.01)
    assert out.frame.pattern_ids[motor] is P.WARNING
    assert sum(c.active for c in out.frame.channels) >= 1


def test_fused_camera_bearing_reaches_diagonal_motor() -> None:
    proc = processor()
    result = camera_obs(proc, 24.0)  # outside S0 beam (15°) but inside gate (25°)
    out = HapticSightPipeline().process(result.capture_ts, us(result.capture_ts, S0=70.0),
                                        result.observations)  # fmt: skip
    assert out.frame.pattern_ids[1] is P.WARNING  # fused camera bearing -> M1
    assert out.frame.pattern_ids[0] is P.WARNING  # raw echo kept -> M0


def test_camera_only_object_does_not_warn_by_default() -> None:
    proc = processor()
    result = camera_obs(proc, 0.0, n=30)
    out = HapticSightPipeline().process(result.capture_ts, us(result.capture_ts),
                                        result.observations)  # fmt: skip
    assert out.frame.pattern_string() == "00000000"
    (a,) = out.report.assessments
    assert a.proximity_severity is None


def test_missing_camera_leaves_ultrasonic_warnings_unchanged() -> None:
    with_cam = HapticSightPipeline().process(0.0, us(0.0, S1=60.0), [])
    assert with_cam.frame.pattern_ids[2] is P.WARNING
    assert with_cam.report.degraded_sensors == ()  # cameras are not ultrasonic coverage


# ------------------------------------------------------------ advisory cue


def advisory_config(**kw: object) -> HapticConfig:
    return HapticConfig(advisory_cue_enabled=True, **kw)  # type: ignore[arg-type]


def cam_track(bearing: float, conf: float = 0.8, age: float = 2.0, id: str = "CAM_FRONT:T1"):  # type: ignore[no-untyped-def]
    return make_obs(
        bearing, None, id=id, confidence=conf, source=SourceType.CAMERA, bearing_unc=4,
        metadata={"track_age_s": age, "label": "person"},
    )  # fmt: skip


def encode_with(cfg: HapticConfig, *obs, health=()):  # type: ignore[no-untyped-def]
    return HapticEncoder(cfg).encode(RiskEngine().assess(list(obs), 0.0, health), 0.0)


def test_advisory_disabled_by_default() -> None:
    assert HapticConfig().advisory_cue_enabled is False
    assert encode_with(HapticConfig(), cam_track(0.0)).pattern_string() == "00000000"


def test_advisory_marks_single_forward_object_distinctly() -> None:
    frame_ = encode_with(advisory_config(), cam_track(5.0), cam_track(-30.0, id="b"))
    assert frame_.pattern_ids[0] is P.ADVISORY
    assert frame_.channels[0].advisory and not frame_.channels[0].severity
    assert sum(c.active for c in frame_.channels) == 1  # max one channel: no flooding
    pat = DEFAULT_PATTERNS[P.ADVISORY]
    assert pat.duty_cycle < DEFAULT_PATTERNS[P.CAUTION].duty_cycle
    assert pat.cycle_ms > DEFAULT_PATTERNS[P.DEGRADED].cycle_ms


@pytest.mark.parametrize(
    "obs",
    [
        cam_track(0.0, conf=0.5),  # not confident enough
        cam_track(0.0, age=0.3),  # not tracked long enough
        cam_track(120.0),  # not in the forward path
    ],
)
def test_advisory_requires_confident_persistent_forward_track(obs) -> None:  # type: ignore[no-untyped-def]
    assert encode_with(advisory_config(), obs).pattern_string() == "00000000"


def test_advisory_never_overrides_hazard_or_degraded() -> None:
    ranged = make_obs(0.0, 150.0, id="us:S0", source=SourceType.ULTRASONIC)
    f1 = encode_with(advisory_config(), cam_track(0.0), ranged)
    assert f1.pattern_ids[0] is P.CAUTION
    from hapticsight.core.models import SensorHealth, SensorStatus

    h = [SensorHealth("S0", 0.0, SensorStatus.STALE, None, None)]
    f2 = encode_with(advisory_config(), cam_track(0.0), health=h)
    assert f2.pattern_ids[0] is P.DEGRADED


def test_advisory_slot_is_valid_on_the_wire() -> None:
    frame_ = encode_with(advisory_config(), cam_track(0.0))
    line = encode(HapticCommandMessage(1, 500, tuple(int(p) for p in frame_.pattern_ids)))
    assert line.startswith("$HAP,1,500,80000000*")
    assert decode(line).patterns[0] == 8  # type: ignore[union-attr]


def test_advisory_through_pipeline_config() -> None:
    cfg = HapticSightConfig(haptics=advisory_config())
    out = HapticSightPipeline(cfg).process(0.0, us(0.0), [cam_track(0.0)])
    assert out.frame.pattern_ids[0] is P.ADVISORY


# ------------------------------------------------------------ concurrent system


def test_vision_system_runs_two_cameras_concurrently_and_stops_cleanly() -> None:
    cfg = VisionConfig()
    caps = [
        ThreadedCapture(cfg.camera("CAM_FRONT"), opener=lambda c: SyntheticCapture()),
        ThreadedCapture(cfg.camera("CAM_REAR"), opener=lambda c: SyntheticCapture()),
    ]
    system = VisionSystem(cfg, MarkerDetector(), captures=caps)
    with system:
        assert wait_until(lambda: len(system.snapshot().results) == 2, timeout=5.0)
        assert wait_until(
            lambda: (
                {o.sensor_id for o in system.camera_observations(time.monotonic())}
                == {"CAM_FRONT", "CAM_REAR"}
            ),
            timeout=5.0,
        )
        snap = system.snapshot()
        assert all(p.processed_frames > 0 for p in snap.perf.values())
        rear = [
            o for o in system.camera_observations(time.monotonic()) if o.sensor_id == "CAM_REAR"
        ]
        assert all(90 < o.bearing_deg < 270 for o in rear)
    assert not any(c.running for c in caps)
    assert system._worker is None


def test_one_camera_missing_does_not_block_the_other() -> None:
    cfg = VisionConfig()
    caps = [
        ThreadedCapture(cfg.camera("CAM_FRONT"), opener=lambda c: SyntheticCapture()),
        ThreadedCapture(
            replace(cfg.camera("CAM_REAR"), reconnect_interval_s=0.05),
            opener=OpenerLog(FakeCapture(opened=False)),
        ),
    ]
    system = VisionSystem(cfg, MarkerDetector(), captures=caps)
    with system:
        assert wait_until(lambda: "CAM_FRONT" in system.snapshot().results, timeout=5.0)
        snap = system.snapshot()
        assert snap.statuses["CAM_REAR"].state.value == "unavailable"
        assert "CAM_REAR" not in snap.results
        assert {o.sensor_id for o in system.camera_observations(time.monotonic())} <= {"CAM_FRONT"}


def test_old_results_are_dropped_when_camera_goes_away() -> None:
    cfg = VisionConfig()
    caps = [ThreadedCapture(cfg.camera("CAM_FRONT"), opener=lambda c: SyntheticCapture())]
    system = VisionSystem(cfg, MarkerDetector(), captures=caps)
    with system:
        assert wait_until(lambda: bool(system.camera_observations(time.monotonic())), 5.0)
    later = time.monotonic() + 5.0
    assert system.camera_observations(later, max_age_s=1.0) == []
