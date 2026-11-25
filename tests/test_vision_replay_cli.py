from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from hapticsight.core.geometry import angular_distance
from hapticsight.pipeline import HapticSightPipeline
from hapticsight.simulation.manual_ultrasonic import ManualMode, ManualUltrasonic
from hapticsight.vision.camera import CameraState, ThreadedCapture
from hapticsight.vision.cli import main
from hapticsight.vision.config import (
    VisionConfig,
    calibration_to_toml,
    load_vision_config,
    vision_config_from_dict,
)
from hapticsight.vision.geometry import CameraCalibration
from hapticsight.vision.recording import VideoRecorder, iter_video, merge_streams, replay
from hapticsight.vision.synthetic import (
    DEMO_SCENE,
    MarkerDetector,
    SyntheticObject,
    SyntheticScene,
    timestamps_path,
    write_synthetic_video,
)
from tests.vision_helpers import frame

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def video(tmp_path_factory: pytest.TempPathFactory) -> Path:
    path = tmp_path_factory.mktemp("vid") / "demo.avi"
    write_synthetic_video(path, seconds=4.0)
    return path


def strip_perf(lines: list[str]) -> list[dict]:  # type: ignore[type-arg]
    out = []
    for line in lines:
        rec = json.loads(line)
        rec.pop("perf")
        out.append(rec)
    return out


def run_replay(video: Path, tmp: Path, name: str, **kw: object) -> tuple[list[dict], object]:  # type: ignore[type-arg]
    cfg = VisionConfig()
    out = tmp / f"{name}.jsonl"
    with open(out, "w") as fh:
        summary = replay(
            {"CAM_FRONT": video},
            {"CAM_FRONT": cfg.camera("CAM_FRONT")},
            MarkerDetector(),
            cfg.filter,
            cfg.tracker,
            export=fh,
            **kw,  # type: ignore[arg-type]
        )
    return strip_perf(out.read_text().splitlines()), summary


def test_replay_is_deterministic(video: Path, tmp_path: Path) -> None:
    a, sa = run_replay(video, tmp_path, "a")
    b, _ = run_replay(video, tmp_path, "b")
    assert a == b
    assert sa.frames == 60  # type: ignore[attr-defined]
    rec = a[30]
    assert {"camera_id", "frame", "t", "detections", "tracks", "observations"} <= rec.keys()
    assert all(o["range_status"] == "unknown" for r in a for o in r["observations"])


def test_replay_uses_sidecar_timestamps(video: Path) -> None:
    frames = list(iter_video(video, "CAM_FRONT"))
    assert [f.timestamp for f in frames[:3]] == pytest.approx([0.0, 1 / 15, 2 / 15], abs=1e-5)
    assert timestamps_path(video).exists()


def test_replay_tracks_match_synthetic_ground_truth(video: Path, tmp_path: Path) -> None:
    recs, _ = run_replay(video, tmp_path, "gt")
    geo = VisionConfig().camera("CAM_FRONT").geometry()
    rec = recs[45]  # t = 3.0 s: person, chair, car visible (bicycle too until 6 s)
    t = rec["t"]
    truth = {label: box for label, box in DEMO_SCENE.ground_truth(t)}
    confirmed = {tr["label"]: tr for tr in rec["tracks"] if tr["state"] == "confirmed"}
    assert set(confirmed) == set(truth)
    for label, tr in confirmed.items():
        expected = geo.body_bearing_deg(truth[label].center_x, DEMO_SCENE.width)
        assert angular_distance(tr["bearing_deg"], expected) < 1.0
    # identity stability: the walking person keeps one id for the whole clip
    person_ids = {tr["track_id"] for r in recs for tr in r["tracks"] if tr["label"] == "person"}
    assert len(person_ids) == 1


def test_replay_with_simulated_ultrasonic_drives_haptics(video: Path, tmp_path: Path) -> None:
    sim = ManualUltrasonic.from_spec("S0=80")
    recs, summary = run_replay(
        video, tmp_path, "us", pipeline=HapticSightPipeline(), ultrasonic=sim.readings
    )
    assert all(r["ultrasonic_source"] == "simulated" for r in recs)
    assert any(r["haptics"][0] == "2" for r in recs)  # M0 WARNING from fusion/echo
    assert summary.pattern_strings  # type: ignore[attr-defined]


def test_two_camera_replay_interleaves_by_time(tmp_path: Path) -> None:
    a = tmp_path / "front.avi"
    b = tmp_path / "rear.avi"
    write_synthetic_video(a, seconds=1.0)
    write_synthetic_video(b, seconds=1.0)
    merged = list(merge_streams({"CAM_FRONT": iter_video(a, "CAM_FRONT"),
                                 "CAM_REAR": iter_video(b, "CAM_REAR")}))  # fmt: skip
    times = [f.timestamp for f in merged]
    assert times == sorted(times)
    assert {f.camera_id for f in merged} == {"CAM_FRONT", "CAM_REAR"}


def test_annotated_video_and_recorder(video: Path, tmp_path: Path) -> None:
    run_replay(video, tmp_path, "ann", annotate_dir=tmp_path / "ann", max_frames=10)
    assert (tmp_path / "ann" / "CAM_FRONT_annotated.mp4").stat().st_size > 1000
    rec = VideoRecorder(tmp_path / "rec.mp4", 15.0, (64, 48))
    for i in range(1, 11):
        rec.add(frame(t=1.0 + i / 15, seq=i, w=64, h=48))
    rec.close()
    sidecar = timestamps_path(tmp_path / "rec.mp4").read_text().splitlines()
    stamps = [json.loads(x)["t"] for x in sidecar]
    assert len(stamps) == 10 and stamps[0] == 0.0
    assert rec.written == 10 and rec.dropped == 0


def test_synthetic_scene_objects_appear_and_disappear() -> None:
    scene = SyntheticScene(objects=(SyntheticObject("car", 10, 10, 30, 20, visible_from=1.0),))
    assert MarkerDetector().detect(scene.render(0.5)) == []
    (d,) = MarkerDetector().detect(scene.render(1.5))
    assert d.label == "car" and d.box.width == pytest.approx(30, abs=1)


# ------------------------------------------------------------ manual ultrasonic


def test_manual_ultrasonic_modes_rate_and_approach() -> None:
    sim = ManualUltrasonic.from_spec("S0=100,S1=X,S2=D")
    first = {r.sensor_id: r for r in sim.readings(0.0)}
    assert first["S0"].distance_cm == 100 and "S2" not in first
    assert sim.readings(0.05) == []  # 10 Hz
    sim.sensors["S0"].speed_cm_s = 50.0
    sim.readings(0.1)
    later = {r.sensor_id: r for r in sim.readings(1.1)}
    # approach runs from t=0.05 (last update before speed was set) to 1.1 s
    assert later["S0"].distance_cm == round(100 - 50 * (1.1 - 0.05))
    sim.set("S0", "D")
    assert sim.sensors["S0"].mode is ManualMode.DROPOUT


# ------------------------------------------------------------ config


def test_vision_config_files_and_overrides(tmp_path: Path) -> None:
    cfg = load_vision_config(REPO / "config" / "vision.example.toml")
    assert cfg.detector.model == "yolox_tiny"
    assert cfg.camera("CAM_REAR").mount_bearing_deg == 180.0
    local = tmp_path / "local.toml"
    local.write_text(
        '[cameras.CAM_REAR]\ndevice = "synthetic"\nwidth = 640\n'
        + calibration_to_toml("CAM_REAR", CameraCalibration(hfov_deg=71.5, calibrated=True))
        + '\n[filter]\nclasses = "all"\n'
    )
    merged = load_vision_config(REPO / "config" / "vision.example.toml", local)
    rear = merged.camera("CAM_REAR")
    assert rear.device == "synthetic" and rear.width == 640
    assert rear.calibration.hfov_deg == pytest.approx(71.5) and rear.calibration.calibrated
    assert merged.filter.classes is None
    with pytest.raises(KeyError):
        vision_config_from_dict({"cameras": {"CAM_FRONT": {"zoom": 2}}})
    with pytest.raises(KeyError):
        vision_config_from_dict({"cameras": {"CAM_SIDE": {}}})


# ------------------------------------------------------------ CLI


def test_cli_commands_without_camera_or_model(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)
    vid = tmp_path / "v.avi"
    assert main(["synth-video", str(vid), "--seconds", "2"]) == 0
    assert main(["models", "--model-dir", str(tmp_path)]) == 0
    assert "not installed" in capsys.readouterr().out
    export = tmp_path / "out.jsonl"
    assert main(["replay", "--front", str(vid), "--model", "synthetic",
                 "--ultrasonic", "S0=80", "--export", str(export)]) == 0  # fmt: skip
    summary = json.loads(capsys.readouterr().out)
    assert summary["frames"] == 30 and summary["ultrasonic"].startswith("SIMULATED")
    assert len(export.read_text().splitlines()) == 30
    assert main(["benchmark", "--model", "synthetic", "--frames", "10"]) == 0
    assert json.loads(capsys.readouterr().out)["frames"] == 10
    points = ["--point", "640:2", "--point", "1100:20", "--point", "200:-17"]
    args = ["calibrate", "--camera", "CAM_FRONT", "--image-width", "1280", *points, "--write"]
    assert main(args) == 0
    assert "[cameras.CAM_FRONT.calibration]" in (tmp_path / "config/cameras.local.toml").read_text()
    with pytest.raises(SystemExit):
        main(["replay", "--front", str(vid), "--model", "yolox_tiny"])  # model not installed here


def test_cli_record_from_synthetic_source(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "rec"
    assert main(["record", "--front", "synthetic", "--rear", "off", "--seconds", "1.0",
                 "--out", str(out)]) == 0  # fmt: skip
    text = capsys.readouterr().out
    assert "CAM_FRONT" in text and "local only" in text
    (session,) = out.iterdir()
    assert (session / "CAM_FRONT.mp4").exists()
    assert not (session / "CAM_REAR.mp4").exists()


# ------------------------------------------------------------ live app (headless)


def test_live_app_smoke(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pygame = pytest.importorskip("pygame")
    from hapticsight.vision.factory import SyntheticCapture
    from hapticsight.vision.live_app import LiveVisionApp
    from hapticsight.vision.processor import VisionSystem

    monkeypatch.chdir(tmp_path)
    cfg = VisionConfig()
    caps = [ThreadedCapture(c, opener=lambda _c: SyntheticCapture()) for c in cfg.cameras]
    system = VisionSystem(cfg, MarkerDetector(), captures=caps)
    app = LiveVisionApp(
        system,
        ultrasonic=ManualUltrasonic.from_spec("S0=90"),
        record_dir=tmp_path / "rec",
        calibration_path=tmp_path / "cal.toml",
    )
    system.start()
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and len(system.snapshot().results) < 2:
            app.step()
            time.sleep(0.02)
        app.step()
        app.draw()

        def key(k: int, mod: int = 0) -> None:
            app.handle_event(pygame.event.Event(pygame.KEYDOWN, key=k, mod=mod))
            app.step()
            app.draw()

        for k in (pygame.K_h, pygame.K_2, pygame.K_UP, pygame.K_e, pygame.K_a, pygame.K_s,
                  pygame.K_d, pygame.K_TAB, pygame.K_c, pygame.K_COMMA, pygame.K_EQUALS,
                  pygame.K_w, pygame.K_p, pygame.K_k):  # fmt: skip
            key(k)
        assert app.selected_sensor == "S1"
        assert "[cameras.CAM_REAR.calibration]" in (tmp_path / "cal.toml").read_text()
        key(pygame.K_f)
        assert system.snapshot().statuses["CAM_FRONT"].state is CameraState.DISABLED
        assert app.result is not None and len(app.result.frame.channels) == 8
        assert list((tmp_path / "recordings").glob("live-*.png"))
    finally:
        app.close()
    session = next((tmp_path / "rec").iterdir())
    log = [json.loads(x) for x in (session / "session.jsonl").read_text().splitlines()]
    assert log and all(e["ultrasonic_source"] == "simulated-manual" for e in log)
    assert all(e["ultrasonic_simulated"] is True for e in log)
