from __future__ import annotations

import time
from pathlib import Path

import pytest

from hapticsight.vision.camera import CameraState, LatestFrameSlot, ThreadedCapture
from hapticsight.vision.config import CameraConfig
from hapticsight.vision.discovery import ProbeResult, diagnose
from hapticsight.vision.synthetic import write_synthetic_video
from tests.vision_helpers import FakeCapture, OpenerLog, frame, wait_until


def cfg(**kw: object) -> CameraConfig:
    base: dict[str, object] = dict(
        camera_id="CAM_FRONT",
        mount_bearing_deg=0.0,
        device=0,
        reconnect_interval_s=0.05,
        stall_timeout_s=0.2,
        max_failed_reads=3,
    )
    base.update(kw)
    return CameraConfig(**base)  # type: ignore[arg-type]


def test_latest_slot_keeps_only_newest() -> None:
    slot = LatestFrameSlot()
    for i in range(1, 6):
        slot.put(frame(seq=i))
    assert slot.latest().seq == 5  # type: ignore[union-attr]
    assert slot.wait_newer(5, 0.01) is None
    assert slot.wait_newer(2, 0.01).seq == 5  # type: ignore[union-attr]


def test_frames_flow_with_monotonic_timestamps_and_no_queue() -> None:
    cap = ThreadedCapture(cfg(), opener=OpenerLog(FakeCapture(fps=200)))
    cap.start()
    try:
        assert wait_until(lambda: (cap.slot.latest() or frame(seq=0)).seq >= 20)
        f1 = cap.slot.latest()
        time.sleep(0.05)  # slow consumer: frames in between are simply skipped
        f2 = cap.slot.latest()
        assert f1 is not None and f2 is not None
        assert f2.seq - f1.seq > 1
        assert f2.timestamp > f1.timestamp
        st = cap.status()
        assert st.state is CameraState.RUNNING and st.fps > 0 and st.width == 64
    finally:
        cap.stop()
    assert cap.status().state is CameraState.STOPPED


def test_unavailable_device_retries_without_blocking_stop() -> None:
    opener = OpenerLog(FakeCapture(opened=False))
    cap = ThreadedCapture(cfg(), opener=opener)
    cap.start()
    try:
        assert wait_until(lambda: opener.calls >= 3)
        st = cap.status()
        assert st.state is CameraState.UNAVAILABLE
        assert "permission" in st.detail
    finally:
        t0 = time.monotonic()
        cap.stop()
        assert time.monotonic() - t0 < 1.0


def test_disconnect_then_reconnect() -> None:
    first = FakeCapture(fail_reads_after=5)
    second = FakeCapture()
    opener = OpenerLog(first, second)
    cap = ThreadedCapture(cfg(), opener=opener)
    cap.start()
    try:
        assert wait_until(lambda: cap.status().reconnects >= 1)
        assert first.released
        assert wait_until(lambda: cap.status().state is CameraState.RUNNING and opener.calls >= 2)
        assert wait_until(lambda: (cap.slot.latest() or frame(seq=0)).seq > 5)
    finally:
        cap.stop()
    assert second.released


def test_stalled_feed_is_reported() -> None:
    stuck = FakeCapture(block_after=3)
    cap = ThreadedCapture(cfg(stall_timeout_s=0.1), opener=OpenerLog(stuck))
    cap.start()
    try:
        assert wait_until(lambda: cap.status().state is CameraState.STALLED)
        assert cap.status().last_frame_age_s > 0.1  # type: ignore[operator]
    finally:
        cap.stop()


def test_disabled_camera_is_never_opened_and_can_be_toggled() -> None:
    opener = OpenerLog(FakeCapture())
    cap = ThreadedCapture(cfg(enabled=False), opener=opener)
    cap.start()
    time.sleep(0.05)
    assert opener.calls == 0
    assert cap.status().state is CameraState.DISABLED
    cap.set_enabled(True)
    try:
        assert wait_until(lambda: cap.status().state is CameraState.RUNNING)
    finally:
        cap.set_enabled(False)
    assert cap.status().state is CameraState.DISABLED
    assert opener.issued[-1].released


def test_video_file_plays_paced_and_finishes(tmp_path: Path) -> None:
    video = tmp_path / "clip.avi"
    write_synthetic_video(video, seconds=1.0)  # 15 frames at 15 fps
    cap = ThreadedCapture(cfg(device=str(video), loop_video=False))
    cap.start()
    try:
        assert wait_until(lambda: cap.status().state is CameraState.FINISHED, timeout=5.0)
        assert cap.status().frames == 15
    finally:
        cap.stop()
    assert cap.status().state is CameraState.FINISHED


def test_diagnose_flags_probable_permission_problem() -> None:
    names = [{"name": "FaceTime HD Camera", "model_id": "", "unique_id": ""}]
    hints = diagnose([ProbeResult(0, "avfoundation", opened=False)], names)
    assert any("permission" in h for h in hints)
    hints2 = diagnose([ProbeResult(0, "avfoundation", opened=True, frame_ok=False)], [])
    assert any("no frames" in h for h in hints2)


@pytest.mark.parametrize("value", [-1.0])
def test_camera_config_is_plain_data(value: float) -> None:
    c = cfg(fps=None, width=None)
    assert c.fps is None and c.width is None  # nothing hard-coded: driver default
