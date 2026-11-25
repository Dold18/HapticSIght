"""Concurrent, bounded-latency camera capture.

Each :class:`ThreadedCapture` owns one background thread that reads frames as
fast as the device delivers them and keeps **only the newest** frame in a
:class:`LatestFrameSlot`. Consumers never see a queue, so latency cannot
accumulate: if inference is slower than capture, old frames are dropped.

Every frame is stamped with host monotonic time when ``read()`` returns.

Failure handling (all reported through :class:`CameraStatus`):
  * device cannot be opened (missing device or macOS camera permission)
    -> UNAVAILABLE, retried every ``reconnect_interval_s``
  * ``max_failed_reads`` consecutive failed reads -> DISCONNECTED, release and reopen
  * no new frame for ``stall_timeout_s`` -> STALLED (reported; a hung driver
    call cannot be interrupted, but its frames are never used once stale)
  * video file ended (no loop) -> FINISHED
  * ``enabled = false`` -> DISABLED, the device is never opened

``stop()`` joins the thread and releases the device.
"""

from __future__ import annotations

import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import StrEnum
from typing import Any, Protocol

from hapticsight.core.clock import Clock, MonotonicClock
from hapticsight.vision.config import CameraConfig
from hapticsight.vision.types import CameraFrame


class CameraState(StrEnum):
    DISABLED = "disabled"
    STARTING = "starting"
    RUNNING = "running"
    STALLED = "stalled"
    DISCONNECTED = "disconnected"
    UNAVAILABLE = "unavailable"  # could not open: missing device or permission denied
    FINISHED = "finished"
    STOPPED = "stopped"


@dataclass(frozen=True, slots=True)
class CameraStatus:
    camera_id: str
    state: CameraState
    detail: str
    frames: int
    fps: float
    last_frame_age_s: float | None
    width: int | None
    height: int | None
    reconnects: int

    @property
    def healthy(self) -> bool:
        return self.state is CameraState.RUNNING


class LatestFrameSlot:
    """Single-slot, thread-safe holder for the newest frame (no queue)."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frame: CameraFrame | None = None

    def put(self, frame: CameraFrame) -> None:
        with self._cond:
            self._frame = frame
            self._cond.notify_all()

    def latest(self) -> CameraFrame | None:
        with self._cond:
            return self._frame

    def wait_newer(self, seq: int, timeout: float) -> CameraFrame | None:
        """Newest frame with ``frame.seq > seq``, waiting up to ``timeout`` seconds."""
        with self._cond:
            if self._frame is not None and self._frame.seq > seq:
                return self._frame
            self._cond.wait(timeout)
            if self._frame is not None and self._frame.seq > seq:
                return self._frame
            return None


class VideoCaptureLike(Protocol):
    def isOpened(self) -> bool: ...
    def read(self) -> tuple[bool, Any]: ...
    def set(self, prop: int, value: float) -> bool: ...
    def get(self, prop: int) -> float: ...
    def release(self) -> None: ...


CaptureOpener = Callable[[CameraConfig], VideoCaptureLike]


def _backend_flag(name: str) -> int:
    import cv2

    flags = {
        "auto": cv2.CAP_ANY,
        "avfoundation": cv2.CAP_AVFOUNDATION,
        "v4l2": cv2.CAP_V4L2,
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
        "ffmpeg": cv2.CAP_FFMPEG,
    }
    return int(flags[name])


def default_backend_for_device(config: CameraConfig) -> str:
    if config.backend != "auto":
        return config.backend
    if isinstance(config.device, int) and sys.platform == "darwin":
        return "avfoundation"
    return "auto"


def opencv_opener(config: CameraConfig) -> VideoCaptureLike:
    import cv2

    device = config.device if config.device is not None else 0
    backend = _backend_flag(default_backend_for_device(config))
    cap = cv2.VideoCapture(device, backend)
    if cap.isOpened() and isinstance(device, int):
        if config.width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
        if config.height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
        if config.fps:
            cap.set(cv2.CAP_PROP_FPS, config.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # honoured by some backends only
    return cap


def is_file_source(config: CameraConfig) -> bool:
    return (
        isinstance(config.device, str)
        and not config.device.isdigit()
        and "://" not in (config.device)
    )


class ThreadedCapture:
    """Background capture for one camera or video file."""

    def __init__(
        self,
        config: CameraConfig,
        opener: CaptureOpener = opencv_opener,
        clock: Clock | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.camera_id = config.camera_id
        self.slot = LatestFrameSlot()
        self._opener = opener
        self._clock = clock or MonotonicClock()
        self._sleep = sleep
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._state = CameraState.DISABLED if not config.enabled else CameraState.STOPPED
        self._detail = "disabled in configuration" if not config.enabled else "not started"
        self._frames = 0
        self._fps = 0.0
        self._last_ts: float | None = None
        self._size: tuple[int, int] | None = None
        self._reconnects = 0
        self._file = is_file_source(config)

    # ------------------------------------------------------------ control

    def start(self) -> None:
        if not self.config.enabled or self._thread is not None:
            return
        self._stop.clear()
        self._set_state(CameraState.STARTING, "opening device")
        self._thread = threading.Thread(
            target=self._run, name=f"capture-{self.camera_id}", daemon=True
        )
        self._thread.start()

    def set_enabled(self, enabled: bool) -> None:
        """Enable/disable at runtime (disabling releases the device)."""
        if enabled:
            self.config = replace(self.config, enabled=True)
            self.start()
        else:
            self.stop()
            self.config = replace(self.config, enabled=False)
            self._set_state(CameraState.DISABLED, "disabled at runtime")

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            self._thread = None
        if self.config.enabled and self._state is not CameraState.FINISHED:
            self._set_state(CameraState.STOPPED, "stopped")

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------ status

    def _set_state(self, state: CameraState, detail: str) -> None:
        with self._lock:
            self._state, self._detail = state, detail

    def status(self, now: float | None = None) -> CameraStatus:
        now = self._clock.now() if now is None else now
        with self._lock:
            state, detail = self._state, self._detail
            age = None if self._last_ts is None else max(0.0, now - self._last_ts)
            if (
                state is CameraState.RUNNING
                and age is not None
                and age > self.config.stall_timeout_s
            ):
                state, detail = CameraState.STALLED, f"no frame for {age:.1f}s"
            return CameraStatus(
                self.camera_id,
                state,
                detail,
                self._frames,
                self._fps,
                age,
                self._size[0] if self._size else None,
                self._size[1] if self._size else None,
                self._reconnects,
            )

    # ------------------------------------------------------------ thread

    def _wait(self, seconds: float) -> bool:
        """Sleep that wakes early on stop(); returns True if stopping."""
        return self._stop.wait(seconds)

    def _run(self) -> None:
        cfg = self.config
        cap: VideoCaptureLike | None = None
        seq = 0
        try:
            while not self._stop.is_set():
                if cap is None:
                    try:
                        cap = self._opener(cfg)
                    except Exception as exc:  # pragma: no cover - driver specific
                        cap = None
                        self._set_state(CameraState.UNAVAILABLE, f"open failed: {exc}")
                    if cap is None or not cap.isOpened():
                        if cap is not None:
                            cap.release()
                        cap = None
                        self._set_state(
                            CameraState.UNAVAILABLE,
                            f"cannot open {cfg.device!r} (device missing, in use, or camera "
                            "permission not granted)",
                        )
                        if self._wait(cfg.reconnect_interval_s):
                            break
                        continue
                    self._set_state(CameraState.RUNNING, "capturing")
                    fails = 0
                    period = 1.0 / cfg.fps if (self._file and cfg.fps) else None
                    if self._file and period is None:
                        try:
                            import cv2

                            file_fps = cap.get(cv2.CAP_PROP_FPS)
                            period = 1.0 / file_fps if file_fps and file_fps > 0 else 1 / 30
                        except Exception:  # pragma: no cover
                            period = 1 / 30
                    next_due = self._clock.now()

                if self._file and period:
                    # Pace file playback like a live camera (frames are dropped
                    # downstream if processing is slower).
                    delay = next_due - self._clock.now()
                    if delay > 0 and self._wait(delay):
                        break
                    next_due += period

                ok, image = cap.read()
                now = self._clock.now()
                if not ok or image is None:
                    fails += 1
                    if self._file:
                        cap.release()
                        cap = None
                        if cfg.loop_video:
                            continue
                        self._set_state(CameraState.FINISHED, "end of video")
                        break
                    if fails >= cfg.max_failed_reads:
                        cap.release()
                        cap = None
                        self._reconnects += 1
                        self._set_state(CameraState.DISCONNECTED, "read failures; reconnecting")
                        if self._wait(cfg.reconnect_interval_s):
                            break
                    else:
                        self._wait(0.01)
                    continue
                fails = 0
                seq += 1
                frame = CameraFrame(self.camera_id, image, now, seq)
                with self._lock:
                    if self._last_ts is not None and now > self._last_ts:
                        inst = 1.0 / (now - self._last_ts)
                        self._fps = inst if self._fps == 0 else 0.9 * self._fps + 0.1 * inst
                    self._last_ts = now
                    self._frames += 1
                    self._size = (frame.width, frame.height)
                    if self._state is not CameraState.RUNNING:
                        self._state, self._detail = CameraState.RUNNING, "capturing"
                self.slot.put(frame)
        finally:
            if cap is not None:
                cap.release()
