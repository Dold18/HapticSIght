"""Per-camera processing and the concurrent dual-camera vision system.

Threading model (chosen for the 8 GB M1):

* one capture thread per camera (:class:`ThreadedCapture`), newest-frame only;
* one inference worker thread that serves both cameras round-robin with a
  single detector instance (one model in memory, no contention between two
  inference sessions), rate-limited per camera;
* the caller (UI / runtime loop) reads immutable snapshots and never blocks on
  capture or inference.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

from hapticsight.core.clock import Clock, MonotonicClock
from hapticsight.core.geometry import normalize_bearing
from hapticsight.core.models import Observation
from hapticsight.vision.camera import CameraStatus, ThreadedCapture
from hapticsight.vision.config import CameraConfig, VisionConfig
from hapticsight.vision.detector import DetectionFilter, ObjectDetector
from hapticsight.vision.geometry import CameraGeometry
from hapticsight.vision.observations import observations_from_tracks
from hapticsight.vision.tracker import IouTracker, TrackerConfig, TrackSnapshot
from hapticsight.vision.types import CameraFrame, Detection, RawDetection


@dataclass(frozen=True, slots=True)
class FrameResult:
    camera_id: str
    frame_seq: int
    capture_ts: float
    processed_ts: float
    frame_width: int
    frame_height: int
    inference_ms: float
    total_ms: float  # detect + filter + localise + track
    stale: bool  # frame too old: skipped, tracker only aged
    raw_count: int
    detections: tuple[Detection, ...]
    tracks: tuple[TrackSnapshot, ...]
    observations: tuple[Observation, ...]

    @property
    def latency_s(self) -> float:
        """Capture -> observations available."""
        return self.processed_ts - self.capture_ts


def localize(
    raw: Sequence[RawDetection], geometry: CameraGeometry, frame: CameraFrame
) -> list[Detection]:
    out = []
    for d in raw:
        est = geometry.estimate(d.box, frame.width)
        out.append(
            Detection(
                class_id=d.class_id,
                label=d.label,
                confidence=d.confidence,
                box=d.box,
                camera_id=frame.camera_id,
                timestamp=frame.timestamp,
                frame_width=frame.width,
                frame_height=frame.height,
                bearing_deg=normalize_bearing(est.bearing_deg),
                bearing_uncertainty_deg=est.uncertainty_deg,
                bearing_left_deg=est.left_deg,
                bearing_right_deg=est.right_deg,
            )
        )
    return out


class CameraProcessor:
    """Detector + filter + geometry + tracker for one camera (not thread-safe)."""

    def __init__(
        self,
        camera: CameraConfig,
        detector: ObjectDetector,
        detection_filter: DetectionFilter | None = None,
        tracker_config: TrackerConfig | None = None,
        clock: Clock | None = None,
    ) -> None:
        self.camera = camera
        self.geometry = camera.geometry()
        self.detector = detector
        self.filter = detection_filter or DetectionFilter()
        self.tracker = IouTracker(camera.camera_id, tracker_config)
        self.clock = clock or MonotonicClock()

    def set_geometry(self, geometry: CameraGeometry) -> None:
        self.geometry = geometry

    def process(self, frame: CameraFrame, now: float | None = None) -> FrameResult:
        start = self.clock.now() if now is None else now
        t0 = time.perf_counter()
        if start - frame.timestamp > self.camera.max_frame_age_s:
            tracks = self.tracker.update([], start)
            return FrameResult(
                frame.camera_id, frame.seq, frame.timestamp, start, frame.width, frame.height,
                0.0, 0.0, True, 0, (), tuple(tracks), tuple(observations_from_tracks(tracks)),
            )  # fmt: skip
        raw = self.detector.detect(frame.image)
        t1 = time.perf_counter()
        kept = self.filter.apply(raw, frame.width, frame.height)
        detections = localize(kept, self.geometry, frame)
        tracks = self.tracker.update(detections, frame.timestamp)
        observations = observations_from_tracks(tracks)
        t2 = time.perf_counter()
        processed = self.clock.now() if now is None else now + (t2 - t0)
        return FrameResult(
            camera_id=frame.camera_id,
            frame_seq=frame.seq,
            capture_ts=frame.timestamp,
            processed_ts=processed,
            frame_width=frame.width,
            frame_height=frame.height,
            inference_ms=(t1 - t0) * 1000,
            total_ms=(t2 - t0) * 1000,
            stale=False,
            raw_count=len(raw),
            detections=tuple(detections),
            tracks=tuple(tracks),
            observations=tuple(observations),
        )


@dataclass
class _PerfCounter:
    processed: int = 0
    stale: int = 0
    fps: float = 0.0
    last_ts: float | None = None
    inference_ms: float = 0.0
    latency_ms: float = 0.0

    def update(self, result: FrameResult) -> None:
        a = 0.1
        if result.stale:
            self.stale += 1
            return
        self.processed += 1
        if self.last_ts is not None and result.processed_ts > self.last_ts:
            inst = 1.0 / (result.processed_ts - self.last_ts)
            self.fps = inst if self.fps == 0 else (1 - a) * self.fps + a * inst
        self.last_ts = result.processed_ts
        lat = result.latency_s * 1000
        self.inference_ms = (
            result.inference_ms
            if self.processed == 1
            else (1 - a) * self.inference_ms + a * result.inference_ms
        )
        self.latency_ms = lat if self.processed == 1 else (1 - a) * self.latency_ms + a * lat


@dataclass(frozen=True, slots=True)
class CameraPerf:
    processed_frames: int
    stale_frames: int
    inference_fps: float
    inference_ms: float
    latency_ms: float


@dataclass(frozen=True, slots=True)
class VisionSnapshot:
    timestamp: float
    statuses: Mapping[str, CameraStatus]
    results: Mapping[str, FrameResult]
    frames: Mapping[str, CameraFrame]  # frame each result was computed on
    perf: Mapping[str, CameraPerf]
    worker_error: str | None = None


@dataclass
class VisionSystem:
    """Runs capture threads + one inference worker; thread-safe snapshot access."""

    config: VisionConfig
    detector: ObjectDetector
    captures: list[ThreadedCapture] = field(default_factory=list)
    clock: Clock = field(default_factory=MonotonicClock)

    def __post_init__(self) -> None:
        if not self.captures:
            from hapticsight.vision.factory import smart_opener

            self.captures = [
                ThreadedCapture(c, opener=smart_opener, clock=self.clock)
                for c in self.config.cameras
            ]
        self.processors = {
            cap.camera_id: CameraProcessor(
                cap.config, self.detector, self.config.filter, self.config.tracker, self.clock
            )
            for cap in self.captures
        }
        self._lock = threading.Lock()
        self._results: dict[str, FrameResult] = {}
        self._frames: dict[str, CameraFrame] = {}
        self._perf: dict[str, _PerfCounter] = {c.camera_id: _PerfCounter() for c in self.captures}
        self._stop = threading.Event()
        self._worker: threading.Thread | None = None
        self._error: str | None = None
        self.paused = False

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        for cap in self.captures:
            cap.start()
        self._stop.clear()
        self._worker = threading.Thread(target=self._run, name="vision-inference", daemon=True)
        self._worker.start()

    def stop(self) -> None:
        self._stop.set()
        if self._worker is not None:
            self._worker.join(5.0)
            self._worker = None
        for cap in self.captures:
            cap.stop()

    def __enter__(self) -> VisionSystem:
        self.start()
        return self

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # ------------------------------------------------------------ worker

    def _run(self) -> None:
        last_seq = {c.camera_id: 0 for c in self.captures}
        next_allowed = {c.camera_id: 0.0 for c in self.captures}
        min_period = 1.0 / max(self.config.max_inference_fps_per_camera, 0.1)
        while not self._stop.is_set():
            did_work = False
            for cap in self.captures:
                if self.paused or not cap.config.enabled:
                    continue
                now = self.clock.now()
                if now < next_allowed[cap.camera_id]:
                    continue
                frame = cap.slot.latest()
                if frame is None or frame.seq <= last_seq[cap.camera_id]:
                    continue
                last_seq[cap.camera_id] = frame.seq
                # Schedule from the previous slot (not from "now") so waiting for
                # the next captured frame does not erode the target rate.
                next_allowed[cap.camera_id] = max(next_allowed[cap.camera_id] + min_period, now)
                try:
                    result = self.processors[cap.camera_id].process(frame)
                except Exception as exc:  # keep the system alive; surface the error
                    self._error = f"{cap.camera_id}: {type(exc).__name__}: {exc}"
                    continue
                with self._lock:
                    self._results[cap.camera_id] = result
                    self._frames[cap.camera_id] = frame
                    self._perf[cap.camera_id].update(result)
                did_work = True
            if not did_work:
                self._stop.wait(0.003)

    # ------------------------------------------------------------ access

    def snapshot(self) -> VisionSnapshot:
        now = self.clock.now()
        statuses = {c.camera_id: c.status(now) for c in self.captures}
        with self._lock:
            perf = {
                k: CameraPerf(p.processed, p.stale, p.fps, p.inference_ms, p.latency_ms)
                for k, p in self._perf.items()
            }
            return VisionSnapshot(
                now, statuses, dict(self._results), dict(self._frames), perf, self._error
            )

    def camera_observations(self, now: float, max_age_s: float = 1.0) -> list[Observation]:
        """Latest confirmed-track observations from every camera.

        Results older than ``max_age_s`` are dropped entirely (e.g. camera
        disconnected); younger ones keep their capture timestamps so the risk
        engine can judge staleness itself.
        """
        with self._lock:
            results = list(self._results.values())
        out: list[Observation] = []
        for r in results:
            if now - r.processed_ts > max_age_s:
                continue
            out.extend(r.observations)
        return out
