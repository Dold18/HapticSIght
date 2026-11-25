"""Local recording of camera streams and deterministic replay of videos.

Recordings stay on the local disk (``recordings/`` is git-ignored) and are
never uploaded anywhere. Each video gets a ``<video>.timestamps.jsonl`` sidecar
with per-frame capture times so replays reproduce the original timing.
"""

from __future__ import annotations

import heapq
import json
import queue
import threading
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import IO, Any

from hapticsight.core.models import UltrasonicReading
from hapticsight.pipeline import HapticSightPipeline
from hapticsight.vision.config import CameraConfig
from hapticsight.vision.detector import DetectionFilter, ObjectDetector
from hapticsight.vision.processor import CameraProcessor, FrameResult
from hapticsight.vision.synthetic import timestamps_path
from hapticsight.vision.tracker import TrackerConfig
from hapticsight.vision.types import CameraFrame, Image

# ---------------------------------------------------------------- recording


class VideoRecorder:
    """Writes frames on a background thread with a bounded queue (drops when full)."""

    def __init__(
        self, path: str | Path, fps: float, size: tuple[int, int], codec: str = "mp4v"
    ) -> None:
        import cv2

        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter.fourcc(*codec), fps, size)
        if not self._writer.isOpened():
            raise RuntimeError(f"cannot open video writer for {self.path}")
        self._ts: IO[str] = open(timestamps_path(self.path), "w")  # noqa: SIM115
        self._queue: queue.Queue[tuple[int, float, Image] | None] = queue.Queue(maxsize=60)
        self.dropped = 0
        self.written = 0
        self._t0: float | None = None
        self._thread = threading.Thread(target=self._run, name=f"rec-{self.path.name}", daemon=True)
        self._thread.start()

    def add(self, frame: CameraFrame) -> None:
        if self._t0 is None:
            self._t0 = frame.timestamp
        try:
            self._queue.put_nowait((frame.seq, frame.timestamp - self._t0, frame.image))
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        index = 0
        while True:
            item = self._queue.get()
            if item is None:
                break
            seq, t, image = item
            self._writer.write(image)
            self._ts.write(json.dumps({"frame": index, "t": round(t, 6), "seq": seq}) + "\n")
            index += 1
            self.written = index

    def close(self) -> None:
        self._queue.put(None)
        self._thread.join(10.0)
        self._writer.release()
        self._ts.close()


# ---------------------------------------------------------------- replay


def iter_video(path: str | Path, camera_id: str) -> Iterator[CameraFrame]:
    """Frames of a video with timestamps from its sidecar, else index / fps."""
    import cv2

    path = Path(path)
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise FileNotFoundError(f"cannot open video {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    times: list[float] = []
    sidecar = timestamps_path(path)
    if sidecar.exists():
        with open(sidecar) as fh:
            times = [float(json.loads(line)["t"]) for line in fh if line.strip()]
    i = 0
    try:
        while True:
            ok, image = cap.read()
            if not ok:
                break
            t = times[i] if i < len(times) else i / fps
            yield CameraFrame(camera_id, image, t, i + 1)  # type: ignore[arg-type]
            i += 1
    finally:
        cap.release()


def merge_streams(streams: Mapping[str, Iterator[CameraFrame]]) -> Iterator[CameraFrame]:
    """Interleave several camera streams in timestamp order (ties: camera id)."""
    heap: list[tuple[float, str, int, CameraFrame]] = []
    for cid, it in streams.items():
        first = next(it, None)
        if first is not None:
            heapq.heappush(heap, (first.timestamp, cid, first.seq, first))
    while heap:
        _, cid, _, frame = heapq.heappop(heap)
        yield frame
        nxt = next(streams[cid], None)
        if nxt is not None:
            heapq.heappush(heap, (nxt.timestamp, cid, nxt.seq, nxt))


@dataclass
class ReplaySummary:
    frames: int = 0
    detections: int = 0
    track_ids: set[str] = field(default_factory=set)
    inference_ms: list[float] = field(default_factory=list)
    pattern_strings: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        lat = sorted(self.inference_ms)
        return {
            "frames": self.frames,
            "detections": self.detections,
            "unique_tracks": len(self.track_ids),
            "inference_ms_mean": round(sum(lat) / len(lat), 2) if lat else None,
            "inference_ms_p95": round(lat[int(len(lat) * 0.95)], 2) if lat else None,
        }


def result_record(
    result: FrameResult, haptics: str | None, ultrasonic: str | None
) -> dict[str, Any]:
    """Deterministic JSON record for one processed frame (timing under 'perf')."""
    rec: dict[str, Any] = {
        "camera_id": result.camera_id,
        "frame": result.frame_seq,
        "t": round(result.capture_ts, 6),
        "detections": [d.to_dict() for d in result.detections],
        "tracks": [t.to_dict() for t in result.tracks],
        "observations": [
            {
                "id": o.id,
                "bearing_deg": round(o.bearing_deg, 2),
                "bearing_uncertainty_deg": round(o.bearing_uncertainty_deg, 2),
                "confidence": round(o.confidence, 4),
                "range_status": o.range_status.value,
            }
            for o in result.observations
        ],
        "perf": {
            "inference_ms": round(result.inference_ms, 3),
            "total_ms": round(result.total_ms, 3),
        },
    }
    if haptics is not None:
        rec["haptics"] = haptics
        rec["ultrasonic_source"] = ultrasonic
    return rec


def replay(
    videos: Mapping[str, str | Path],
    cameras: Mapping[str, CameraConfig],
    detector: ObjectDetector,
    detection_filter: DetectionFilter | None = None,
    tracker_config: TrackerConfig | None = None,
    pipeline: HapticSightPipeline | None = None,
    ultrasonic: Callable[[float], list[UltrasonicReading]] | None = None,
    export: IO[str] | None = None,
    annotate_dir: Path | None = None,
    max_frames: int | None = None,
    on_result: Callable[[CameraFrame, FrameResult], None] | None = None,
) -> ReplaySummary:
    """Process recorded video(s) frame by frame, in timestamp order.

    Deterministic for a deterministic detector: no wall clock is involved
    (``now`` = each frame's recorded timestamp) and no frames are dropped.
    """
    from hapticsight.vision.overlay import annotate

    processors = {
        cid: CameraProcessor(cameras[cid], detector, detection_filter, tracker_config)
        for cid in videos
    }
    streams = {cid: iter_video(path, cid) for cid, path in videos.items()}
    writers: dict[str, Any] = {}
    latest: dict[str, FrameResult] = {}
    summary = ReplaySummary()
    try:
        for frame in merge_streams(streams):
            if max_frames is not None and summary.frames >= max_frames:
                break
            result = processors[frame.camera_id].process(frame, now=frame.timestamp)
            latest[frame.camera_id] = result
            haptics = None
            us_label = None
            if pipeline is not None:
                readings = ultrasonic(frame.timestamp) if ultrasonic else []
                us_label = "simulated" if ultrasonic else "none"
                cams = [o for r in latest.values() for o in r.observations]
                out = pipeline.process(frame.timestamp, readings, cams)
                haptics = out.frame.pattern_string()
                summary.pattern_strings.append(haptics)
            summary.frames += 1
            summary.detections += len(result.detections)
            summary.track_ids.update(t.track_id for t in result.tracks)
            summary.inference_ms.append(result.inference_ms)
            if export is not None:
                export.write(json.dumps(result_record(result, haptics, us_label)) + "\n")
            if annotate_dir is not None:
                writers.setdefault(
                    frame.camera_id,
                    _annotation_writer(annotate_dir, frame),
                ).write(
                    annotate(
                        frame.image,
                        result,
                        processors[frame.camera_id].geometry,
                        header=f"{frame.camera_id} t={frame.timestamp:.2f}s "
                        f"haptics={haptics or '-'} (replay)",
                    )
                )
            if on_result is not None:
                on_result(frame, result)
    finally:
        for w in writers.values():
            w.release()
    return summary


def _annotation_writer(directory: Path, frame: CameraFrame) -> Any:
    import cv2

    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{frame.camera_id}_annotated.mp4"
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter.fourcc(*"mp4v"), 15.0, (frame.width, frame.height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"cannot write {path}")
    return writer
