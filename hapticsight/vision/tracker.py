"""Lightweight per-camera multi-object tracker (IoU association with motion prediction).

Why not SORT/ByteTrack/DeepSORT? At the frame rates we run (≥ 10 Hz per camera)
objects move little between frames, so greedy IoU matching against a
constant-velocity-predicted box is adequate, deterministic, dependency-free,
and easy to explain. A centre-distance fallback handles small or fast boxes
whose IoU drops to zero. This can be upgraded later behind the same interface.

Lifecycle: TENTATIVE until ``min_hits`` matches → CONFIRMED → removed once
not matched for ``max_missed_s`` seconds. Only confirmed tracks become
observations, which suppresses single-frame false positives; one track = one
observation, which prevents duplicate warnings from repeated detections.

Image-plane motion (pixel velocity, box scale rate) is recorded **separately**
and is never converted to a metric closing speed.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import StrEnum

from hapticsight.vision.types import BoundingBox, Detection


class TrackState(StrEnum):
    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"


@dataclass(frozen=True, slots=True)
class TrackerConfig:
    iou_threshold: float = 0.25
    center_gate: float = 0.6  # fallback: centre distance / box diagonal
    min_hits: int = 2
    max_missed_s: float = 0.6  # persistence through brief missed detections
    duplicate_iou: float = 0.7  # suppress new tracks overlapping a matched one
    max_history: int = 30
    confidence_smoothing: float = 0.5  # EMA weight of the newest detection
    motion_smoothing: float = 0.5

    def __post_init__(self) -> None:
        if not 0 < self.iou_threshold < 1:
            raise ValueError("iou_threshold must be in (0, 1)")
        if self.min_hits < 1 or self.max_missed_s <= 0:
            raise ValueError("min_hits >= 1 and max_missed_s > 0 required")


@dataclass(frozen=True, slots=True)
class HistoryEntry:
    timestamp: float
    box: BoundingBox
    confidence: float
    bearing_deg: float


@dataclass
class Track:
    track_id: str
    camera_id: str
    class_id: int
    label: str
    detection: Detection  # most recent matched detection
    first_seen: float
    last_seen: float
    hits: int = 1
    confidence: float = 0.0
    state: TrackState = TrackState.TENTATIVE
    velocity_px_s: tuple[float, float] = (0.0, 0.0)  # image-plane centre velocity
    scale_rate_per_s: float = 0.0  # d ln(box height)/dt, image-plane only
    history: deque[HistoryEntry] = field(default_factory=deque)

    @property
    def box(self) -> BoundingBox:
        return self.detection.box

    def age_s(self) -> float:
        return self.last_seen - self.first_seen

    def missed_for(self, now: float) -> float:
        return max(0.0, now - self.last_seen)

    def predicted_box(self, t: float) -> BoundingBox:
        dt = max(0.0, t - self.last_seen)
        return self.box.translated(self.velocity_px_s[0] * dt, self.velocity_px_s[1] * dt)


@dataclass(frozen=True, slots=True)
class TrackSnapshot:
    """Immutable view of a track for display, export, and observation building."""

    track_id: str
    camera_id: str
    class_id: int
    label: str
    state: TrackState
    detection: Detection
    first_seen: float
    last_seen: float
    hits: int
    confidence: float
    velocity_px_s: tuple[float, float]
    scale_rate_per_s: float
    updated: bool  # matched in the most recent frame

    @property
    def age_s(self) -> float:
        return self.last_seen - self.first_seen

    def to_dict(self) -> dict[str, object]:
        return {
            "track_id": self.track_id,
            "label": self.label,
            "state": self.state.value,
            "confidence": round(self.confidence, 4),
            "hits": self.hits,
            "age_s": round(self.age_s, 3),
            "updated": self.updated,
            "bearing_deg": round(self.detection.bearing_deg, 2),
            "box": self.detection.box.as_list(),
            "image_velocity_px_s": [round(v, 1) for v in self.velocity_px_s],
            "image_scale_rate_per_s": round(self.scale_rate_per_s, 3),
        }


def _center_distance_ratio(a: BoundingBox, b: BoundingBox) -> float:
    diag = math.hypot(max(a.width, b.width), max(a.height, b.height))
    return math.hypot(a.center_x - b.center_x, a.center_y - b.center_y) / max(diag, 1e-6)


class IouTracker:
    def __init__(self, camera_id: str, config: TrackerConfig | None = None) -> None:
        self.camera_id = camera_id
        self.config = config or TrackerConfig()
        self.tracks: list[Track] = []
        self._next_id = 1

    def reset(self) -> None:
        self.tracks.clear()
        self._next_id = 1

    def update(self, detections: list[Detection], timestamp: float) -> list[TrackSnapshot]:
        cfg = self.config
        # 1. expire tracks that have been missing too long
        self.tracks = [t for t in self.tracks if timestamp - t.last_seen <= cfg.max_missed_s]

        # 2. candidate pairs (same class), scored by IoU with the predicted box
        pairs: list[tuple[float, int, int]] = []
        for ti, tr in enumerate(self.tracks):
            pred = tr.predicted_box(timestamp)
            for di, det in enumerate(detections):
                if det.class_id != tr.class_id:
                    continue
                iou = pred.iou(det.box)
                if iou >= cfg.iou_threshold:
                    pairs.append((iou, ti, di))
                else:
                    ratio = _center_distance_ratio(pred, det.box)
                    if ratio <= cfg.center_gate:
                        # ranks below every IoU match, closer centres first
                        pairs.append((-ratio, ti, di))
        pairs.sort(key=lambda p: (-p[0], p[1], p[2]))

        matched_t: set[int] = set()
        matched_d: set[int] = set()
        for _, ti, di in pairs:
            if ti in matched_t or di in matched_d:
                continue
            matched_t.add(ti)
            matched_d.add(di)
            self._update_track(self.tracks[ti], detections[di], timestamp)

        # 3. new tracks for unmatched detections (unless duplicating a matched track)
        matched_boxes = [self.tracks[ti].box for ti in matched_t]
        for di, det in enumerate(detections):
            if di in matched_d:
                continue
            if any(det.box.iou(b) >= cfg.duplicate_iou for b in matched_boxes):
                continue
            self.tracks.append(self._new_track(det, timestamp))
            matched_boxes.append(det.box)

        return [self._snapshot(t, timestamp) for t in self.tracks]

    def _new_track(self, det: Detection, ts: float) -> Track:
        tid = f"{self.camera_id}:T{self._next_id}"
        self._next_id += 1
        tr = Track(
            track_id=tid,
            camera_id=self.camera_id,
            class_id=det.class_id,
            label=det.label,
            detection=det,
            first_seen=ts,
            last_seen=ts,
            confidence=det.confidence,
            state=TrackState.CONFIRMED if self.config.min_hits <= 1 else TrackState.TENTATIVE,
        )
        tr.history.append(HistoryEntry(ts, det.box, det.confidence, det.bearing_deg))
        return tr

    def _update_track(self, tr: Track, det: Detection, ts: float) -> None:
        cfg = self.config
        dt = ts - tr.last_seen
        if dt > 0:
            vx = (det.box.center_x - tr.box.center_x) / dt
            vy = (det.box.center_y - tr.box.center_y) / dt
            a = cfg.motion_smoothing
            tr.velocity_px_s = (
                a * vx + (1 - a) * tr.velocity_px_s[0],
                a * vy + (1 - a) * tr.velocity_px_s[1],
            )
            if tr.box.height > 0 and det.box.height > 0:
                rate = math.log(det.box.height / tr.box.height) / dt
                tr.scale_rate_per_s = a * rate + (1 - a) * tr.scale_rate_per_s
        w = cfg.confidence_smoothing
        tr.confidence = w * det.confidence + (1 - w) * tr.confidence
        tr.detection = det
        tr.last_seen = ts
        tr.hits += 1
        if tr.hits >= cfg.min_hits:
            tr.state = TrackState.CONFIRMED
        tr.history.append(HistoryEntry(ts, det.box, det.confidence, det.bearing_deg))
        while len(tr.history) > cfg.max_history:
            tr.history.popleft()

    def _snapshot(self, tr: Track, ts: float) -> TrackSnapshot:
        return TrackSnapshot(
            track_id=tr.track_id,
            camera_id=tr.camera_id,
            class_id=tr.class_id,
            label=tr.label,
            state=tr.state,
            detection=tr.detection,
            first_seen=tr.first_seen,
            last_seen=tr.last_seen,
            hits=tr.hits,
            confidence=tr.confidence,
            velocity_px_s=tr.velocity_px_s,
            scale_rate_per_s=tr.scale_rate_per_s,
            updated=tr.last_seen == ts,
        )

    def history(self, track_id: str) -> list[HistoryEntry]:
        for t in self.tracks:
            if t.track_id == track_id:
                return list(t.history)
        return []
