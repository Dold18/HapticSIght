"""Synthetic scenes for tests, CI, and demos without cameras or models.

:class:`SyntheticScene` renders solid-colour rectangles moving on a grey
background; :class:`MarkerDetector` finds them by colour. This is **not** an
object detector: it exists so capture → detection → tracking → geometry →
pipeline can be exercised deterministically with no webcam, no model download,
and no copyrighted footage.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from hapticsight.vision.coco import COCO_LABELS
from hapticsight.vision.types import BoundingBox, Image, RawDetection

# BGR colours -> COCO label the marker stands in for
MARKER_CLASSES: dict[tuple[int, int, int], str] = {
    (40, 40, 220): "person",  # red
    (220, 60, 40): "bicycle",  # blue
    (40, 200, 40): "chair",  # green
    (0, 200, 230): "car",  # yellow
}
BACKGROUND = 110


@dataclass(frozen=True, slots=True)
class SyntheticObject:
    label: str
    x: float  # left edge at t=0 (px)
    y: float
    w: float
    h: float
    vx: float = 0.0  # px/s
    vy: float = 0.0
    grow_per_s: float = 0.0  # image-plane growth (px/s on each side)
    visible_from: float = 0.0
    visible_until: float = float("inf")

    def box_at(self, t: float) -> BoundingBox | None:
        if not self.visible_from <= t < self.visible_until:
            return None
        g = self.grow_per_s * t
        return BoundingBox(
            self.x + self.vx * t - g,
            self.y + self.vy * t - g,
            self.x + self.vx * t + self.w + g,
            self.y + self.vy * t + self.h + g,
        )


def _color_for(label: str) -> tuple[int, int, int]:
    for color, name in MARKER_CLASSES.items():
        if name == label:
            return color
    raise ValueError(f"no synthetic marker colour for {label!r}")


@dataclass(frozen=True, slots=True)
class SyntheticScene:
    width: int = 640
    height: int = 360
    fps: float = 15.0
    objects: tuple[SyntheticObject, ...] = ()

    def render(self, t: float) -> Image:
        import cv2

        img = np.full((self.height, self.width, 3), BACKGROUND, dtype=np.uint8)
        for obj in self.objects:
            box = obj.box_at(t)
            if box is None:
                continue
            b = box.clipped(self.width, self.height)
            if b.width < 1 or b.height < 1:
                continue
            cv2.rectangle(
                img,
                (int(b.x1), int(b.y1)),
                (int(b.x2) - 1, int(b.y2) - 1),
                _color_for(obj.label),
                thickness=-1,
            )
        return img

    def ground_truth(self, t: float) -> list[tuple[str, BoundingBox]]:
        out = []
        for obj in self.objects:
            box = obj.box_at(t)
            if box is not None:
                out.append((obj.label, box.clipped(self.width, self.height)))
        return out


class MarkerDetector:
    """Finds synthetic colour markers. Deterministic; for tests and demos only."""

    name = "synthetic-markers"

    def __init__(self, tolerance: int = 30, min_area: int = 30) -> None:
        self.tolerance = tolerance
        self.min_area = min_area

    def detect(self, image: Image) -> list[RawDetection]:
        import cv2

        out: list[RawDetection] = []
        for color, label in MARKER_CLASSES.items():
            lo = np.clip(np.array(color) - self.tolerance, 0, 255).astype(np.uint8)
            hi = np.clip(np.array(color) + self.tolerance, 0, 255).astype(np.uint8)
            mask = cv2.inRange(image, lo, hi)
            n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for i in range(1, n):
                x, y, w, h, area = (int(v) for v in stats[i])
                if area < self.min_area:
                    continue
                out.append(
                    RawDetection(
                        COCO_LABELS.index(label),
                        label,
                        0.9,
                        BoundingBox(float(x), float(y), float(x + w), float(y + h)),
                    )
                )
        out.sort(key=lambda d: (d.box.x1, d.box.y1, d.label))
        return out


DEMO_SCENE = SyntheticScene(
    objects=(
        SyntheticObject("person", x=-60, y=120, w=50, h=150, vx=45),  # walks left -> right
        SyntheticObject("chair", x=420, y=220, w=70, h=80),
        SyntheticObject("car", x=520, y=140, w=90, h=60, grow_per_s=3.0, visible_from=2.0),
        SyntheticObject("bicycle", x=200, y=150, w=60, h=60, vx=-20, visible_until=6.0),
    )
)


def write_synthetic_video(
    path: str | Path,
    scene: SyntheticScene = DEMO_SCENE,
    seconds: float = 8.0,
    codec: str = "MJPG",
) -> int:
    """Render ``scene`` to a video file plus a ``.timestamps.jsonl`` sidecar."""
    import cv2

    path = Path(path)
    n = round(seconds * scene.fps)
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter.fourcc(*codec), scene.fps, (scene.width, scene.height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"cannot open video writer for {path}")
    with open(timestamps_path(path), "w") as ts:
        for i in range(n):
            t = i / scene.fps
            writer.write(scene.render(t))
            ts.write(json.dumps({"frame": i, "t": round(t, 6)}) + "\n")
    writer.release()
    return n


def timestamps_path(video: str | Path) -> Path:
    video = Path(video)
    return video.with_name(video.name + ".timestamps.jsonl")
