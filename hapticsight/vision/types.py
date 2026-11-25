"""Value types for the vision layer (frames, boxes, detections).

These are vision-internal. The only thing the rest of HapticSight consumes is
the shared :class:`hapticsight.core.models.Observation`, produced by
:mod:`hapticsight.vision.observations`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np
    import numpy.typing as npt

    Image = npt.NDArray[np.uint8]
else:
    Image = Any


@dataclass(frozen=True, slots=True)
class CameraFrame:
    """One captured image. ``timestamp`` is host monotonic time at capture."""

    camera_id: str
    image: Image = field(repr=False)
    timestamp: float
    seq: int

    @property
    def height(self) -> int:
        return int(self.image.shape[0])

    @property
    def width(self) -> int:
        return int(self.image.shape[1])


@dataclass(frozen=True, slots=True)
class BoundingBox:
    """Axis-aligned box in pixel coordinates (x right, y down)."""

    x1: float
    y1: float
    x2: float
    y2: float

    def __post_init__(self) -> None:
        if self.x2 < self.x1 or self.y2 < self.y1:
            raise ValueError(f"invalid box {self}")

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return self.width * self.height

    @property
    def center_x(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y1 + self.y2) / 2.0

    def iou(self, other: BoundingBox) -> float:
        ix = max(0.0, min(self.x2, other.x2) - max(self.x1, other.x1))
        iy = max(0.0, min(self.y2, other.y2) - max(self.y1, other.y1))
        inter = ix * iy
        union = self.area + other.area - inter
        return inter / union if union > 0 else 0.0

    def clipped(self, width: float, height: float) -> BoundingBox:
        return BoundingBox(
            min(max(self.x1, 0.0), width),
            min(max(self.y1, 0.0), height),
            min(max(self.x2, 0.0), width),
            min(max(self.y2, 0.0), height),
        )

    def translated(self, dx: float, dy: float) -> BoundingBox:
        return BoundingBox(self.x1 + dx, self.y1 + dy, self.x2 + dx, self.y2 + dy)

    def as_list(self) -> list[float]:
        return [round(self.x1, 1), round(self.y1, 1), round(self.x2, 1), round(self.y2, 1)]


@dataclass(frozen=True, slots=True)
class RawDetection:
    """Detector output before localisation."""

    class_id: int
    label: str
    confidence: float
    box: BoundingBox


@dataclass(frozen=True, slots=True)
class Detection:
    """A localised detection: detector output plus camera context and bearing."""

    class_id: int
    label: str
    confidence: float
    box: BoundingBox
    camera_id: str
    timestamp: float
    frame_width: int
    frame_height: int
    bearing_deg: float  # body-relative, [0, 360)
    bearing_uncertainty_deg: float  # ± (1-sigma-style budget, see geometry.py)
    bearing_left_deg: float  # body bearing of the box's left edge
    bearing_right_deg: float  # body bearing of the box's right edge

    def to_dict(self) -> dict[str, Any]:
        return {
            "class_id": self.class_id,
            "label": self.label,
            "confidence": round(self.confidence, 4),
            "box": self.box.as_list(),
            "camera_id": self.camera_id,
            "timestamp": round(self.timestamp, 6),
            "frame": [self.frame_width, self.frame_height],
            "bearing_deg": round(self.bearing_deg, 2),
            "bearing_uncertainty_deg": round(self.bearing_uncertainty_deg, 2),
            "bearing_extent_deg": [
                round(self.bearing_left_deg, 2),
                round(self.bearing_right_deg, 2),
            ],
        }
