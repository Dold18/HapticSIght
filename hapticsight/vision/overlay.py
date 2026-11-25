"""OpenCV drawing of detections/tracks/bearings (annotated videos, screenshots)."""

from __future__ import annotations

from hapticsight.core.format import format_bearing
from hapticsight.vision.geometry import CameraGeometry
from hapticsight.vision.tracker import TrackState
from hapticsight.vision.types import Image

CONFIRMED = (80, 220, 80)
TENTATIVE = (160, 160, 160)
DETECTION = (0, 200, 255)


def annotate(
    image: Image,
    result: object,
    geometry: CameraGeometry | None = None,
    header: str | None = None,
    bearing_grid_deg: float = 15.0,
) -> Image:
    """Return a copy of ``image`` with tracks, bearings, and an optional bearing grid."""
    import cv2

    from hapticsight.vision.processor import FrameResult

    assert isinstance(result, FrameResult)
    out = image.copy()
    h, w = out.shape[:2]
    if geometry is not None and bearing_grid_deg > 0:
        b = -90.0
        while b <= 90.0:
            u = geometry.column_for_body_bearing(geometry.mount_bearing_deg + b, w)
            if u is not None and 0 <= u < w:
                cv2.line(out, (int(u), 0), (int(u), 10), (200, 200, 200), 1)
                cv2.putText(
                    out,
                    format_bearing(geometry.mount_bearing_deg + b),
                    (int(u) + 2, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (220, 220, 220),
                    1,
                )
            b += bearing_grid_deg
    for tr in result.tracks:
        box = tr.detection.box
        color = CONFIRMED if tr.state is TrackState.CONFIRMED else TENTATIVE
        thickness = 2 if tr.updated else 1
        p1, p2 = (int(box.x1), int(box.y1)), (int(box.x2), int(box.y2))
        cv2.rectangle(out, p1, p2, color, thickness)
        tid = tr.track_id.split(":")[-1]
        label = (
            f"{tid} {tr.label} {tr.confidence:.2f} "
            f"{format_bearing(tr.detection.bearing_deg)}"
            f"±{tr.detection.bearing_uncertainty_deg:.0f}"
        )
        cv2.putText(
            out, label, (p1[0], max(12, p1[1] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1
        )
    if header:
        cv2.putText(out, header, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return out
