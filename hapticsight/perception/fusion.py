"""Camera + ultrasonic association (fusion foundation).

The cameras give good *bearing* but no metric range; the ultrasonic sensors give
range but only a coarse sector bearing. This module pairs them with a simple,
explainable gating rule:

* A camera detection is associated with an ultrasonic echo when the camera
  bearing lies within ``beam_half_angle + association_margin`` of that sensor's
  boresight. The margin accounts for wide objects whose body overlaps the beam
  even though their centre is outside it.
* The fused observation takes bearing from the camera and distance / closing
  speed from the ultrasonic echo.
* An HC-SR04 reports only the *nearest* echo. If several camera detections
  share one echo we cannot tell which object produced it, so each gets the
  (conservative, i.e. nearest) range and is flagged ``shared_range``.
* The raw ultrasonic observation is suppressed only when the associated camera
  bearing lies inside the beam itself. Otherwise both are kept, because the
  echo may come from an object the camera does not see.
* Camera detections with no echo match remain bearing-only (range UNKNOWN).

Nothing here invents range for a camera-only object.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from hapticsight.core.geometry import angular_distance
from hapticsight.core.models import Observation, RangeStatus, SourceType


@dataclass(frozen=True, slots=True)
class FusionConfig:
    enabled: bool = True
    association_margin_deg: float = 10.0
    max_ultrasonic_age_s: float = 0.4


def fuse(
    ultrasonic: Sequence[Observation],
    camera: Sequence[Observation],
    config: FusionConfig | None = None,
    now: float | None = None,
) -> list[Observation]:
    """Return the combined observation list (fused + unmatched raw).

    When ``now`` is given, ultrasonic echoes older than ``max_ultrasonic_age_s``
    are not used for association: a stale range must not be attached to a fresh
    camera detection. Stale echoes are still passed through unchanged so the
    risk engine can report them as stale.
    """
    cfg = config or FusionConfig()
    if not cfg.enabled:
        return [*ultrasonic, *camera]

    ranged_us = [
        u
        for u in ultrasonic
        if u.has_range and (now is None or u.age(now) <= cfg.max_ultrasonic_age_s)
    ]
    matches: dict[str, list[Observation]] = {u.id: [] for u in ranged_us}
    unmatched_camera: list[Observation] = []
    best_for_cam: dict[str, Observation] = {}

    for cam in camera:
        candidates = [
            u
            for u in ranged_us
            if angular_distance(cam.bearing_deg, u.bearing_deg)
            <= u.bearing_uncertainty_deg + cfg.association_margin_deg
        ]
        if not candidates:
            unmatched_camera.append(cam)
            continue
        best = min(
            candidates,
            key=lambda u: (angular_distance(cam.bearing_deg, u.bearing_deg), u.distance_cm),
        )
        matches[best.id].append(cam)
        best_for_cam[cam.id] = best

    fused: list[Observation] = []
    suppressed_us: set[str] = set()
    for cam_id, us in best_for_cam.items():
        cam = next(c for c in camera if c.id == cam_id)
        sharers = len(matches[us.id])
        inside_beam = (
            angular_distance(cam.bearing_deg, us.bearing_deg) <= us.bearing_uncertainty_deg
        )
        if inside_beam:
            suppressed_us.add(us.id)
        fused.append(
            Observation(
                id=f"fused:{cam.id}",
                source=SourceType.FUSED,
                bearing_deg=cam.bearing_deg,
                bearing_uncertainty_deg=cam.bearing_uncertainty_deg,
                # Use the older timestamp so the fused result is never fresher
                # than its stalest input.
                timestamp=min(cam.timestamp, us.timestamp),
                confidence=max(cam.confidence, us.confidence),
                range_status=RangeStatus.VALID,
                distance_cm=us.distance_cm,
                distance_uncertainty_cm=us.distance_uncertainty_cm,
                closing_speed_cm_s=us.closing_speed_cm_s,
                sensor_id=us.sensor_id,
                metadata={
                    "camera_id": cam.id,
                    "ultrasonic_id": us.id,
                    "shared_range": sharers > 1,
                    "camera_confidence": cam.confidence,
                },
            )
        )

    kept_us = [u for u in ultrasonic if u.id not in suppressed_us]
    return [*kept_us, *fused, *unmatched_camera]
