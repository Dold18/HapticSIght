"""Camera + ultrasonic association (fusion foundation).

The cameras give good *bearing* but no metric range; the ultrasonic sensors give
range but only a coarse sector bearing. This module pairs them with a simple,
explainable gating rule:

* Only **fresh, sufficiently confident** camera detections may be associated
  (``min_camera_confidence``, ``max_camera_age_s``). Low-confidence detections
  pass through unchanged as camera-only observations; they never borrow
  confidence from an echo.
* Only fresh echoes are used (``max_ultrasonic_age_s``).
* A camera detection is associated with an ultrasonic echo when the camera
  bearing lies within ``beam_half_angle + association_margin`` of that sensor's
  boresight. The margin accounts for wide objects whose body overlaps the beam
  even though their centre is outside it.
* The fused observation takes bearing from the camera and distance / closing
  speed from the echo. Its confidence is ``min(camera, ultrasonic)``: the joint
  claim "this object, at this bearing, at this range" is no more certain than
  its weakest part.
* An HC-SR04 reports only the *nearest* echo. If several camera detections
  share one echo we cannot tell which object produced it, so each gets the
  (conservative, i.e. nearest) range and is flagged ``shared_range``.
* The raw ultrasonic observation is suppressed only when an accepted camera
  bearing lies inside the beam itself **and** the fused observation is at least
  as trustworthy as the gate. Otherwise the echo is kept, so an unreliable
  association can never remove an independent ultrasonic proximity warning.
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
    max_camera_age_s: float = 0.4
    # Must be >= RiskConfig.min_confidence (validated by HapticSightConfig) so an
    # accepted fused observation is always actionable when its echo would be.
    min_camera_confidence: float = 0.5

    def __post_init__(self) -> None:
        if not 0.0 <= self.min_camera_confidence <= 1.0:
            raise ValueError("min_camera_confidence must be within [0, 1]")
        if self.max_ultrasonic_age_s <= 0 or self.max_camera_age_s <= 0:
            raise ValueError("fusion age limits must be positive")


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
        eligible = cam.confidence >= cfg.min_camera_confidence and (
            now is None or cam.age(now) <= cfg.max_camera_age_s
        )
        if not eligible:
            unmatched_camera.append(cam)
            continue
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
        confidence = min(cam.confidence, us.confidence)
        inside_beam = (
            angular_distance(cam.bearing_deg, us.bearing_deg) <= us.bearing_uncertainty_deg
        )
        if inside_beam and confidence >= cfg.min_camera_confidence:
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
                confidence=confidence,
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
                    "ultrasonic_confidence": us.confidence,
                },
            )
        )

    kept_us = [u for u in ultrasonic if u.id not in suppressed_us]
    return [*kept_us, *fused, *unmatched_camera]
