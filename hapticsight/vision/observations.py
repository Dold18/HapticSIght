"""Tracks -> shared :class:`Observation` (the only vision output the pipeline sees).

Invariants (tested):
  * ``source = CAMERA``, ``range_status = UNKNOWN``, ``distance_cm = None``,
    ``closing_speed_cm_s = None``: a monocular webcam does not measure range.
  * ``id`` is the stable track id (e.g. ``CAM_FRONT:T7``) and ``sensor_id`` the camera.
  * ``timestamp`` is the *capture* time of the last matched frame, so a track
    that coasts through missed detections ages honestly and goes stale.
  * Image-plane motion is exported in metadata only, clearly labelled.
"""

from __future__ import annotations

from collections.abc import Iterable

from hapticsight.core.models import Observation, RangeStatus, SourceType
from hapticsight.vision.tracker import TrackSnapshot, TrackState


def track_to_observation(track: TrackSnapshot) -> Observation:
    det = track.detection
    return Observation(
        id=track.track_id,
        source=SourceType.CAMERA,
        bearing_deg=det.bearing_deg,
        timestamp=track.last_seen,
        confidence=min(1.0, max(0.0, track.confidence)),
        range_status=RangeStatus.UNKNOWN,
        distance_cm=None,
        bearing_uncertainty_deg=det.bearing_uncertainty_deg,
        closing_speed_cm_s=None,
        sensor_id=track.camera_id,
        metadata={
            "label": track.label,
            "class_id": track.class_id,
            "box": det.box.as_list(),
            "frame_size": (det.frame_width, det.frame_height),
            "bearing_extent_deg": (det.bearing_left_deg, det.bearing_right_deg),
            "track_hits": track.hits,
            "track_age_s": track.age_s,
            # Image-plane quantities. NOT metric motion, NOT closing speed.
            "image_velocity_px_s": track.velocity_px_s,
            "image_scale_rate_per_s": track.scale_rate_per_s,
        },
    )


def observations_from_tracks(tracks: Iterable[TrackSnapshot]) -> list[Observation]:
    """Confirmed tracks only; tentative ones are not yet trusted."""
    return [track_to_observation(t) for t in tracks if t.state is TrackState.CONFIRMED]
