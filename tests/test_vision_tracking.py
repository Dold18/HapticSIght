from __future__ import annotations

import pytest

from hapticsight.core.models import RangeStatus, SourceType
from hapticsight.vision.observations import observations_from_tracks, track_to_observation
from hapticsight.vision.tracker import IouTracker, TrackerConfig, TrackState
from tests.vision_helpers import det

DT = 1 / 15


def run(tracker: IouTracker, frames: list[list], start: float = 0.0):  # type: ignore[no-untyped-def,type-arg]
    out = []
    for i, dets in enumerate(frames):
        out.append(tracker.update(dets, start + i * DT))
    return out


def ids(snaps) -> set[str]:  # type: ignore[no-untyped-def]
    return {s.track_id for s in snaps}


def test_stable_id_through_ordinary_motion() -> None:
    tr = IouTracker("CAM_FRONT")
    frames = [[det("person", 100 + 8 * i, 50, 160 + 8 * i, 250, t=i * DT)] for i in range(30)]
    history = run(tr, frames)
    assert all(ids(h) == {"CAM_FRONT:T1"} for h in history)
    assert history[0][0].state is TrackState.TENTATIVE
    assert history[1][0].state is TrackState.CONFIRMED  # min_hits = 2
    vx, _ = history[-1][0].velocity_px_s
    assert vx == pytest.approx(8 / DT, rel=0.05)  # image-plane velocity, px/s


def test_persists_through_brief_misses_then_expires() -> None:
    tr = IouTracker("CAM_FRONT", TrackerConfig(max_missed_s=0.6))
    seq = [[det("person", 100, 50, 160, 250)]] * 3 + [[]] * 5 + [[det("person", 104, 50, 164, 250)]]
    history = run(tr, seq)
    assert ids(history[-1]) == {"CAM_FRONT:T1"}  # 5 missed frames (0.33 s) bridged
    assert not history[-2][0].updated  # coasting while missed
    tr2 = IouTracker("CAM_FRONT", TrackerConfig(max_missed_s=0.3))
    history2 = run(tr2, seq)
    assert ids(history2[-1]) == {"CAM_FRONT:T2"}  # expired, new id
    assert tr2.history("CAM_FRONT:T1") == []


def test_track_expires_when_object_leaves() -> None:
    tr = IouTracker("CAM_FRONT", TrackerConfig(max_missed_s=0.5))
    run(tr, [[det("chair", 10, 10, 60, 60)]] * 3)
    assert tr.update([], 2 * DT + 0.49)  # last seen at 2*DT: still remembered
    assert tr.update([], 2 * DT + 0.6) == []


def test_two_overlapping_objects_keep_identities() -> None:
    tr = IouTracker("CAM_FRONT")
    frames = []
    for i in range(20):  # two people walk past each other with overlapping boxes
        a = det("person", 100 + 10 * i, 50, 160 + 10 * i, 250)
        b = det("person", 330 - 10 * i, 60, 390 - 10 * i, 260)
        frames.append([a, b])
    history = run(tr, frames)
    # Track T1 started on the left-moving-right object; it must still be moving right.
    final = {s.track_id: s for s in history[-1]}
    assert set(final) == {"CAM_FRONT:T1", "CAM_FRONT:T2"}
    assert final["CAM_FRONT:T1"].velocity_px_s[0] > 0
    assert final["CAM_FRONT:T2"].velocity_px_s[0] < 0


def test_different_classes_never_associate() -> None:
    tr = IouTracker("CAM_FRONT")
    run(tr, [[det("person", 100, 50, 160, 250)]] * 2)
    snaps = tr.update([det("bicycle", 100, 50, 160, 250)], 2 * DT)
    assert ids(snaps) == {"CAM_FRONT:T1", "CAM_FRONT:T2"}


def test_duplicate_detections_do_not_spawn_duplicate_tracks() -> None:
    tr = IouTracker("CAM_FRONT")
    run(tr, [[det("person", 100, 50, 160, 250)]] * 2)
    snaps = tr.update(
        [det("person", 101, 50, 161, 250), det("person", 102, 51, 160, 249, conf=0.6)], 2 * DT
    )
    assert len(snaps) == 1


def test_center_gate_associates_fast_small_object_with_zero_iou() -> None:
    tr = IouTracker("CAM_FRONT", TrackerConfig(min_hits=1))
    tr.update([det("person", 100, 100, 110, 130)], 0.0)
    snaps = tr.update([det("person", 112, 100, 122, 130)], DT)  # no overlap, adjacent
    assert ids(snaps) == {"CAM_FRONT:T1"}


def test_history_is_bounded_and_scale_rate_is_image_plane_only() -> None:
    tr = IouTracker("CAM_FRONT", TrackerConfig(max_history=5))
    frames = [[det("car", 200 - 2 * i, 100 - 2 * i, 260 + 2 * i, 160 + 2 * i)] for i in range(12)]
    history = run(tr, frames)
    assert len(tr.history("CAM_FRONT:T1")) == 5
    snap = history[-1][0]
    assert snap.scale_rate_per_s > 0  # the box grows ("looming")...
    obs = track_to_observation(snap)
    assert obs.closing_speed_cm_s is None  # ...but that is NOT a measured closing speed
    assert obs.metadata["image_scale_rate_per_s"] == snap.scale_rate_per_s


def test_observations_only_from_confirmed_tracks_with_required_fields() -> None:
    tr = IouTracker("CAM_REAR")
    first = tr.update([det("person", 10, 10, 60, 200, camera="CAM_REAR", bearing=200.0)], 5.0)
    assert observations_from_tracks(first) == []  # tentative
    second = tr.update(
        [det("person", 12, 10, 62, 200, t=5.1, camera="CAM_REAR", bearing=201.0)], 5.1
    )
    (obs,) = observations_from_tracks(second)
    assert obs.source is SourceType.CAMERA
    assert obs.range_status is RangeStatus.UNKNOWN
    assert obs.distance_cm is None and obs.closing_speed_cm_s is None
    assert obs.id == "CAM_REAR:T1" and obs.sensor_id == "CAM_REAR"
    assert obs.timestamp == 5.1
    assert obs.bearing_deg == pytest.approx(201.0)
    assert obs.bearing_uncertainty_deg == 5.0
    assert obs.metadata["label"] == "person"


def test_coasting_track_keeps_old_timestamp_so_it_ages() -> None:
    tr = IouTracker("CAM_FRONT")
    run(tr, [[det("person", 100, 50, 160, 250)]] * 2)
    (snap,) = tr.update([], 0.5)
    assert track_to_observation(snap).timestamp == pytest.approx(DT)


def test_tracker_config_validation() -> None:
    with pytest.raises(ValueError):
        TrackerConfig(iou_threshold=0)
    with pytest.raises(ValueError):
        TrackerConfig(min_hits=0)
