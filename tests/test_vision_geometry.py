from __future__ import annotations

import math

import pytest

from hapticsight.core.geometry import angular_distance
from hapticsight.vision.geometry import CameraCalibration, CameraGeometry, fit_calibration
from hapticsight.vision.types import BoundingBox

W = 1280


def front(**cal: object) -> CameraGeometry:
    return CameraGeometry("CAM_FRONT", 0.0, CameraCalibration(**cal))  # type: ignore[arg-type]


def rear(**cal: object) -> CameraGeometry:
    return CameraGeometry("CAM_REAR", 180.0, CameraCalibration(**cal))  # type: ignore[arg-type]


def test_centre_pixel_maps_to_mount_bearing() -> None:
    assert front().body_bearing_deg(W / 2, W) == pytest.approx(0.0)
    assert rear().body_bearing_deg(W / 2, W) == pytest.approx(180.0)


@pytest.mark.parametrize("hfov", [50.0, 60.0, 78.0, 90.0])
def test_image_edges_map_to_half_fov(hfov: float) -> None:
    g = front(hfov_deg=hfov)
    assert g.camera_angle_deg(W, W) == pytest.approx(hfov / 2)
    assert g.camera_angle_deg(0, W) == pytest.approx(-hfov / 2)
    assert g.effective_hfov_deg(W) == pytest.approx(hfov)


def test_front_camera_right_is_wearer_right_and_wraps_left() -> None:
    g = front(hfov_deg=60.0)
    assert g.body_bearing_deg(W, W) == pytest.approx(30.0)  # right edge -> 30°
    assert g.body_bearing_deg(0, W) == pytest.approx(330.0)  # left edge -> 330°, not -30°


def test_rear_camera_image_right_is_wearer_rear_left() -> None:
    """The rear camera faces backwards: its right-hand side is the wearer's left."""
    g = rear(hfov_deg=60.0)
    assert g.body_bearing_deg(W, W) == pytest.approx(210.0)  # rear-left (M5 side)
    assert g.body_bearing_deg(0, W) == pytest.approx(150.0)  # rear-right (M3 side)


def test_mirrored_webcam_is_flipped() -> None:
    assert front(mirrored=True).body_bearing_deg(W, W) == pytest.approx(330.0)
    assert front(mirrored=True).body_bearing_deg(0, W) == pytest.approx(30.0)


def test_yaw_offset_and_wraparound() -> None:
    g = front(yaw_offset_deg=-3.0)
    assert g.body_bearing_deg(W / 2, W) == pytest.approx(357.0)
    g2 = rear(yaw_offset_deg=185.0)  # silly but must still normalise
    assert 0 <= g2.body_bearing_deg(W / 2, W) < 360


def test_cameras_need_not_share_fov() -> None:
    a = front(hfov_deg=55.0)
    b = rear(hfov_deg=85.0)
    assert a.camera_angle_deg(W, W) == pytest.approx(27.5)
    assert b.camera_angle_deg(W, W) == pytest.approx(42.5)


def test_measured_intrinsics_scale_with_resolution() -> None:
    fx = (1280 / 2) / math.tan(math.radians(35))
    g = front(fx=fx, cx=650.0, reference_width=1280)
    assert g.fx(640) == pytest.approx(fx / 2)
    assert g.cx(640) == pytest.approx(325.0)
    assert g.camera_angle_deg(650.0, 1280) == pytest.approx(0.0)
    assert g.camera_angle_deg(325.0, 640) == pytest.approx(0.0)


def test_radial_distortion_inversion() -> None:
    k1 = -0.2
    g = front(hfov_deg=70.0, k1=k1)
    x_true = 0.5  # undistorted normalised coordinate
    x_dist = x_true * (1 + k1 * x_true**2)
    u = W / 2 + x_dist * g.fx(W)
    assert g.camera_angle_deg(u, W) == pytest.approx(math.degrees(math.atan(x_true)), abs=1e-3)


@pytest.mark.parametrize("bearing", [-25.0, -10.0, 0.0, 12.5, 29.0])
def test_inverse_mapping_round_trip(bearing: float) -> None:
    g = front(hfov_deg=60.0, yaw_offset_deg=2.0)
    u = g.column_for_body_bearing(bearing, W)
    assert u is not None
    assert angular_distance(g.body_bearing_deg(u, W), bearing) == pytest.approx(0.0, abs=1e-6)
    assert g.column_for_body_bearing(180.0, W) is None  # behind the camera


def box_at(u: float, half_w: float = 20.0) -> BoundingBox:
    return BoundingBox(u - half_w, 100, u + half_w, 300)


def test_uncertainty_budget_behaviour() -> None:
    g = front()
    centre = g.estimate(box_at(W / 2), W)
    edge = g.estimate(box_at(W - 100), W)
    assert centre.uncertainty_deg > 0
    assert edge.uncertainty_deg > centre.uncertainty_deg  # FOV + distortion terms grow
    truncated = g.estimate(BoundingBox(W - 200, 0, W, 300), W)
    whole = g.estimate(BoundingBox(W - 260, 0, W - 60, 300), W)
    assert truncated.uncertainty_deg > whole.uncertainty_deg  # object cut off by frame edge
    calibrated = front(
        hfov_uncertainty_deg=1.0, yaw_uncertainty_deg=1.0, distortion_uncertainty_deg=0.5
    )
    assert calibrated.estimate(box_at(W - 100), W).uncertainty_deg < edge.uncertainty_deg


def test_estimate_reports_extent_in_body_frame() -> None:
    est = rear(hfov_deg=60.0).estimate(BoundingBox(W - 200, 0, W, 300), W)
    # left/right edges of the box map to bearings around rear-left
    assert 180 < est.left_deg < est.right_deg <= 210.0 + 1e-6
    assert angular_distance(est.bearing_deg, 180 + est.camera_angle_deg) < 1e-9


def test_calibration_validation() -> None:
    with pytest.raises(ValueError):
        CameraCalibration(hfov_deg=0.5)
    with pytest.raises(ValueError):
        CameraCalibration(fx=500.0)  # missing reference width
    with pytest.raises(ValueError):
        CameraCalibration(yaw_uncertainty_deg=-1.0)


# ------------------------------------------------------------ calibration fitting


def synth_points(hfov: float, yaw: float, mount: float, bearings: list[float]):  # type: ignore[no-untyped-def]
    g = CameraGeometry("c", mount, CameraCalibration(hfov_deg=hfov, yaw_offset_deg=yaw))
    return [(g.column_for_body_bearing(b, W), b) for b in bearings]


def test_fit_recovers_hfov_and_yaw() -> None:
    pts = synth_points(72.0, 4.0, 0.0, [-25.0, -10.0, 0.0, 15.0, 30.0])
    fit = fit_calibration(pts, W, 0.0)
    assert fit.calibration.hfov_deg == pytest.approx(72.0, abs=0.05)
    assert fit.calibration.yaw_offset_deg == pytest.approx(4.0, abs=0.03)
    assert fit.rms_residual_deg < 0.05
    assert fit.calibration.calibrated
    assert fit.calibration.yaw_uncertainty_deg == pytest.approx(1.0)  # 1° floor


def test_fit_rear_camera_and_noisy_points() -> None:
    pts = synth_points(65.0, -2.0, 180.0, [155.0, 170.0, 185.0, 200.0])
    noisy = [(u + d, b) for (u, b), d in zip(pts, [3.0, -2.0, 2.0, -3.0], strict=True)]  # type: ignore[operator]
    fit = fit_calibration(noisy, W, 180.0)
    assert fit.calibration.hfov_deg == pytest.approx(65.0, abs=1.0)
    assert fit.calibration.yaw_offset_deg == pytest.approx(-2.0, abs=0.3)


def test_fit_single_point_solves_yaw_only() -> None:
    fit = fit_calibration([(W / 2 + 100, 8.0)], W, 0.0, CameraCalibration(hfov_deg=60.0))
    g = CameraGeometry("c", 0.0, fit.calibration)
    assert g.body_bearing_deg(W / 2 + 100, W) == pytest.approx(8.0)
    assert fit.calibration.hfov_deg == 60.0


def test_fit_rejects_bad_input() -> None:
    with pytest.raises(ValueError):
        fit_calibration([], W, 0.0)
    with pytest.raises(ValueError):
        fit_calibration([(640.0, 0.0), (650.0, 1.0)], W, 0.0)  # points too close together
