"""Image column -> body-relative bearing, with an explicit uncertainty budget.

Model (horizontal only, viewed from above):

* Pinhole camera with horizontal focal length ``fx`` (pixels) and principal
  point ``cx``. Either measured (``fx``/``cx`` from calibration) or derived from a
  horizontal field of view: ``fx = (W / 2) / tan(HFOV / 2)``.
* Optional radial distortion (``k1``, ``k2``) on the normalised x coordinate.
* Camera angle ``a = atan((u - cx) / fx)`` is positive to the **camera's right**,
  which is clockwise when viewed from above, for any mounting direction.
* Body bearing ``b = normalize(mount_bearing + yaw_offset + a)``. So for the
  rear camera (mount 180°) an object on the right of its image is at ~200°,
  i.e. behind-left of the wearer. That is correct, because the rear camera's
  right is the wearer's left.
* Mirrored webcams (selfie-style) are handled with ``mirrored = true``.

Bearing uncertainty (± degrees) is the root-sum-square of:

1. ``yaw_uncertainty_deg``: mounting / aiming error of the camera.
2. FOV term: how much the angle at this column changes if the HFOV is off by
   ``± hfov_uncertainty_deg``. Zero at the image centre, largest at the edges.
3. Distortion term: ``distortion_uncertainty_deg`` scaled by |normalised x|.
4. Box term: 10 % of the box's angular width (centre-of-box ≠ centre-of-object),
   or half the angular width if the box touches the left/right frame edge
   (the object is truncated, so its true centre is unknown).

No distance is ever derived from images.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

from hapticsight.core.geometry import normalize_bearing
from hapticsight.vision.types import BoundingBox

BOX_CENTER_FRACTION = 0.10
EDGE_TOLERANCE_PX = 2.0


@dataclass(frozen=True, slots=True)
class CameraCalibration:
    """Horizontal calibration for one camera. Defaults are deliberately *uncalibrated*."""

    hfov_deg: float = 60.0
    fx: float | None = None  # pixels, at ``reference_width``
    cx: float | None = None  # pixels, at ``reference_width``
    reference_width: int | None = None
    k1: float = 0.0
    k2: float = 0.0
    yaw_offset_deg: float = 0.0  # correction added to the nominal mount bearing
    mirrored: bool = False
    hfov_uncertainty_deg: float = 15.0
    yaw_uncertainty_deg: float = 5.0
    distortion_uncertainty_deg: float = 2.0
    calibrated: bool = False
    note: str = "uncalibrated nominal values"

    def __post_init__(self) -> None:
        if not 1.0 < self.hfov_deg < 179.0:
            raise ValueError("hfov_deg must be within (1, 179)")
        if (self.fx is not None or self.cx is not None) and not self.reference_width:
            raise ValueError("fx/cx require reference_width")
        if self.fx is not None and self.fx <= 0:
            raise ValueError("fx must be positive")
        for name in ("hfov_uncertainty_deg", "yaw_uncertainty_deg", "distortion_uncertainty_deg"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True, slots=True)
class BearingEstimate:
    bearing_deg: float
    uncertainty_deg: float
    left_deg: float
    right_deg: float
    camera_angle_deg: float


@dataclass(frozen=True, slots=True)
class CameraGeometry:
    camera_id: str
    mount_bearing_deg: float
    calibration: CameraCalibration = CameraCalibration()

    # ------------------------------------------------------------ intrinsics

    def fx(self, width: int) -> float:
        c = self.calibration
        if c.fx is not None and c.reference_width:
            return c.fx * width / c.reference_width
        return (width / 2.0) / math.tan(math.radians(c.hfov_deg) / 2.0)

    def cx(self, width: int) -> float:
        c = self.calibration
        if c.cx is not None and c.reference_width:
            return c.cx * width / c.reference_width
        return width / 2.0

    def effective_hfov_deg(self, width: int) -> float:
        fx, cx = self.fx(width), self.cx(width)
        return math.degrees(math.atan(cx / fx) + math.atan((width - cx) / fx))

    # ------------------------------------------------------------ mapping

    def _normalised_x(self, u: float, width: int, fx: float | None = None) -> float:
        x = (u - self.cx(width)) / (fx if fx is not None else self.fx(width))
        if self.calibration.mirrored:
            x = -x
        return self._undistort(x)

    def _undistort(self, xd: float) -> float:
        k1, k2 = self.calibration.k1, self.calibration.k2
        if k1 == 0.0 and k2 == 0.0:
            return xd
        x = xd
        for _ in range(8):  # fixed-point inversion of xd = x (1 + k1 x² + k2 x⁴)
            r2 = x * x
            x = xd / (1.0 + k1 * r2 + k2 * r2 * r2)
        return x

    def camera_angle_deg(self, u: float, width: int) -> float:
        """Angle of pixel column ``u`` from the optical axis (+ = camera's right)."""
        return math.degrees(math.atan(self._normalised_x(u, width)))

    def body_bearing_deg(self, u: float, width: int) -> float:
        c = self.calibration
        return normalize_bearing(
            self.mount_bearing_deg + c.yaw_offset_deg + self.camera_angle_deg(u, width)
        )

    def column_for_body_bearing(self, bearing_deg: float, width: int) -> float | None:
        """Inverse mapping (ignores distortion); None if behind the camera."""
        from hapticsight.core.geometry import signed_angle_difference

        c = self.calibration
        a = signed_angle_difference(bearing_deg, self.mount_bearing_deg + c.yaw_offset_deg)
        if abs(a) >= 89.0:
            return None
        x = math.tan(math.radians(a))
        if c.mirrored:
            x = -x
        return self.cx(width) + x * self.fx(width)

    # ------------------------------------------------------------ uncertainty

    def _fov_term(self, u: float, width: int) -> float:
        # Applies to measured focal lengths too: hfov_uncertainty_deg then holds the
        # (small) residual uncertainty from the calibration fit.
        c = self.calibration
        hfov = self.effective_hfov_deg(width)
        lo = max(1.0, hfov - c.hfov_uncertainty_deg)
        hi = min(179.0, hfov + c.hfov_uncertainty_deg)
        x_px = u - self.cx(width)
        a_lo = math.degrees(math.atan(x_px / ((width / 2.0) / math.tan(math.radians(lo) / 2))))
        a_hi = math.degrees(math.atan(x_px / ((width / 2.0) / math.tan(math.radians(hi) / 2))))
        return abs(a_hi - a_lo) / 2.0

    def estimate(self, box: BoundingBox, width: int) -> BearingEstimate:
        c = self.calibration
        u = box.center_x
        angle = self.camera_angle_deg(u, width)
        left = self.body_bearing_deg(box.x1, width)
        right = self.body_bearing_deg(box.x2, width)
        if c.mirrored:
            left, right = right, left
        angular_width = abs(
            self.camera_angle_deg(box.x2, width) - self.camera_angle_deg(box.x1, width)
        )
        truncated = box.x1 <= EDGE_TOLERANCE_PX or box.x2 >= width - EDGE_TOLERANCE_PX
        box_term = angular_width * (0.5 if truncated else BOX_CENTER_FRACTION)
        x_norm = min(1.0, abs(u - self.cx(width)) / max(1.0, width / 2.0))
        terms = (
            c.yaw_uncertainty_deg,
            self._fov_term(u, width),
            c.distortion_uncertainty_deg * x_norm,
            box_term,
        )
        uncertainty = math.sqrt(sum(t * t for t in terms))
        return BearingEstimate(
            bearing_deg=normalize_bearing(self.mount_bearing_deg + c.yaw_offset_deg + angle),
            uncertainty_deg=uncertainty,
            left_deg=left,
            right_deg=right,
            camera_angle_deg=angle,
        )


# ---------------------------------------------------------------- calibration


@dataclass(frozen=True, slots=True)
class CalibrationFit:
    calibration: CameraCalibration
    rms_residual_deg: float
    points: int


def fit_calibration(
    points: Sequence[tuple[float, float]],
    width: int,
    mount_bearing_deg: float,
    base: CameraCalibration | None = None,
    yaw_search_deg: float = 30.0,
) -> CalibrationFit:
    """Fit yaw offset (and focal length if possible) from reference targets.

    ``points`` are ``(pixel_column_u, true_body_bearing_deg)`` pairs: targets
    placed at known bearings around the wearer (e.g. tape marks at 0° and ±20°
    measured with a protractor or tape-measure triangle).

    * 1 point: solves only the yaw offset; HFOV is kept from ``base``.
    * ≥ 2 points spanning the image: solves yaw offset and ``fx`` (principal
      point assumed at the image centre), by a 1-D search over yaw with a
      closed-form least-squares ``fx`` for each candidate.

    Resulting uncertainties are set from the fit residual with a 1° floor.
    """
    from hapticsight.core.geometry import signed_angle_difference

    if not points:
        raise ValueError("need at least one calibration point")
    base = base or CameraCalibration()
    cx = width / 2.0
    geo = CameraGeometry("cal", mount_bearing_deg, replace(base, yaw_offset_deg=0.0))
    sign = -1.0 if base.mirrored else 1.0

    def rel(b: float, yaw: float) -> float:
        return signed_angle_difference(b, mount_bearing_deg + yaw)

    if len(points) == 1:
        u, b = points[0]
        angle = geo.camera_angle_deg(u, width)
        yaw = signed_angle_difference(b, mount_bearing_deg) - angle
        cal = replace(
            base,
            yaw_offset_deg=yaw,
            yaw_uncertainty_deg=1.0,
            calibrated=True,
            note="yaw from 1 reference point; HFOV not calibrated",
        )
        return CalibrationFit(cal, 0.0, 1)

    us = [sign * (u - cx) for u, _ in points]
    if max(us) - min(us) < width * 0.2:
        raise ValueError("calibration points must span at least 20% of the image width")
    best: tuple[float, float, float] | None = None  # (sse, yaw, fx)
    steps = int(yaw_search_deg * 2 / 0.02)
    for i in range(steps + 1):
        yaw = -yaw_search_deg + i * 0.02
        ts = [math.tan(math.radians(rel(b, yaw))) for _, b in points]
        denom = sum(t * t for t in ts)
        if denom == 0:
            continue
        fx = sum(x * t for x, t in zip(us, ts, strict=True)) / denom
        if fx <= 0:
            continue
        sse = sum((x - fx * t) ** 2 for x, t in zip(us, ts, strict=True))
        if best is None or sse < best[0]:
            best = (sse, yaw, fx)
    if best is None:
        raise ValueError("calibration fit failed; check the point bearings")
    _, yaw, fx = best
    residuals = [
        rel(b, yaw) - math.degrees(math.atan(x / fx)) for x, (_, b) in zip(us, points, strict=True)
    ]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    hfov = math.degrees(2 * math.atan((width / 2.0) / fx))
    unc = max(1.0, 2.0 * rms)
    cal = replace(
        base,
        hfov_deg=hfov,
        fx=fx,
        cx=cx,
        reference_width=width,
        yaw_offset_deg=yaw,
        hfov_uncertainty_deg=unc,
        yaw_uncertainty_deg=unc,
        calibrated=True,
        note=f"fitted from {len(points)} reference points (rms {rms:.2f} deg)",
    )
    return CalibrationFit(cal, rms, len(points))
