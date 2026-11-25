"""Vision configuration: cameras, detector, tracker. Loadable from TOML.

Nothing about the physical webcams (resolution, FPS, FOV) is hard-coded: the
defaults are placeholders that request the driver's default mode and use an
explicitly *uncalibrated* nominal FOV with large uncertainty. See
``config/vision.example.toml`` and docs/COMPUTER_VISION.md.
"""

from __future__ import annotations

import dataclasses
import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

from hapticsight.core.layout import CAMERAS
from hapticsight.vision.coco import OBSTACLE_CLASSES
from hapticsight.vision.detector import DetectionFilter, Runtime
from hapticsight.vision.geometry import CameraCalibration, CameraGeometry
from hapticsight.vision.models import DEFAULT_MODEL
from hapticsight.vision.tracker import TrackerConfig

Backend = Literal["auto", "avfoundation", "v4l2", "dshow", "msmf", "ffmpeg"]


@dataclass(frozen=True, slots=True)
class CameraConfig:
    camera_id: str
    mount_bearing_deg: float
    enabled: bool = True
    # Device index (int), device name/URL, or a video file path (str).
    device: int | str | None = None
    width: int | None = None  # None = driver default
    height: int | None = None
    fps: float | None = None
    backend: Backend = "auto"
    loop_video: bool = True  # for file sources in live mode
    stall_timeout_s: float = 1.0  # no new frame for this long -> STALLED
    reconnect_interval_s: float = 2.0
    max_failed_reads: int = 10
    max_frame_age_s: float = 0.5  # frames older than this are not processed
    calibration: CameraCalibration = field(default_factory=CameraCalibration)

    def geometry(self) -> CameraGeometry:
        return CameraGeometry(self.camera_id, self.mount_bearing_deg, self.calibration)


def default_cameras() -> tuple[CameraConfig, ...]:
    """CAM_FRONT -> device 0, CAM_REAR -> device 1 (reassign in config)."""
    return tuple(
        CameraConfig(spec.camera_id, spec.bearing_deg, device=i) for i, spec in enumerate(CAMERAS)
    )


@dataclass(frozen=True, slots=True)
class DetectorConfig:
    model: str = DEFAULT_MODEL
    runtime: Runtime | Literal["auto"] = "auto"
    score_threshold: float = 0.25  # pre-NMS
    nms_threshold: float = 0.45
    num_threads: int | None = None


@dataclass(frozen=True, slots=True)
class VisionConfig:
    cameras: tuple[CameraConfig, ...] = field(default_factory=default_cameras)
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    filter: DetectionFilter = field(default_factory=DetectionFilter)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    max_inference_fps_per_camera: float = 15.0  # caps CPU/thermal load on the M1
    model_dir: str = "models"

    def camera(self, camera_id: str) -> CameraConfig:
        for c in self.cameras:
            if c.camera_id == camera_id:
                return c
        raise KeyError(camera_id)

    def with_camera(self, camera_id: str, **changes: Any) -> VisionConfig:
        cams = tuple(
            dataclasses.replace(c, **changes) if c.camera_id == camera_id else c
            for c in self.cameras
        )
        return dataclasses.replace(self, cameras=cams)


# ---------------------------------------------------------------- TOML


def _replace_checked(instance: Any, data: dict[str, Any], path: str) -> Any:
    names = {f.name for f in dataclasses.fields(instance)}
    changes: dict[str, Any] = {}
    for key, value in data.items():
        if key not in names:
            raise KeyError(f"unknown vision config key {path}{key}")
        current = getattr(instance, key)
        if dataclasses.is_dataclass(current) and isinstance(value, dict):
            changes[key] = _replace_checked(current, value, f"{path}{key}.")
        elif key == "classes":
            changes[key] = None if value in ("all", None) else frozenset(value)
        else:
            changes[key] = value
    return dataclasses.replace(instance, **changes)


def vision_config_from_dict(data: dict[str, Any], base: VisionConfig | None = None) -> VisionConfig:
    cfg = base or VisionConfig()
    data = dict(data)
    cams = data.pop("cameras", {})
    result: VisionConfig = _replace_checked(cfg, data, "")
    for camera_id, cam_data in cams.items():
        updated = _replace_checked(result.camera(camera_id), cam_data, f"cameras.{camera_id}.")
        result = dataclasses.replace(
            result,
            cameras=tuple(updated if c.camera_id == camera_id else c for c in result.cameras),
        )
    return result


def load_vision_config(*paths: str | Path) -> VisionConfig:
    """Load and merge TOML files in order (later files override earlier ones)."""
    cfg = VisionConfig()
    for p in paths:
        with open(p, "rb") as fh:
            cfg = vision_config_from_dict(tomllib.load(fh), cfg)
    return cfg


def calibration_to_toml(camera_id: str, cal: CameraCalibration) -> str:
    """Serialise a calibration as a ``[cameras.<id>.calibration]`` TOML table."""
    lines = [f"[cameras.{camera_id}.calibration]"]
    for f in dataclasses.fields(cal):
        v = getattr(cal, f.name)
        if v is None:
            continue
        if isinstance(v, bool):
            lines.append(f"{f.name} = {'true' if v else 'false'}")
        elif isinstance(v, str):
            lines.append(f'{f.name} = "{v}"')
        elif isinstance(v, float):
            lines.append(f"{f.name} = {v:.4f}")
        else:
            lines.append(f"{f.name} = {v}")
    return "\n".join(lines) + "\n"


__all__ = [
    "OBSTACLE_CLASSES",
    "CameraConfig",
    "DetectorConfig",
    "VisionConfig",
    "calibration_to_toml",
    "default_cameras",
    "load_vision_config",
    "vision_config_from_dict",
]
