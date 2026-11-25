"""Construct detectors and capture openers from configuration."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

from hapticsight.vision.config import CameraConfig, DetectorConfig
from hapticsight.vision.detector import ObjectDetector, Runtime, YoloxDetector
from hapticsight.vision.synthetic import DEMO_SCENE, MarkerDetector, SyntheticScene


def resolve_runtime(requested: str) -> Runtime:
    if requested != "auto":
        return requested  # type: ignore[return-value]
    try:
        import onnxruntime as ort

        providers = ort.get_available_providers()
    except ImportError:
        return "opencv"
    if sys.platform == "darwin" and "CoreMLExecutionProvider" in providers:
        return "onnxruntime-coreml"
    return "onnxruntime"


def create_detector(config: DetectorConfig, model_dir: str | Path = "models") -> ObjectDetector:
    if config.model == "synthetic":
        return MarkerDetector()
    return YoloxDetector(
        config.model,
        runtime=resolve_runtime(config.runtime),
        score_threshold=config.score_threshold,
        nms_threshold=config.nms_threshold,
        num_threads=config.num_threads,
        model_dir=Path(model_dir),
    )


class SyntheticCapture:
    """A VideoCapture-like device that renders a :class:`SyntheticScene` in real time."""

    def __init__(self, scene: SyntheticScene = DEMO_SCENE, loop_s: float = 10.0) -> None:
        self.scene = scene
        self.loop_s = loop_s
        self._t0 = time.monotonic()
        self._next = self._t0
        self._open = True

    def isOpened(self) -> bool:
        return self._open

    def read(self) -> tuple[bool, Any]:
        if not self._open:
            return False, None
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self._next += 1.0 / self.scene.fps
        t = (time.monotonic() - self._t0) % self.loop_s
        return True, self.scene.render(t)

    def set(self, prop: int, value: float) -> bool:
        return False

    def get(self, prop: int) -> float:
        return self.scene.fps

    def release(self) -> None:
        self._open = False


def smart_opener(config: CameraConfig) -> Any:
    """OpenCV for devices/files; ``device = "synthetic"`` renders the demo scene."""
    from hapticsight.vision.camera import opencv_opener

    if config.device == "synthetic":
        return SyntheticCapture()
    return opencv_opener(config)
