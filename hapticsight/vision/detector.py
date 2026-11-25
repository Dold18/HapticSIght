"""Object detectors.

* :class:`YoloxDetector`: real pretrained YOLOX (COCO-80) ONNX model, run with
  ONNX Runtime (default) or OpenCV DNN. Runs fully locally.
* :class:`ScriptedDetector`: deterministic outputs for tests and replays.
* :class:`DetectionFilter`: confidence threshold, class allow-list, and size limits.

Detectors return :class:`RawDetection` in *original image* pixel coordinates.
"""

from __future__ import annotations

import os
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Protocol

import numpy as np

from hapticsight.vision.coco import COCO_LABELS, OBSTACLE_CLASSES
from hapticsight.vision.models import ModelNotInstalledError, resolve_model
from hapticsight.vision.types import BoundingBox, Image, RawDetection

Runtime = Literal["onnxruntime", "onnxruntime-coreml", "opencv"]


class ObjectDetector(Protocol):
    name: str

    def detect(self, image: Image) -> list[RawDetection]: ...


@dataclass(frozen=True, slots=True)
class DetectionFilter:
    min_confidence: float = 0.4
    classes: frozenset[str] | None = OBSTACLE_CLASSES  # None = all classes
    min_box_area_fraction: float = 0.0005  # ignore specks (fraction of frame area)
    max_detections: int = 50

    def __post_init__(self) -> None:
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be within [0, 1]")
        if self.classes is not None:
            unknown = set(self.classes) - set(COCO_LABELS)
            if unknown:
                raise ValueError(f"unknown class names: {sorted(unknown)}")

    def apply(
        self, detections: Iterable[RawDetection], frame_w: int, frame_h: int
    ) -> list[RawDetection]:
        min_area = self.min_box_area_fraction * frame_w * frame_h
        kept = [
            d
            for d in detections
            if d.confidence >= self.min_confidence
            and (self.classes is None or d.label in self.classes)
            and d.box.area >= min_area
        ]
        kept.sort(key=lambda d: -d.confidence)
        return kept[: self.max_detections]


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    """Greedy non-maximum suppression. ``boxes`` is (N, 4) x1, y1, x2, y2."""
    order = np.argsort(-scores, kind="stable")
    keep: list[int] = []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    while order.size:
        i = int(order[0])
        keep.append(i)
        rest = order[1:]
        ix = np.maximum(0, np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest]))
        iy = np.maximum(0, np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest]))
        inter = ix * iy
        iou = inter / np.maximum(areas[i] + areas[rest] - inter, 1e-9)
        order = rest[iou <= iou_threshold]
    return keep


def letterbox(image: Image, size: int) -> tuple[np.ndarray, float]:
    """Resize keeping aspect ratio, pad bottom/right with 114 (YOLOX convention)."""
    import cv2

    h, w = image.shape[:2]
    ratio = min(size / h, size / w)
    nh, nw = round(h * ratio), round(w * ratio)
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[:nh, :nw] = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return padded, ratio


def yolox_decode(
    raw: np.ndarray, input_size: int, strides: Sequence[int] = (8, 16, 32)
) -> np.ndarray:
    """Decode raw YOLOX grid outputs (N, 85) into absolute cx, cy, w, h (input pixels)."""
    grids, expanded = [], []
    for s in strides:
        n = input_size // s
        xv, yv = np.meshgrid(np.arange(n), np.arange(n))
        grids.append(np.stack((xv, yv), 2).reshape(-1, 2))
        expanded.append(np.full((n * n, 1), s))
    grid = np.concatenate(grids, 0).astype(np.float32)
    stride = np.concatenate(expanded, 0).astype(np.float32)
    out = raw.copy()
    out[:, :2] = (out[:, :2] + grid) * stride
    out[:, 2:4] = np.exp(out[:, 2:4]) * stride
    return out


def yolox_postprocess(
    raw: np.ndarray,
    input_size: int,
    ratio: float,
    score_threshold: float,
    nms_threshold: float,
    labels: Sequence[str] = COCO_LABELS,
) -> list[RawDetection]:
    pred = yolox_decode(raw, input_size)
    scores_all = pred[:, 4:5] * pred[:, 5:]
    class_ids = scores_all.argmax(1)
    scores = scores_all[np.arange(len(class_ids)), class_ids]
    mask = scores >= score_threshold
    if not mask.any():
        return []
    pred, scores, class_ids = pred[mask], scores[mask], class_ids[mask]
    boxes = np.empty((len(pred), 4), dtype=np.float32)
    boxes[:, 0] = pred[:, 0] - pred[:, 2] / 2
    boxes[:, 1] = pred[:, 1] - pred[:, 3] / 2
    boxes[:, 2] = pred[:, 0] + pred[:, 2] / 2
    boxes[:, 3] = pred[:, 1] + pred[:, 3] / 2
    boxes /= ratio
    # Class-aware NMS via the coordinate-offset trick.
    offset = class_ids[:, None].astype(np.float32) * 10_000.0
    keep = nms(boxes + offset, scores, nms_threshold)
    out = []
    for i in keep:
        x1, y1, x2, y2 = (float(v) for v in boxes[i])
        cid = int(class_ids[i])
        out.append(
            RawDetection(
                class_id=cid,
                label=labels[cid],
                confidence=float(scores[i]),
                box=BoundingBox(x1, y1, max(x1, x2), max(y1, y2)),
            )
        )
    return out


class YoloxDetector:
    """YOLOX COCO detector from an official ONNX export."""

    def __init__(
        self,
        model: str = "yolox_tiny",
        runtime: Runtime = "onnxruntime",
        score_threshold: float = 0.25,
        nms_threshold: float = 0.45,
        num_threads: int | None = None,
        model_dir: Path | None = None,
    ) -> None:
        path, registry_name = resolve_model(model, model_dir or Path("models"))
        if not path.exists():
            raise ModelNotInstalledError(path, registry_name)
        self.path = path
        self.runtime = runtime
        self.score_threshold = score_threshold
        self.nms_threshold = nms_threshold
        self.name = f"{path.stem}[{runtime}]"
        self._infer: Callable[[np.ndarray], np.ndarray]
        if runtime == "opencv":
            import cv2

            net = cv2.dnn.readNetFromONNX(str(path))
            if num_threads:
                cv2.setNumThreads(num_threads)
            self.input_size = _input_size_from_name(path)

            def infer_cv(blob: np.ndarray) -> np.ndarray:
                net.setInput(blob)
                return np.asarray(net.forward())

            self._infer = infer_cv
        else:
            import onnxruntime as ort

            os.environ.setdefault("ORT_LOGGING_LEVEL", "3")
            opts = ort.SessionOptions()
            opts.log_severity_level = 3
            if num_threads:
                opts.intra_op_num_threads = num_threads
            providers: list[Any] = ["CPUExecutionProvider"]
            if runtime == "onnxruntime-coreml":
                providers.insert(0, "CoreMLExecutionProvider")
            session = ort.InferenceSession(str(path), sess_options=opts, providers=providers)
            inp = session.get_inputs()[0]
            self.input_size = int(inp.shape[2])
            input_name = inp.name

            def infer_ort(blob: np.ndarray) -> np.ndarray:
                return np.asarray(session.run(None, {input_name: blob})[0])

            self._infer = infer_ort

    def preprocess(self, image: Image) -> tuple[np.ndarray, float]:
        padded, ratio = letterbox(image, self.input_size)
        blob = np.ascontiguousarray(padded.transpose(2, 0, 1)[None].astype(np.float32))
        return blob, ratio

    def detect(self, image: Image) -> list[RawDetection]:
        blob, ratio = self.preprocess(image)
        raw = self._infer(blob)[0]
        h, w = image.shape[:2]
        return [
            RawDetection(d.class_id, d.label, d.confidence, d.box.clipped(w, h))
            for d in yolox_postprocess(
                raw, self.input_size, ratio, self.score_threshold, self.nms_threshold
            )
        ]


def _input_size_from_name(path: Path) -> int:
    return 640 if path.stem.endswith("_s") or path.stem.endswith("_m") else 416


@dataclass
class ScriptedDetector:
    """Deterministic detector for tests: returns preset detections per call.

    ``script`` maps a call index (0, 1, 2, ...) to detections; missing indices
    return nothing (a missed detection). Alternatively pass ``fn`` that maps the
    image to detections.
    """

    script: Mapping[int, Sequence[RawDetection]] = field(default_factory=dict)
    fn: Callable[[Image], list[RawDetection]] | None = None
    name: str = "scripted"
    calls: int = 0

    def detect(self, image: Image) -> list[RawDetection]:
        i = self.calls
        self.calls += 1
        if self.fn is not None:
            return self.fn(image)
        return list(self.script.get(i, ()))
