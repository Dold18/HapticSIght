from __future__ import annotations

import urllib.request
from pathlib import Path

import numpy as np
import pytest

from hapticsight.vision import models as model_registry
from hapticsight.vision.coco import COCO_LABELS, OBSTACLE_CLASSES
from hapticsight.vision.detector import (
    DetectionFilter,
    ScriptedDetector,
    YoloxDetector,
    letterbox,
    nms,
    yolox_postprocess,
)
from hapticsight.vision.models import (
    MODELS,
    ModelNotInstalledError,
    ModelSpec,
    download_model,
    sha256_of,
)
from tests.vision_helpers import raw

REPO = Path(__file__).resolve().parents[1]


def test_filter_threshold_classes_area_and_limit() -> None:
    dets = [
        raw("person", 0, 0, 100, 100, 0.9),
        raw("person", 0, 0, 100, 100, 0.3),  # below threshold
        raw("cup", 0, 0, 100, 100, 0.95),  # not an obstacle class
        raw("chair", 0, 0, 2, 2, 0.95),  # speck
        raw("car", 0, 0, 100, 100, 0.8),
    ]
    out = DetectionFilter(min_confidence=0.4).apply(dets, 640, 480)
    assert [d.label for d in out] == ["person", "car"]
    assert len(DetectionFilter(classes=None, max_detections=2).apply(dets, 640, 480)) == 2
    with pytest.raises(ValueError):
        DetectionFilter(classes=frozenset({"wall"}))  # not a COCO class
    assert "person" in OBSTACLE_CLASSES and "cup" not in OBSTACLE_CLASSES


def test_nms_keeps_best_of_overlapping_boxes() -> None:
    boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]], dtype=np.float32)
    scores = np.array([0.8, 0.9, 0.7], dtype=np.float32)
    assert nms(boxes, scores, 0.5) == [1, 2]


def test_letterbox_preserves_aspect_and_pads() -> None:
    img = np.full((360, 640, 3), 255, dtype=np.uint8)
    out, ratio = letterbox(img, 416)
    assert out.shape == (416, 416, 3)
    assert ratio == pytest.approx(416 / 640)
    assert out[-1, -1].tolist() == [114, 114, 114]  # padding value


def test_yolox_postprocess_decodes_known_anchor() -> None:
    size = 416
    n = (size // 8) ** 2 + (size // 16) ** 2 + (size // 32) ** 2
    raw_out = np.zeros((n, 85), dtype=np.float32)
    raw_out[:, 4] = -10  # objectness logits are already sigmoid-ed in the export: use 0
    raw_out[:, 4] = 0.0
    # stride-8 grid cell (x=10, y=20): index = 20 * 52 + 10
    i = 20 * 52 + 10
    raw_out[i, 0:2] = [0.5, 0.5]  # offset inside the cell
    raw_out[i, 2:4] = [np.log(4.0), np.log(8.0)]  # w = 4*8 = 32, h = 8*8 = 64 px
    raw_out[i, 4] = 0.9
    raw_out[i, 5 + COCO_LABELS.index("person")] = 0.95
    dets = yolox_postprocess(raw_out, size, ratio=0.5, score_threshold=0.3, nms_threshold=0.45)
    assert len(dets) == 1
    d = dets[0]
    assert d.label == "person"
    assert d.confidence == pytest.approx(0.9 * 0.95)
    cx, cy = (10.5 * 8) / 0.5, (20.5 * 8) / 0.5  # back to original image scale
    assert d.box.center_x == pytest.approx(cx)
    assert d.box.center_y == pytest.approx(cy)
    assert d.box.width == pytest.approx(32 / 0.5)


def test_missing_model_gives_install_hint(tmp_path: Path) -> None:
    with pytest.raises(ModelNotInstalledError, match="hapticsight-vision download-model"):
        YoloxDetector("yolox_tiny", model_dir=tmp_path)


def test_registry_has_provenance_licence_and_checksums() -> None:
    for spec in MODELS.values():
        assert spec.license == "Apache-2.0"
        assert spec.url.startswith("https://github.com/Megvii-BaseDetection/YOLOX/releases/")
        assert len(spec.sha256) == 64
        assert "Megvii" in spec.provenance


def test_download_verifies_checksum_offline(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    src = tmp_path / "src.onnx"
    src.write_bytes(b"fake model bytes")
    good = ModelSpec("fake", "fake.onnx", src.as_uri(), sha256_of(src), 16, 416, 0, "x", "test")
    bad = ModelSpec("bad", "bad.onnx", src.as_uri(), "0" * 64, 16, 416, 0, "x", "test")
    monkeypatch.setitem(model_registry.MODELS, "fake", good)
    monkeypatch.setitem(model_registry.MODELS, "bad", bad)
    calls = []
    real = urllib.request.urlretrieve
    monkeypatch.setattr(urllib.request, "urlretrieve", lambda u, p: calls.append(u) or real(u, p))
    out = download_model("fake", tmp_path / "models")
    assert out.read_bytes() == b"fake model bytes"
    assert download_model("fake", tmp_path / "models") == out  # cached, verified, no refetch
    assert len(calls) == 1
    with pytest.raises(ValueError, match="checksum mismatch"):
        download_model("bad", tmp_path / "models")
    assert not (tmp_path / "models" / "bad.onnx").exists()


def test_scripted_detector_is_deterministic() -> None:
    d = ScriptedDetector({0: [raw("person", 0, 0, 10, 10)], 2: [raw("car", 0, 0, 10, 10)]})
    img = np.zeros((10, 10, 3), np.uint8)
    assert [x.label for x in d.detect(img)] == ["person"]
    assert d.detect(img) == []
    assert [x.label for x in d.detect(img)] == ["car"]


@pytest.mark.skipif(
    not (REPO / "models" / "yolox_tiny.onnx").exists(),
    reason="real model not installed (hapticsight-vision download-model yolox_tiny)",
)
@pytest.mark.parametrize("runtime", ["onnxruntime", "opencv"])
def test_real_yolox_model_runs_locally(runtime: str) -> None:
    det = YoloxDetector("yolox_tiny", runtime=runtime, model_dir=REPO / "models")  # type: ignore[arg-type]
    blank = np.full((360, 640, 3), 120, dtype=np.uint8)
    out = det.detect(blank)
    assert isinstance(out, list)
    assert all(0 <= d.box.x1 <= d.box.x2 <= 640 for d in out)
