"""Registry of supported pretrained detector models (provenance, license, checksum).

Model files are **never** committed and never downloaded during tests. Install
explicitly with ``hapticsight-vision download-model <name>``.
"""

from __future__ import annotations

import hashlib
import urllib.request
from dataclasses import dataclass
from pathlib import Path

DEFAULT_MODEL_DIR = Path("models")


@dataclass(frozen=True, slots=True)
class ModelSpec:
    name: str
    filename: str
    url: str
    sha256: str
    size_bytes: int
    input_size: int
    coco_map: float  # COCO val2017 mAP@[.5:.95] as published by the authors
    license: str
    provenance: str


_YOLOX_RELEASE = "https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0"
_YOLOX_PROVENANCE = (
    "Megvii-BaseDetection/YOLOX GitHub release 0.1.1rc0 (official ONNX export). "
    "Code and weights Apache-2.0. Trained on COCO 2017 (annotations CC BY 4.0; images "
    "from Flickr under their own terms)."
)

MODELS: dict[str, ModelSpec] = {
    "yolox_nano": ModelSpec(
        "yolox_nano",
        "yolox_nano.onnx",
        f"{_YOLOX_RELEASE}/yolox_nano.onnx",
        "c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d",
        3_659_407,
        416,
        25.8,
        "Apache-2.0",
        _YOLOX_PROVENANCE,
    ),
    "yolox_tiny": ModelSpec(
        "yolox_tiny",
        "yolox_tiny.onnx",
        f"{_YOLOX_RELEASE}/yolox_tiny.onnx",
        "427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7",
        20_219_662,
        416,
        32.8,
        "Apache-2.0",
        _YOLOX_PROVENANCE,
    ),
    "yolox_s": ModelSpec(
        "yolox_s",
        "yolox_s.onnx",
        f"{_YOLOX_RELEASE}/yolox_s.onnx",
        "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063",
        35_858_002,
        640,
        40.5,
        "Apache-2.0",
        _YOLOX_PROVENANCE,
    ),
}

DEFAULT_MODEL = "yolox_tiny"


class ModelNotInstalledError(FileNotFoundError):
    def __init__(self, path: Path, name: str | None) -> None:
        hint = f"hapticsight-vision download-model {name}" if name else "check the --model path"
        super().__init__(f"model file not found: {path}\n  install it with: {hint}")


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def resolve_model(
    name_or_path: str, model_dir: Path = DEFAULT_MODEL_DIR
) -> tuple[Path, str | None]:
    """Return (path, registry name or None) for a model name or explicit path."""
    if name_or_path in MODELS:
        return model_dir / MODELS[name_or_path].filename, name_or_path
    return Path(name_or_path), None


def download_model(name: str, model_dir: Path = DEFAULT_MODEL_DIR, force: bool = False) -> Path:
    """Download a registered model and verify its SHA-256. Network use is explicit."""
    spec = MODELS[name]
    model_dir.mkdir(parents=True, exist_ok=True)
    target = model_dir / spec.filename
    if target.exists() and not force:
        if sha256_of(target) == spec.sha256:
            return target
        raise ValueError(f"{target} exists but its checksum does not match; use --force")
    tmp = target.with_suffix(".part")
    urllib.request.urlretrieve(spec.url, tmp)
    digest = sha256_of(tmp)
    if digest != spec.sha256:
        tmp.unlink(missing_ok=True)
        raise ValueError(f"checksum mismatch for {name}: got {digest}, expected {spec.sha256}")
    tmp.replace(target)
    return target
