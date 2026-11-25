"""Camera discovery and capability probing.

OpenCV cannot enumerate device *names*, and on macOS the AVFoundation index
order is not guaranteed to match ``system_profiler`` order, so we report both
and let the user assign devices explicitly in config. Resolutions/FPS are
whatever the driver *negotiates* for a request, which may differ from what was
asked, so we report the actual values read back.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from dataclasses import dataclass, field

from hapticsight.vision.camera import _backend_flag

COMMON_MODES: tuple[tuple[int, int, float], ...] = (
    (640, 480, 30.0),
    (1280, 720, 30.0),
    (1920, 1080, 30.0),
)


@dataclass
class NegotiatedMode:
    requested: tuple[int, int, float]
    actual: tuple[int, int, float]
    frame_ok: bool


@dataclass
class ProbeResult:
    index: int
    backend: str
    opened: bool
    frame_ok: bool = False
    width: int = 0
    height: int = 0
    fps: float = 0.0
    first_frame_s: float | None = None
    modes: list[NegotiatedMode] = field(default_factory=list)
    error: str = ""


def _read_first(cap: object, attempts: int, delay: float) -> tuple[bool, object]:
    for _ in range(attempts):
        ok, img = cap.read()  # type: ignore[attr-defined]
        if ok and img is not None:
            return True, img
        time.sleep(delay)
    return False, None


def probe_camera(
    index: int,
    backend: str = "auto",
    modes: tuple[tuple[int, int, float], ...] = (),
    attempts: int = 20,
) -> ProbeResult:
    import cv2

    if backend == "auto":
        backend = "avfoundation" if sys.platform == "darwin" else "auto"
    t0 = time.monotonic()
    cap = cv2.VideoCapture(index, _backend_flag(backend))
    result = ProbeResult(index=index, backend=backend, opened=cap.isOpened())
    if not result.opened:
        result.error = "could not open"
        cap.release()
        return result
    try:
        ok, img = _read_first(cap, attempts, 0.05)
        result.frame_ok = ok
        result.first_frame_s = time.monotonic() - t0 if ok else None
        if ok:
            result.height, result.width = img.shape[:2]  # type: ignore[attr-defined]
        result.fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        for w, h, f in modes:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, f)
            mok, mimg = _read_first(cap, attempts, 0.05)
            aw, ah = (mimg.shape[1], mimg.shape[0]) if mok else (0, 0)  # type: ignore[attr-defined]
            result.modes.append(
                NegotiatedMode((w, h, f), (aw, ah, float(cap.get(cv2.CAP_PROP_FPS) or 0)), mok)
            )
    finally:
        cap.release()
    return result


def probe_cameras(
    max_index: int = 4, backend: str = "auto", modes: tuple[tuple[int, int, float], ...] = ()
) -> list[ProbeResult]:
    return [probe_camera(i, backend, modes) for i in range(max_index)]


def macos_camera_names() -> list[dict[str, str]]:
    """Camera names from ``system_profiler`` (macOS only; does not need camera permission)."""
    if sys.platform != "darwin":
        return []
    try:
        out = subprocess.run(
            ["system_profiler", "SPCameraDataType", "-json"],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        ).stdout
        items = json.loads(out).get("SPCameraDataType", [])
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return []
    return [
        {
            "name": str(i.get("_name", "?")),
            "model_id": str(i.get("spcamera_model-id", "")),
            "unique_id": str(i.get("spcamera_unique-id", "")),
        }
        for i in items
    ]


def diagnose(results: list[ProbeResult], names: list[dict[str, str]]) -> list[str]:
    """Human-readable hints, e.g. probable missing macOS camera permission."""
    hints = []
    opened = [r for r in results if r.opened]
    if names and not opened:
        hints.append(
            f"macOS reports {len(names)} camera(s) but none could be opened. This almost always "
            "means camera permission has not been granted to the app running Python "
            "(Terminal, iTerm, or VS Code): System Settings > Privacy & Security > Camera, "
            "enable it, then restart that app."
        )
    for r in opened:
        if not r.frame_ok:
            hints.append(f"device {r.index} opened but delivered no frames (in use or stalled?)")
    if names:
        hints.append(
            "OpenCV index order is not guaranteed to match the name list; verify which index "
            "is which with `hapticsight-vision live` before assigning CAM_FRONT / CAM_REAR."
        )
    return hints
