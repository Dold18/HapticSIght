"""Shared fakes for vision tests (no webcams, no model files)."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np

from hapticsight.vision.coco import COCO_LABELS
from hapticsight.vision.types import BoundingBox, CameraFrame, Detection, RawDetection


def raw(label: str, x1: float, y1: float, x2: float, y2: float, conf: float = 0.9) -> RawDetection:
    return RawDetection(COCO_LABELS.index(label), label, conf, BoundingBox(x1, y1, x2, y2))


def det(
    label: str,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    t: float = 0.0,
    conf: float = 0.9,
    camera: str = "CAM_FRONT",
    bearing: float = 0.0,
) -> Detection:
    return Detection(
        COCO_LABELS.index(label), label, conf, BoundingBox(x1, y1, x2, y2), camera, t,
        640, 360, bearing, 5.0, bearing - 2, bearing + 2,
    )  # fmt: skip


def frame(camera: str = "CAM_FRONT", t: float = 0.0, seq: int = 1, w: int = 640, h: int = 360):  # type: ignore[no-untyped-def]
    return CameraFrame(camera, np.zeros((h, w, 3), dtype=np.uint8), t, seq)


class FakeCapture:
    """Scriptable VideoCapture stand-in.

    ``fail_reads_after``: reads succeed this many times, then fail forever.
    ``block_after``: after this many reads, ``read`` blocks until released (stall).
    """

    def __init__(
        self,
        opened: bool = True,
        fps: float = 100.0,
        fail_reads_after: int | None = None,
        block_after: int | None = None,
        size: tuple[int, int] = (64, 48),
    ) -> None:
        self.opened = opened
        self.period = 1.0 / fps
        self.reads = 0
        self.fail_reads_after = fail_reads_after
        self.block_after = block_after
        self.released = False
        self._unblock = threading.Event()
        self.size = size

    def isOpened(self) -> bool:
        return self.opened and not self.released

    def read(self) -> tuple[bool, Any]:
        if self.block_after is not None and self.reads >= self.block_after:
            self._unblock.wait(5.0)
            return False, None
        self.reads += 1
        time.sleep(self.period)
        if self.fail_reads_after is not None and self.reads > self.fail_reads_after:
            return False, None
        w, h = self.size
        return True, np.full((h, w, 3), self.reads % 255, dtype=np.uint8)

    def set(self, prop: int, value: float) -> bool:
        return True

    def get(self, prop: int) -> float:
        return 1.0 / self.period

    def release(self) -> None:
        self.released = True
        self._unblock.set()


class OpenerLog:
    """Opener that hands out a sequence of FakeCaptures and counts calls."""

    def __init__(self, *captures: FakeCapture) -> None:
        self.captures = list(captures)
        self.calls = 0
        self.issued: list[FakeCapture] = []

    def __call__(self, config: Any) -> FakeCapture:
        self.calls += 1
        cap = self.captures.pop(0) if len(self.captures) > 1 else self.captures[0]
        if cap in self.issued and cap.released:
            cap = FakeCapture(opened=cap.opened, fps=1.0 / cap.period)
        self.issued.append(cap)
        return cap


def wait_until(pred: Any, timeout: float = 3.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.005)
    return bool(pred())
