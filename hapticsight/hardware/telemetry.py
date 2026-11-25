"""Structured local telemetry logs (JSON Lines) for debugging and evaluation.

One file per session under the chosen directory (git-ignored ``recordings/`` by
default). Each line is ``{"t": <host monotonic s>, "kind": ..., ...}``. Kinds:

``session``      metadata: sources (real / simulated), port, firmware, config
``rx_line``      raw line from the Arduino (+ whether it decoded)
``tx_line``      raw line sent to the Arduino
``link_state``   bridge state changes; ``reset_detected``, ``reset_pulse``
``readings``     ultrasonic readings that entered the pipeline (source-labelled)
``step``         per host-loop summary: observations, top assessments, haptic frame

No camera images are written here; video recording is a separate opt-in.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import IO, Any


class TelemetryLogger:
    def __init__(self, directory: str | Path, prefix: str = "session") -> None:
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.path = self.dir / f"{prefix}-{stamp}.jsonl"
        n = 1
        while self.path.exists():
            self.path = self.dir / f"{prefix}-{stamp}-{n}.jsonl"
            n += 1
        self._fh: IO[str] | None = open(self.path, "w")  # noqa: SIM115
        self.records = 0

    def log(self, kind: str, t: float, **data: Any) -> None:
        if self._fh is None:
            return
        rec = {"t": round(t, 6), "kind": kind, **data}
        self._fh.write(json.dumps(rec, default=str) + "\n")
        self.records += 1

    def hook(self, kind: str, t: float, data: dict[str, Any]) -> None:
        """Adapter for ``ArduinoBridge(on_event=logger.hook)``."""
        self.log(kind, t, **data)

    def flush(self) -> None:
        if self._fh is not None:
            self._fh.flush()

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


def read_log(path: str | Path) -> list[dict[str, Any]]:
    with open(path) as fh:
        return [json.loads(line) for line in fh if line.strip()]
