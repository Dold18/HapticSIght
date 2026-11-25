"""Benchmarks: detector latency on frames, and the full concurrent vision system.

Results report *what was measured on what input*; callers must label whether
frames came from real webcams, prerecorded video, or synthetic images.
"""

from __future__ import annotations

import resource
import statistics
import sys
import time
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from typing import Any

from hapticsight.vision.detector import ObjectDetector
from hapticsight.vision.processor import VisionSystem
from hapticsight.vision.types import Image


def max_rss_mb() -> float:
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return rss / 1e6 if sys.platform == "darwin" else rss / 1e3  # bytes on macOS, KiB on Linux


def cpu_seconds() -> float:
    r = resource.getrusage(resource.RUSAGE_SELF)
    return r.ru_utime + r.ru_stime


def _pct(values: Sequence[float], q: float) -> float:
    s = sorted(values)
    return s[min(len(s) - 1, int(len(s) * q))]


@dataclass
class LatencyStats:
    n: int
    mean_ms: float
    p50_ms: float
    p95_ms: float
    max_ms: float

    @classmethod
    def of(cls, ms: Sequence[float]) -> LatencyStats:
        return cls(
            len(ms),
            round(statistics.mean(ms), 2),
            round(_pct(ms, 0.5), 2),
            round(_pct(ms, 0.95), 2),
            round(max(ms), 2),
        )


def benchmark_detector(
    detector: ObjectDetector, frames: Sequence[Image], warmup: int = 5
) -> dict[str, Any]:
    if not frames:
        raise ValueError("no frames to benchmark")
    for f in frames[:warmup]:
        detector.detect(f)
    lat = []
    cpu0, t0 = cpu_seconds(), time.perf_counter()
    for f in frames:
        s = time.perf_counter()
        detector.detect(f)
        lat.append((time.perf_counter() - s) * 1000)
    wall = time.perf_counter() - t0
    return {
        "detector": detector.name,
        "frames": len(frames),
        "frame_size": list(frames[0].shape[1::-1]),
        "inference": asdict(LatencyStats.of(lat)),
        "throughput_fps": round(len(frames) / wall, 1),
        "cpu_utilisation_cores": round((cpu_seconds() - cpu0) / wall, 2),
        "max_rss_mb": round(max_rss_mb(), 1),
    }


def benchmark_system(system: VisionSystem, seconds: float) -> dict[str, Any]:
    """Run the full capture + inference system and sample its performance."""
    inference: dict[str, list[float]] = {}
    latency: dict[str, list[float]] = {}
    seen: dict[str, int] = {}
    cpu0 = cpu_seconds()
    system.start()
    t0 = time.perf_counter()
    try:
        while time.perf_counter() - t0 < seconds:
            snap = system.snapshot()
            for cid, r in snap.results.items():
                if r.stale or seen.get(cid) == r.frame_seq:
                    continue
                seen[cid] = r.frame_seq
                inference.setdefault(cid, []).append(r.inference_ms)
                latency.setdefault(cid, []).append(r.latency_s * 1000)
            time.sleep(0.002)
        final = system.snapshot()
    finally:
        system.stop()
    wall = time.perf_counter() - t0
    cams: dict[str, Any] = {}
    for cid, st in final.statuses.items():
        perf = final.perf.get(cid)
        cams[cid] = {
            "state": st.state.value,
            "capture_fps": round(st.fps, 1),
            "frame_size": [st.width, st.height],
            "processed_frames": perf.processed_frames if perf else 0,
            "processed_fps_whole_run": round((perf.processed_frames if perf else 0) / wall, 1),
            "processed_fps_steady": round(perf.inference_fps, 1) if perf else 0.0,
            "inference": asdict(LatencyStats.of(inference[cid])) if inference.get(cid) else None,
            "capture_to_observation": (
                asdict(LatencyStats.of(latency[cid])) if latency.get(cid) else None
            ),
        }
    return {
        "seconds": round(wall, 1),
        "detector": system.detector.name,
        "max_inference_fps_per_camera": system.config.max_inference_fps_per_camera,
        "cameras": cams,
        "cpu_utilisation_cores": round((cpu_seconds() - cpu0) / wall, 2),
        "max_rss_mb": round(max_rss_mb(), 1),
        "worker_error": final.worker_error,
    }
