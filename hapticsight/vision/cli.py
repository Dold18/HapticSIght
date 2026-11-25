"""``hapticsight-vision``: camera discovery, models, live demo, replay, recording, benchmarks.

Configuration: ``--config FILE`` (repeatable) is merged over the defaults, and
``config/cameras.local.toml`` (git-ignored, machine-specific) is loaded
automatically when present.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import sys
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

from hapticsight.config import HapticSightConfig, load_config
from hapticsight.vision.config import (
    VisionConfig,
    calibration_to_toml,
    load_vision_config,
)

LOCAL_CAMERA_CONFIG = Path("config/cameras.local.toml")


def _source(value: str) -> dict[str, Any]:
    """Parse a camera source: index, 'off', 'synthetic', or a video path."""
    v = value.strip()
    if v.lower() in ("off", "none", "disabled"):
        return {"enabled": False}
    if v.isdigit():
        return {"enabled": True, "device": int(v)}
    return {"enabled": True, "device": v}


def build_vision_config(args: argparse.Namespace) -> VisionConfig:
    paths = list(getattr(args, "config", None) or [])
    if LOCAL_CAMERA_CONFIG.exists() and str(LOCAL_CAMERA_CONFIG) not in paths:
        paths.insert(0, str(LOCAL_CAMERA_CONFIG))
    cfg = load_vision_config(*paths) if paths else VisionConfig()
    for cid, attr in (("CAM_FRONT", "front"), ("CAM_REAR", "rear")):
        src = getattr(args, attr, None)
        if src is not None:
            cfg = cfg.with_camera(cid, **_source(src))
        for prop in ("width", "height", "fps"):
            value = getattr(args, prop, None)
            if value:
                cfg = cfg.with_camera(cid, **{prop: value})
        hfov = getattr(args, f"{attr}_hfov", None)
        if hfov:
            cal = replace(cfg.camera(cid).calibration, hfov_deg=hfov)
            cfg = cfg.with_camera(cid, calibration=cal)
    det = cfg.detector
    if getattr(args, "model", None):
        det = replace(det, model=args.model)
    if getattr(args, "runtime", None):
        det = replace(det, runtime=args.runtime)
    cfg = replace(cfg, detector=det)
    if getattr(args, "max_inference_fps", None):
        cfg = replace(cfg, max_inference_fps_per_camera=args.max_inference_fps)
    if getattr(args, "min_confidence", None) is not None:
        cfg = replace(cfg, filter=replace(cfg.filter, min_confidence=args.min_confidence))
    if getattr(args, "all_classes", False):
        cfg = replace(cfg, filter=replace(cfg.filter, classes=None))
    return cfg


def build_pipeline_config(args: argparse.Namespace) -> HapticSightConfig:
    cfg = (
        load_config(args.pipeline_config)
        if getattr(args, "pipeline_config", None)
        else (HapticSightConfig())
    )
    if getattr(args, "advisory", False):
        cfg = replace(cfg, haptics=replace(cfg.haptics, advisory_cue_enabled=True))
    return cfg


# ---------------------------------------------------------------- commands


def cmd_cameras(args: argparse.Namespace) -> int:
    from hapticsight.vision.discovery import (
        COMMON_MODES,
        diagnose,
        macos_camera_names,
        probe_cameras,
    )

    names = macos_camera_names()
    if names:
        print("Cameras reported by macOS (system_profiler; order may differ from OpenCV index):")
        for n in names:
            print(f"  - {n['name']}  model={n['model_id']}  id={n['unique_id']}")
    results = probe_cameras(args.max_index, args.backend, COMMON_MODES if args.modes else ())
    print(f"\nOpenCV probe (backend={args.backend}):")
    for r in results:
        if not r.opened:
            print(f"  index {r.index}: not available ({r.error})")
            continue
        first = f"{r.first_frame_s:.2f}s" if r.first_frame_s is not None else "n/a"
        print(
            f"  index {r.index}: opened, frames={'yes' if r.frame_ok else 'NO'}, "
            f"default {r.width}x{r.height} @ {r.fps:g} fps (driver-reported), first frame {first}"
        )
        for m in r.modes:
            rw, rh, rf = m.requested
            aw, ah, af = m.actual
            print(f"      request {rw}x{rh}@{rf:g} -> got {aw}x{ah}@{af:g} ok={m.frame_ok}")
    for hint in diagnose(results, names):
        print(f"\nNOTE: {hint}")
    return 0


def cmd_models(args: argparse.Namespace) -> int:
    from hapticsight.vision.models import DEFAULT_MODEL, MODELS

    for spec in MODELS.values():
        path = Path(args.model_dir) / spec.filename
        mark = "installed" if path.exists() else "not installed"
        default = " (default)" if spec.name == DEFAULT_MODEL else ""
        print(
            f"{spec.name:<11}{default:<10} {spec.input_size}px  COCO mAP {spec.coco_map:4.1f}  "
            f"{spec.size_bytes / 1e6:5.1f} MB  {spec.license}  [{mark}]"
        )
    print(f"\nprovenance: {next(iter(MODELS.values())).provenance}")
    return 0


def cmd_download(args: argparse.Namespace) -> int:
    from hapticsight.vision.models import MODELS, download_model

    names = list(MODELS) if args.name == "all" else [args.name]
    for name in names:
        spec = MODELS[name]
        print(f"downloading {name} ({spec.size_bytes / 1e6:.1f} MB) from {spec.url}")
        path = download_model(name, Path(args.model_dir), force=args.force)
        print(f"  ok: {path} (sha256 verified, license {spec.license})")
    return 0


def _make_detector(cfg: VisionConfig) -> Any:
    from hapticsight.vision.factory import create_detector
    from hapticsight.vision.models import ModelNotInstalledError

    try:
        return create_detector(cfg.detector, cfg.model_dir)
    except ModelNotInstalledError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


def cmd_live(args: argparse.Namespace) -> int:
    from hapticsight.simulation.manual_ultrasonic import ManualUltrasonic
    from hapticsight.vision.live_app import LiveVisionApp
    from hapticsight.vision.processor import VisionSystem

    cfg = build_vision_config(args)
    detector = _make_detector(cfg)
    label = detector.name
    if cfg.detector.model == "synthetic":
        label = "SYNTHETIC MARKER DETECTOR (demo only, not a real model)"
    system = VisionSystem(cfg, detector)
    app = LiveVisionApp(
        system,
        build_pipeline_config(args),
        ManualUltrasonic.from_spec(args.ultrasonic),
        detector_label=label,
        record_dir=Path(args.record) if args.record else None,
        fps=args.ui_fps,
    )
    if args.screenshot:
        import time

        import pygame

        system.start()
        try:
            t_end = time.monotonic() + args.warmup
            while time.monotonic() < t_end:
                app.step()
                time.sleep(1 / 30)
            app.step()
            app.draw()
            pygame.image.save(app.screen, args.screenshot)
            print(f"saved {args.screenshot}")
        finally:
            app.close()
        return 0
    import time

    t0 = time.monotonic()
    app.run(max_frames=args.frames)
    if args.frames:
        elapsed = time.monotonic() - t0
        snap = app.snapshot
        print(
            f"UI: {app.frames_rendered} frames in {elapsed:.1f}s "
            f"({app.frames_rendered / elapsed:.1f} fps incl. start-up)"
        )
        for cid, st in snap.statuses.items():
            perf = snap.perf[cid]
            res = snap.results.get(cid)
            print(
                f"  {cid}: {st.state.value} capture {st.fps:.1f} fps, inference "
                f"{perf.inference_fps:.1f} fps {perf.inference_ms:.1f} ms, cap->obs "
                f"{perf.latency_ms:.0f} ms, tracks now {len(res.tracks) if res else 0}"
            )
    return 0


def cmd_replay(args: argparse.Namespace) -> int:
    from hapticsight.pipeline import HapticSightPipeline
    from hapticsight.simulation.manual_ultrasonic import ManualUltrasonic
    from hapticsight.vision.recording import replay

    cfg = build_vision_config(args)
    videos = {
        cid: path for cid, path in (("CAM_FRONT", args.front), ("CAM_REAR", args.rear)) if path
    }
    if not videos:
        print("error: give --front and/or --rear video paths", file=sys.stderr)
        return 2
    detector = _make_detector(cfg)
    us = ManualUltrasonic.from_spec(args.ultrasonic) if args.ultrasonic else None
    with contextlib.ExitStack() as stack:
        export = stack.enter_context(open(args.export, "w")) if args.export else None
        summary = replay(
            videos,
            {cid: cfg.camera(cid) for cid in videos},
            detector,
            cfg.filter,
            cfg.tracker,
            pipeline=HapticSightPipeline(build_pipeline_config(args)),
            ultrasonic=us.readings if us else None,
            export=export,
            annotate_dir=Path(args.annotate) if args.annotate else None,
            max_frames=args.max_frames,
        )
    info = summary.as_dict()
    info["detector"] = detector.name
    info["ultrasonic"] = "SIMULATED " + args.ultrasonic if args.ultrasonic else "none"
    print(json.dumps(info, indent=2))
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    import time

    from hapticsight.vision.camera import ThreadedCapture
    from hapticsight.vision.factory import smart_opener
    from hapticsight.vision.recording import VideoRecorder

    cfg = build_vision_config(args)
    out = Path(args.out) / time.strftime("rec-%Y%m%d-%H%M%S")
    caps = [ThreadedCapture(c, opener=smart_opener) for c in cfg.cameras if c.enabled]
    recorders: dict[str, VideoRecorder] = {}
    last: dict[str, int] = {}
    for c in caps:
        c.start()
    try:
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            for c in caps:
                f = c.slot.latest()
                if f is None or last.get(c.camera_id) == f.seq:
                    continue
                last[c.camera_id] = f.seq
                if c.camera_id not in recorders:
                    fps = c.status().fps or (c.config.fps or 30.0)
                    recorders[c.camera_id] = VideoRecorder(
                        out / f"{c.camera_id}.mp4", fps, (f.width, f.height)
                    )
                recorders[c.camera_id].add(f)
            time.sleep(0.003)
    finally:
        for c in caps:
            c.stop()
        for r in recorders.values():
            r.close()
    for c in caps:
        st = c.status()
        rec = recorders.get(c.camera_id)
        print(
            f"{c.camera_id}: {st.state.value} ({st.detail}); "
            f"written {rec.written if rec else 0} frames, dropped {rec.dropped if rec else 0}"
        )
    print(f"recordings (local only): {out}")
    return 0


def cmd_benchmark(args: argparse.Namespace) -> int:
    from hapticsight.vision.benchmark import benchmark_detector, benchmark_system

    cfg = build_vision_config(args)
    detector = _make_detector(cfg)
    if args.system:
        from hapticsight.vision.processor import VisionSystem

        result: dict[str, Any] = benchmark_system(VisionSystem(cfg, detector), args.seconds)
        result["input"] = {c.camera_id: str(c.device) for c in cfg.cameras if c.enabled}
    else:
        frames = _benchmark_frames(args)
        result = benchmark_detector(detector, frames)
        result["input"] = args.video or "synthetic"
    print(json.dumps(result, indent=2))
    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(result, indent=2))
    return 0


def _benchmark_frames(args: argparse.Namespace) -> list[Any]:
    if args.video:
        from hapticsight.vision.recording import iter_video

        frames = []
        for f in iter_video(args.video, "bench"):
            frames.append(f.image)
            if len(frames) >= args.frames:
                break
        return frames
    from hapticsight.vision.synthetic import DEMO_SCENE

    return [DEMO_SCENE.render(i / DEMO_SCENE.fps) for i in range(args.frames)]


def cmd_calibrate(args: argparse.Namespace) -> int:
    from hapticsight.vision.geometry import fit_calibration

    cfg = build_vision_config(args)
    cam = cfg.camera(args.camera)
    points = []
    for p in args.point:
        u, _, b = p.partition(":")
        points.append((float(u), float(b)))
    fit = fit_calibration(points, args.image_width, cam.mount_bearing_deg, cam.calibration)
    c = fit.calibration
    print(
        f"{args.camera}: HFOV {c.hfov_deg:.2f}°  yaw offset {c.yaw_offset_deg:+.2f}°  "
        f"rms residual {fit.rms_residual_deg:.2f}° from {fit.points} point(s)"
    )
    block = calibration_to_toml(args.camera, c)
    print("\n" + block)
    if args.write:
        LOCAL_CAMERA_CONFIG.parent.mkdir(parents=True, exist_ok=True)
        existing = LOCAL_CAMERA_CONFIG.read_text() if LOCAL_CAMERA_CONFIG.exists() else ""
        header = f"[cameras.{args.camera}.calibration]"
        if header in existing:
            print(f"error: {LOCAL_CAMERA_CONFIG} already has {header}; edit it manually")
            return 1
        LOCAL_CAMERA_CONFIG.write_text(existing + ("\n" if existing else "") + block)
        print(f"appended to {LOCAL_CAMERA_CONFIG}")
    return 0


def cmd_synth(args: argparse.Namespace) -> int:
    from hapticsight.vision.synthetic import write_synthetic_video

    n = write_synthetic_video(args.out, seconds=args.seconds)
    print(f"wrote {n} synthetic frames to {args.out} (+ timestamps sidecar)")
    return 0


# ---------------------------------------------------------------- parser


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hapticsight-vision", description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="command", required=True)

    def common(sp: argparse.ArgumentParser, cameras: bool = True, detector: bool = True) -> None:
        sp.add_argument("--config", action="append", help="vision TOML (repeatable)")
        if cameras:
            sp.add_argument(
                "--front", help="CAM_FRONT source: index | video path | synthetic | off"
            )
            sp.add_argument("--rear", help="CAM_REAR source: index | video path | synthetic | off")
            sp.add_argument("--width", type=int, help="requested capture width")
            sp.add_argument("--height", type=int, help="requested capture height")
            sp.add_argument("--fps", type=float, help="requested capture FPS")
            sp.add_argument("--front-hfov", type=float, help="override CAM_FRONT HFOV (deg)")
            sp.add_argument("--rear-hfov", type=float, help="override CAM_REAR HFOV (deg)")
        if detector:
            sp.add_argument("--model", help="yolox_nano | yolox_tiny | yolox_s | path | synthetic")
            sp.add_argument(
                "--runtime", choices=["auto", "onnxruntime", "onnxruntime-coreml", "opencv"]
            )
            sp.add_argument("--min-confidence", type=float)
            sp.add_argument("--all-classes", action="store_true", help="disable class filter")
            sp.add_argument("--max-inference-fps", type=float)

    sp = sub.add_parser("cameras", help="list and probe cameras")
    sp.add_argument("--max-index", type=int, default=4)
    sp.add_argument("--backend", default="auto")
    sp.add_argument("--modes", action="store_true", help="also test common resolutions")
    sp.set_defaults(func=cmd_cameras)

    sp = sub.add_parser("models", help="list supported models")
    sp.add_argument("--model-dir", default="models")
    sp.set_defaults(func=cmd_models)

    sp = sub.add_parser("download-model", help="download a model (explicit network use)")
    sp.add_argument("name", help="model name or 'all'")
    sp.add_argument("--model-dir", default="models")
    sp.add_argument("--force", action="store_true")
    sp.set_defaults(func=cmd_download)

    sp = sub.add_parser("live", help="live dual-camera diagnostic UI")
    common(sp)
    sp.add_argument("--ultrasonic", help='SIMULATED ranges, e.g. "S0=80,S1=X,S2=D"')
    sp.add_argument("--advisory", action="store_true", help="enable experimental advisory cue")
    sp.add_argument("--pipeline-config", help="HapticSight pipeline TOML")
    sp.add_argument("--record", help="record raw camera video + session log to DIR")
    sp.add_argument("--ui-fps", type=int, default=30)
    sp.add_argument("--frames", type=int, help="exit after N UI frames")
    sp.add_argument("--screenshot", help="save one UI frame to PNG and exit")
    sp.add_argument("--warmup", type=float, default=3.0, help="seconds before --screenshot")
    sp.set_defaults(func=cmd_live)

    sp = sub.add_parser("replay", help="deterministically process recorded video")
    common(sp, cameras=False)
    sp.add_argument("--front", help="CAM_FRONT video")
    sp.add_argument("--rear", help="CAM_REAR video")
    sp.add_argument("--front-hfov", type=float)
    sp.add_argument("--rear-hfov", type=float)
    sp.add_argument("--ultrasonic", help='SIMULATED ranges, e.g. "S0=80"')
    sp.add_argument("--advisory", action="store_true")
    sp.add_argument("--pipeline-config")
    sp.add_argument("--export", help="per-frame JSONL output")
    sp.add_argument("--annotate", help="directory for annotated videos")
    sp.add_argument("--max-frames", type=int)
    sp.set_defaults(func=cmd_replay)

    sp = sub.add_parser("record", help="record camera video locally")
    common(sp, detector=False)
    sp.add_argument("--seconds", type=float, default=10.0)
    sp.add_argument("--out", default="recordings")
    sp.set_defaults(func=cmd_record)

    sp = sub.add_parser("benchmark", help="measure detector / system performance")
    common(sp)
    sp.add_argument("--video", help="benchmark on frames from this video")
    sp.add_argument("--frames", type=int, default=150)
    sp.add_argument("--system", action="store_true", help="run full capture+inference system")
    sp.add_argument("--seconds", type=float, default=20.0)
    sp.add_argument("--json", help="also write results to this file")
    sp.set_defaults(func=cmd_benchmark)

    sp = sub.add_parser("calibrate", help="fit HFOV/yaw from reference points")
    common(sp, cameras=False, detector=False)
    sp.add_argument("--camera", default="CAM_FRONT")
    sp.add_argument(
        "--image-width", type=int, required=True, help="image width of the measurements"
    )
    sp.add_argument(
        "--point", action="append", required=True, help="PIXEL_X:TRUE_BODY_BEARING (repeatable)"
    )
    sp.add_argument("--write", action="store_true", help=f"append to {LOCAL_CAMERA_CONFIG}")
    sp.set_defaults(func=cmd_calibrate)

    sp = sub.add_parser("synth-video", help="write a synthetic test video (no camera needed)")
    sp.add_argument("out")
    sp.add_argument("--seconds", type=float, default=8.0)
    sp.set_defaults(func=cmd_synth)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
