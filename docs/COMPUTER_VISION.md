# Computer Vision (Milestone 2)

This document covers how HapticSight turns two webcam feeds into the shared `Observation`s
used by the Milestone 1 pipeline: capture, detection, tracking, bearing estimation, calibration,
measured performance, and known limitations.

**Status:** implemented and tested in software. It has been run with two real cameras on the
development MacBook (see §7), but **not** with the vest-mounted USB webcams, which were not
available. Nothing here is validated as a safety system.

---

## 1. Data flow

```
 camera thread (CAM_FRONT) ─┐  newest-frame slot (no queue)
 camera thread (CAM_REAR) ──┤
                            ▼
      inference worker (1 thread, round-robin, ≤15 fps per camera by default)
        YOLOX detect → class/confidence filter → pixel→body bearing (+ uncertainty)
        → per-camera IoU tracker → confirmed tracks → Observation(CAMERA, range UNKNOWN)
                            ▼
      HapticSightPipeline.process(now, ultrasonic_readings, camera_observations)
        (unchanged Milestone 1 fusion → risk → haptic encoder)
```

| Module | Role |
|--------|------|
| `vision/camera.py` | `ThreadedCapture`: one thread per camera, newest-frame slot, monotonic timestamps, failure states, reconnection |
| `vision/discovery.py` | probe devices, negotiated modes, macOS camera names, permission diagnosis |
| `vision/detector.py`, `models.py` | YOLOX ONNX detector (ONNX Runtime / CoreML / OpenCV DNN), model registry with checksums |
| `vision/geometry.py` | pixel column → body bearing, uncertainty budget, calibration fit |
| `vision/tracker.py` | IoU + motion-prediction multi-object tracker |
| `vision/observations.py` | confirmed tracks → shared `Observation` |
| `vision/processor.py` | `CameraProcessor` (one camera), `VisionSystem` (threads + worker + snapshots) |
| `vision/recording.py` | local recording with timestamp sidecars; deterministic replay; JSONL export |
| `vision/live_app.py` | live Pygame diagnostic UI |
| `vision/synthetic.py` | synthetic scenes + colour-marker detector for tests (not a real detector) |
| `simulation/manual_ultrasonic.py` | hand-controlled **simulated** ultrasonic ranges for demos |

## 2. Camera capture

* **Backend:** AVFoundation on macOS (`CAP_AVFOUNDATION`, chosen automatically for integer
  device indices); `auto` elsewhere. Video files use FFmpeg.
* **Nothing hard-coded:** width, height, and FPS default to the driver's choice; requested values
  are *negotiated*, and `hapticsight-vision cameras --modes` shows what the driver actually
  delivers. Driver-reported FPS can be wrong (one test device reported "1 fps" while delivering
  30), so the UI and benchmarks report **measured** FPS.
* **Concurrency:** each camera has its own thread, so a slow or stalled device never blocks the
  other. Each thread keeps only the newest frame, so latency cannot accumulate when inference is
  slower than capture (frames are dropped, not queued).
* **Failure states:** `UNAVAILABLE` (cannot open: missing, in use, or no permission; retried every
  2 s), `DISCONNECTED` (repeated read failures; released and reopened), `STALLED` (no frame for
  1 s), `DISABLED` (config or runtime toggle), `FINISHED` (video ended). Frames older than
  `max_frame_age_s` (0.5 s) are never processed.
* **macOS permission:** the app that launches Python (Terminal, iTerm, VS Code) needs
  *System Settings → Privacy & Security → Camera*. Without it OpenCV opens nothing;
  `hapticsight-vision cameras` detects this case (macOS lists cameras but none open) and says so.
* **Device assignment:** OpenCV cannot report device names and macOS index order is not
  guaranteed to match `system_profiler`. Assign indices explicitly in
  `config/cameras.local.toml` after checking them in the live view.
* **Shutdown:** `VisionSystem.stop()` joins the worker and every capture thread and releases all
  devices.

## 3. Object detection: model selection

Candidates were restricted to pretrained COCO detectors with permissive licences that run
locally on Apple Silicon without PyTorch (only ~10 GB of disk was free, and an 8 GB machine
benefits from a small runtime):

| Considered | Licence | Outcome |
|------------|---------|---------|
| **YOLOX-Nano / Tiny / S** (Megvii) | Apache-2.0 code + weights | **benchmarked; YOLOX-Tiny chosen** |
| NanoDet-Plus (OpenCV zoo) | Apache-2.0 | downloaded; dropped: needs a separate decoder, and YOLOX already spans the size range |
| YOLOv5/v8/v11 (Ultralytics) | AGPL-3.0 | excluded: copyleft obligations for a public portfolio repo |
| YOLOv6/v7/v9 | GPL-3.0 | excluded (licence) |
| YOLO-NAS | non-commercial weights | excluded (licence) |
| PyTorch-based detectors (DETR family etc.) | various | excluded: heavyweight runtime for no demonstrated benefit here |

Runtimes compared: ONNX Runtime CPU, ONNX Runtime with the **CoreML execution provider**
(Apple Neural Engine/GPU), and OpenCV DNN. All three produced identical detections.

**Measured on prerecorded video** (OpenCV sample `vtest.avi`, 768×576 pedestrians; 150 frames,
single stream, back-to-back; MacBook Air M1 8 GB, macOS 26.3.1, ONNX Runtime 1.32.0,
OpenCV 5.1.0):

| Model (input) | COCO mAP* | Runtime | Mean / p95 latency | Throughput | Peak RSS |
|---------------|----------:|---------|--------------------|-----------:|---------:|
| YOLOX-Nano (416) | 25.8 | ORT CPU | 9.4 / 10.5 ms | 106 fps | 305 MB |
| YOLOX-Nano (416) | 25.8 | ORT CoreML | 8.7 / 9.5 ms | 115 fps | 318 MB |
| YOLOX-Nano (416) | 25.8 | OpenCV DNN | 12.0 / 12.8 ms | 84 fps | 296 MB |
| **YOLOX-Tiny (416)** | **32.8** | ORT CPU | 30.0 / 32.1 ms | 33 fps | 342 MB |
| **YOLOX-Tiny (416)** | **32.8** | **ORT CoreML** | **11.2 / 12.3 ms** | **89 fps** | 354 MB |
| YOLOX-Tiny (416) | 32.8 | OpenCV DNN | 25.8 / 27.6 ms | 39 fps | 339 MB |
| YOLOX-S (640) | 40.5 | ORT CPU | 105.5 / 110.8 ms | 9.5 fps | 399 MB |
| YOLOX-S (640) | 40.5 | ORT CoreML | 24.3 / 27.2 ms | 41 fps | 347 MB |
| YOLOX-S (640) | 40.5 | OpenCV DNN | 91.8 / 103.8 ms | 11 fps | 415 MB |

\* mAP@[.5:.95] on COCO val2017 as published by the YOLOX authors, not re-measured here.
Model load time: YOLOX-Tiny 0.14 s (CPU) / 1.35 s (CoreML, includes compilation); YOLOX-S
2.26 s with CoreML.

**Decision: YOLOX-Tiny with ONNX Runtime + CoreML** (`runtime = "auto"` picks CoreML on macOS
and CPU ONNX Runtime elsewhere). Tiny gains +7 mAP over Nano for ~2.5 ms. In the dual-camera
system (§7), YOLOX-S costs ~2.7× the inference time, ~2× the capture→observation latency, and
~1.6× the CPU for +8 mAP. YOLOX-S stays available with `--model yolox_s`.

Install (explicit, checksum-verified, never run in CI):

```bash
hapticsight-vision download-model yolox_tiny     # or yolox_nano | yolox_s | all
hapticsight-vision models                        # list, licence, install state
```

Provenance: official ONNX exports from the
[Megvii-BaseDetection/YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) GitHub release
`0.1.1rc0`; SHA-256 checksums are pinned in `hapticsight/vision/models.py`.

### Detection output

Each `Detection` keeps: class label and COCO id, confidence, bounding box, camera id, capture
timestamp, frame size, body-relative bearing, bearing uncertainty, and the bearings of the box's
left/right edges. Filtering (`[filter]` config): confidence ≥ 0.4, an obstacle-relevant class
allow-list (`vision/coco.py: OBSTACLE_CLASSES`, or `classes = "all"`), and a minimum box area.

### What the detector cannot see

A COCO detector only knows 80 everyday object classes. It does **not** detect:

* walls, doors, glass and other transparent surfaces, mirrors;
* poles, posts, lamp posts, sign posts, bollards, traffic cones, barriers, fences, railings;
* curbs, steps, stairs, holes, drop-offs, uneven ground;
* overhanging obstacles (branches, open cabinet doors) or low obstacles below the camera view.

In the sample video, pedestrians, a van, and a car were detected while the lamp post, cones, and
a sign were not. **The ultrasonic sensors, not the cameras, are the primary obstacle detectors;**
the cameras add bearing precision and object identity.

## 4. Tracking

A per-camera IoU tracker (`vision/tracker.py`) with constant-velocity box prediction, a
centre-distance fallback for small or fast boxes, and class-consistent matching. It is greedy
rather than Hungarian, dependency-free, and deterministic.

* **IDs** are stable per camera (`CAM_FRONT:T7`); tracks need 2 hits to be *confirmed*
  (suppressing one-frame false positives) and survive up to 0.6 s of missed detections.
* **One observation per confirmed track**, so repeated detections never become duplicate warnings;
  near-duplicate boxes (IoU ≥ 0.7) never spawn a second track.
* **Stale tracks** keep the timestamp of their last real detection, so the risk engine marks
  them stale; they are deleted after `max_missed_s`.
* **Image-plane motion** (pixel velocity, `d ln(box height)/dt`) is recorded in observation
  metadata **only**. It is never converted into `closing_speed_cm_s`: box growth depends on
  object size, pose, and truncation, and is not a metric measurement.

Evaluation: deterministic synthetic sequences in `tests/test_vision_tracking.py` and
`tests/test_vision_replay_cli.py` (stable IDs through motion, misses, crossings, and
duplicates; one ID per synthetic object across a 60-frame replay; bearing error < 1° against
synthetic ground truth). On the 300-frame pedestrian sample video the tracker held 21 confirmed
tracks, several continuously for the whole clip; there is no ground truth for that video, so no
MOTA-style accuracy is claimed.

## 5. Bearing estimation

`vision/geometry.py` maps the box centre column `u` to a body bearing:

```
camera angle a = atan((u − cx) / fx)          (+ = camera's right; distortion removed first)
body bearing  b = normalize(mount + yaw_offset + a)
fx from calibration, or fx = (W/2) / tan(HFOV/2)
```

* Viewed from above, the camera's right is clockwise for **any** mounting direction. For the
  rear camera (mount 180°) an object on the right of the image is at ~200° (behind-left), and the
  tests pin this.
* Mirrored ("selfie") webcams are supported with `mirrored = true`.
* The cameras need not share a FOV; each has its own calibration.
* No distance is ever derived from images.

### Bearing uncertainty (explicit, per detection)

Root-sum-square of:

| Term | Source | Typical size |
|------|--------|--------------|
| mounting yaw | camera not pointing exactly where assumed | 5° uncalibrated, 1–2° calibrated |
| FOV error | HFOV wrong by ± `hfov_uncertainty_deg`; zero at the centre, largest at the edge | up to ~7° at the edge with ±15° HFOV uncertainty |
| lens distortion | residual radial distortion, scaled by distance from centre | ≤ 2° (uncalibrated default) |
| box | centre of box ≠ centre of object: 10 % of box angular width, or 50 % if the box is cut off by the frame edge | 0.5–10° |

The resulting `bearing_uncertainty_deg` drives the Milestone 1 haptic encoder's honest spreading
across neighbouring motors.

### How much does an uncalibrated FOV matter?

Assuming the nominal 60° HFOV when the real lens is different:

| Real HFOV | Error halfway to the edge | Error at the image edge |
|----------:|--------------------------:|------------------------:|
| 50° | +3.0° | +5.0° |
| 55° | +1.5° | +2.5° |
| 70° | −3.2° | −5.0° |
| 78° | −5.9° | −9.0° |
| 90° | −10.5° | −15.0° |

Motor sectors are 45° wide (boundaries at ±22.5°), so a 5–15° error can move a warning to the
neighbouring motor for objects near a sector boundary. **Calibrate before trusting diagonal
motor assignments.** Mounting yaw errors add directly at every pixel.

## 6. Calibration procedure

You need: the camera mounted as it will be worn (or on a stand at chest height), a tape measure,
and 3–5 targets (cups, cones, tape marks) at known bearings.

1. **Place targets** about 2 m from the camera at known body bearings, e.g. −25°, 0°, +25° for
   CAM_FRONT (or 155°, 180°, 205° for CAM_REAR). A bearing is set by a forward distance `d` and
   a sideways offset `s`: `bearing = atan(s / d)`. For example, 200 cm forward and 93 cm right
   gives 25°.
2. **Quick visual alignment:** `hapticsight-vision live`, press `Tab` to select the camera and
   `C` for the bearing-grid overlay. Adjust yaw with `,` / `.` and HFOV with `-` / `=` until the
   grid lines sit on the targets, then press `W` to write the calibration to
   `config/cameras.local.toml`.
3. **Precise fit (recommended):** read each target's pixel column (from a frame or the overlay)
   and run:
   ```bash
   hapticsight-vision calibrate --camera CAM_FRONT --image-width 1280 \
       --point 212:-25 --point 641:0 --point 1069:25 --write
   ```
   This fits HFOV and yaw offset by least squares (principal point assumed at the centre),
   reports the RMS residual, and sets the uncertainties from it (1° floor). With one point only
   the yaw offset is fitted.
4. **Full intrinsics (optional):** if you have OpenCV chessboard calibration results, set `fx`,
   `cx`, `reference_width`, `k1`, `k2` in the camera's `[cameras.<id>.calibration]` table.
5. Re-check after any remount: yaw errors from mounting usually dominate.

## 7. Measured system performance

All measurements were taken on 2026-11-24 on the MacBook Air M1 (8 GB), with YOLOX-Tiny via
ONNX Runtime + CoreML and the default 15 fps per-camera inference cap unless stated. Latency is
capture timestamp → observations available.

**Real cameras.** These were the development Mac's built-in FaceTime HD camera and an iPhone
Continuity Camera, **standing in for the target USB webcams**, which were not connected (§9).
Index 0 delivered 1920×1080 and index 1 1280×720, both at a measured 30 fps. Runs lasted 20 s.

| Configuration | Steady inference rate / camera | Inference mean / p95 | Capture→observation mean / p95 | CPU | Peak RSS |
|---------------|-------------------------------:|---------------------|--------------------------------|----:|---------:|
| YOLOX-Tiny, cap 15 fps | 15.0 / 15.0 fps | 10.6 / 14.3 ms; 10.1 / 13.8 ms | 19.8 / 27.6 ms; 29.5 / 32.9 ms | 1.7 cores | 294 MB |
| YOLOX-Tiny, cap 30 fps | ~27 fps each (whole-run avg) | 12.4 / 15.7 ms; 11.3 / 15.4 ms | 18.2 / 27.5 ms; 16.3 / 29.5 ms | 3.2 cores | 302 MB |
| YOLOX-S, cap 15 fps | 14.5 / 14.6 fps | 28.4 / 39.0 ms; 29.1 / 41.6 ms | 40.9 / 66.9 ms; 49.3 / 65.9 ms | 2.7 cores | 321 MB |

The live UI (`hapticsight-vision live`, 450 frames, both real cameras) rendered at 27.6 fps
including start-up, against a 30 fps target, while both cameras captured at 30 fps and inference
ran at 15.1 fps per camera (9.0 ms).

**Prerecorded video, two simultaneous streams** (`vtest.avi` played as both cameras, paced at
its native 10 fps): 10.1 fps processed per stream, inference 10.4 / 12.5 ms and 12.4 / 13.7 ms
(mean / p95), capture→observation 13.2 / 21.7 ms and 24.0 / 28.0 ms, 1.2 CPU cores, 215 MB.

These are short runs on a fanless laptop; sustained thermal behaviour over long sessions has not
been measured. Reproduce them with `hapticsight-vision benchmark` (see README).

## 8. Camera-only advisory cue (experimental, off by default)

Objects seen only by a camera have no measured range, so by default they produce **no**
vibration (Milestone 1 behaviour, preserved). An optional experimental cue exists
(`--advisory`, or `haptics.advisory_cue_enabled = true`):

* a single 40 ms tick every 3 s (pattern slot 8), unlike every proximity pattern;
* at most one channel at a time: the most forward, confident (≥ 0.6) camera track that has
  been followed for ≥ 1 s and lies within ±45° of straight ahead;
* lowest priority: it never overrides a hazard or the degraded-coverage cue;
* it implies neither distance nor danger, and has not been user-tested. Slot 8 is documented
  as optional in PROTOCOL.md; the motor mapping is unchanged.

## 9. Limitations and unverified items

* **Target webcams not tested.** The two USB webcams (models, FOVs, and supported modes
  unknown) were not connected; real-camera results above come from the Mac's built-in camera and
  an iPhone Continuity Camera. Device enumeration order, USB bandwidth with two external
  cameras, and auto-exposure behaviour still need verification.
* Calibration defaults are **nominal** (60° HFOV ± 15°) until the procedure in §6 is run on the
  mounted cameras.
* COCO classes only; see §3 for what is not detected. Detection quality degrades in low light,
  with motion blur, at long range, and for partially visible objects.
* Tracking is per camera; an object moving from the front to the rear camera gets a new ID (the
  FOVs do not overlap).
* Fusion pairs camera bearings with ultrasonic echoes by angle only (see ARCHITECTURE D12);
  wrong pairings are possible when several objects share one beam (flagged `shared_range`).
* Pixel motion and box growth are not converted into closing speed. Monocular time-to-contact
  from looming is a possible future addition, but it would need separate validation.
* The live demo's ultrasonic values are **simulated** (magenta, labelled `SIMULATED` in the UI,
  session logs, and replay exports).

## 10. Privacy

Camera frames are processed in memory. Nothing is recorded unless you pass `--record` or use
`hapticsight-vision record`, and recordings stay under `recordings/` (git-ignored) on the local
disk. Nothing is uploaded anywhere. The only network access is the explicit `download-model`
command. Test fixtures are generated synthetically, and no third-party or personal video is
committed.
