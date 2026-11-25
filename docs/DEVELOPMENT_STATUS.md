# Development Status

_Last updated: 2026-11-24 · Milestone 1 merged to `main` (PR #1) · Milestone 2 on
`feature/dual-camera-computer-vision` (PR open, not merged)._

---

## Milestone 2: dual-camera computer vision: **complete in software; vest webcams not yet tested**

### Acceptance criteria

| # | Criterion | Status | Evidence / note |
|---|-----------|--------|-----------------|
| 1 | Both webcams captured concurrently | ✅ with stand-ins | Two real cameras (built-in FaceTime HD + iPhone Continuity Camera) captured concurrently at 30 fps each. **The two target USB webcams were not connected**, so their capture is unverified. |
| 2 | Real pretrained detector processes frames | ✅ | YOLOX-Tiny (Apache-2.0) via ONNX Runtime + CoreML on real camera frames and the OpenCV pedestrian sample video |
| 3 | Multiple objects tracked | ✅ | IoU tracker; synthetic regression tests; 21 confirmed tracks on a 300-frame pedestrian video |
| 4 | Correct body-relative bearings | ✅ | `tests/test_vision_geometry.py` (mounting, rear mirroring, wraparound, distortion, calibration fit); bearing error < 1° vs synthetic ground truth |
| 5 | Camera observations enter the shared pipeline | ✅ | `VisionSystem.camera_observations` → `HapticSightPipeline.process`; `tests/test_vision_integration.py` |
| 6 | Live detection + virtual haptics visualised | ✅ | `hapticsight-vision live` (run with both real cameras: ~28 fps UI) |
| 7 | Recorded video replayable for repeatable tests | ✅ | `hapticsight-vision replay`; determinism test on synthetic video |
| 8 | Fusion confidence corrected and tested | ✅ | `perception/fusion.py`, `tests/test_fusion_confidence.py` (commit `fce5689`) |
| 9 | Disconnects, stale data, missing range explicit | ✅ | camera states + reconnection tests; stale frames skipped; tracks age; range UNKNOWN |
| 10 | Tests, lint, types pass | ✅ | see below |
| 11 | Docs reflect the implementation | ✅ | README, COMPUTER_VISION.md, ARCHITECTURE (D16–D22), PROTOCOL (slot 8), HARDWARE_INTERFACE §4, THIRD_PARTY |
| 12 | Committed on the feature branch | ✅ | commit list below |

### Test and quality status (measured 2026-11-24, macOS 26.3.1, Python 3.11.18)

* `pytest`: **319 passed** with the YOLOX model installed locally. Without it (as in CI):
  **317 passed, 2 skipped** (the two real-model tests). No test needs a webcam or network.
* `ruff check .`, `ruff format --check .`: clean. `mypy`: no issues in 55 source files.
* CI (Ubuntu, Python 3.11/3.12) installs OpenCV-headless and ONNX Runtime; results are on the PR.

### Measured performance (MacBook Air M1 8 GB; full tables in COMPUTER_VISION.md §3, §7)

| Input | Result |
|-------|--------|
| Real cameras (index 0 at 1920×1080 + index 1 at 1280×720: the built-in FaceTime HD camera and an iPhone Continuity Camera, mapping not verified, standing in for the USB webcams), YOLOX-Tiny CoreML, 15 fps cap | capture 30 fps each; inference 15.0 fps each, 10.1–10.6 ms mean (p95 ≤ 14.3 ms); capture→observation 19.8 / 29.5 ms mean (p95 ≤ 32.9 ms); 1.7 CPU cores; 294 MB peak RSS |
| Same, inference cap 30 fps | ~27 fps per camera; capture→observation 16–18 ms mean; 3.2 CPU cores |
| Same, YOLOX-S | 14.5 fps per camera; inference ~29 ms; capture→observation 41–49 ms mean; 2.7 cores |
| Live UI, both real cameras | 27.6 fps UI incl. start-up (30 target) |
| Prerecorded video ×2 (vtest, 10 fps) | 10.1 fps per stream; capture→observation 13–24 ms mean; 1.2 cores; 215 MB |
| Detector only, prerecorded video | YOLOX-Tiny CoreML 11.2 ms mean; CPU ORT 30.0 ms; OpenCV DNN 25.8 ms |

These are 20 s runs; sustained thermal throttling on the fanless Air is **not measured**.

### Key decisions (full log: ARCHITECTURE.md D16–D22, COMPUTER_VISION.md)

* **Detector:** YOLOX-Tiny, Apache-2.0, ONNX Runtime with CoreML on macOS (`runtime=auto`), no
  PyTorch. Ultralytics (AGPL) and GPL/non-commercial YOLO variants were excluded for licensing.
* **Concurrency:** one capture thread per camera with a newest-frame slot (no queues), one
  rate-limited inference worker (15 fps per camera by default) serving both cameras.
* **Tracker:** greedy IoU with motion prediction and a centre-distance fallback; confirmed after
  2 hits; 0.6 s persistence; image-plane motion recorded but never treated as closing speed.
* **Geometry:** pinhole + optional radial distortion, mount bearing + yaw offset, mirrored
  support, explicit uncertainty budget; nominal 60° ± 15° HFOV until calibrated.
* **Fusion fix (PR #1 review):** fused confidence = min(camera, ultrasonic); camera detections
  below 0.5 confidence or older than 0.4 s are never fused; echoes are suppressed only by
  confident fusion; `HapticSightConfig` validates the invariants.
* **Advisory cue:** implemented as **experimental, off by default** (slot 8, ≤ 1 channel,
  lowest priority). Not user-tested.
* **Haptic rank table renumbered** to fit ADVISORY below DEGRADED (internal; wire slots
  unchanged except the new optional slot 8).
* **Simulated ultrasonic** values in demos are hand-controlled and labelled `SIMULATED`.

### Known limitations / open issues

1. **The target USB webcams have not been tested**: models, FOV, modes, USB bandwidth with two
   external cameras, and index order are unknown. Calibration has not been run on mounted cameras.
2. COCO detector only: walls, poles, posts, curbs, stairs, glass, holes, and overhanging
   obstacles are **not** detected (COMPUTER_VISION.md §3).
3. Uncalibrated FOV can shift edge bearings by 5–15°, enough to move warnings between motors.
4. Tracking is per camera; IDs change when an object passes from one camera's view to the other.
5. Camera↔echo association is angle-only; wrong pairings are possible with several objects in
   one beam (flagged `shared_range`).
6. No monocular range or time-to-contact; camera-only objects only produce the optional advisory
   cue.
7. Thermal behaviour in long sessions, low-light performance, and motion blur are untested.
8. Driver-reported FPS can be wrong (one device reported 1 fps while delivering 30); the
   software relies on measured FPS.
9. Milestone 1 limitations still apply (no firmware, simplified sensor models, provisional
   thresholds and patterns).

### Next steps: Milestone 3 (recommended): Arduino firmware and hardware link

1. **Firmware** (Arduino Uno) implementing PROTOCOL.md v1: sequential HC-SR04 ranging with a
   per-sensor timeout, `$US` at 10 Hz, `$ST` heartbeat ≤ 250 ms, `$HLO` at boot, `$HAP` with TTL
   ≤ 2000 ms, `$PAT` table (built-in defaults for slots 1–7; undefined slots play OFF), 74HC595
   output with `OE` held off at boot, and the ultrasonic-only FALLBACK mode on command expiry.
   Firmware unit tests: pinned wire examples from `tests/test_protocol.py`.
2. **Host serial transport:** `pyserial`-backed `ByteTransport` + `hapticsight-run` runtime:
   `ArduinoBridge.poll()` + `VisionSystem.camera_observations()` → `pipeline.process()` →
   `ArduinoBridge.send()` at 10–20 Hz on `MonotonicClock`. Add a "serial loopback" mode to the
   live app that replaces `ManualUltrasonic` with real `$US` data and shows the link state.
3. **Bench tests with real hardware:** HC-SR04 beam width/noise per sensor; end-to-end latency
   (echo → motor) measured with a logic analyser or high-speed video; verify fallback on USB
   unplug.
4. **Mount and calibrate the USB webcams** on the vest (COMPUTER_VISION.md §6), then record and
   replay real walks for regression tests (kept local).

## Milestone 1: software foundation and simulator: **merged (PR #1, `08f62d3`)**

Shared observation model, ultrasonic interpretation and range-rate, gating fusion,
deterministic risk engine, 8-channel timing-coded haptic encoder with hysteresis and
degraded-coverage cue, protocol v1 host side, and the Pygame simulator with 10 deterministic,
regression-tested scenarios. Measured then: 223 tests passing; simulator ~51–56 fps at a 60 cap.
Details: README, ARCHITECTURE.md D1–D15, PROTOCOL.md, HARDWARE_INTERFACE.md. Fusion confidence
handling from M1 was superseded by D21 in Milestone 2.

## Hand-off notes for the next agent

* Read `CLAUDE.md` (invariants 1–10), then ARCHITECTURE.md and COMPUTER_VISION.md.
* Python on this Mac: `/opt/homebrew/bin/python3.11`; venv at `.venv`; models in `models/`
  (git-ignored; reinstall with `hapticsight-vision download-model yolox_tiny`).
* Camera permission for the terminal/VS Code was granted on this Mac during M2. OpenCV indices
  seen: 0 = 1920×1080 device, 1 = 1280×720 device (the built-in FaceTime camera and an iPhone
  Continuity Camera; the exact mapping was not verified). USB webcams will add or shift indices.
* Fast behaviour checks without hardware:
  `hapticsight-vision live --front synthetic --rear synthetic --model synthetic`, and
  `hapticsight-sim --headless -s N`.
* Benchmark JSONs from this milestone are under `benchmarks/local/` (git-ignored, this machine).
* Never merge to `main` without the owner's approval.

## Commit history (Milestone 2)

```
fdd6ef6 Add vision dependencies, hapticsight-vision entry point, and ignore rules
fce5689 Fix fusion confidence: never let a weak camera borrow echo confidence
5bb2b24 Add experimental camera-only advisory cue (disabled by default)
b446514 Add vision core: YOLOX detector, bearing geometry, calibration, IoU tracker
6a352f5 Add concurrent dual-camera capture, inference worker, discovery, benchmarks
3d020da Add live vision app, recording/replay, and hapticsight-vision CLI
```
Followed by the synthetic-scene fix, documentation, and any CI fix commits (see `git log`).
