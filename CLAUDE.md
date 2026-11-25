# CLAUDE.md: HapticSight agent guide

HapticSight is a wearable obstacle-awareness prototype: 4 ultrasonic sensors + 2 webcams →
Python pipeline on a MacBook → 8 on/off ERM vibration motors via an Arduino Uno. This repo is the
software: shared pipeline + simulator (Milestone 1), dual-camera computer vision
(Milestone 2, `hapticsight/vision/`), and Arduino Uno firmware + serial runtime (Milestone 3,
`firmware/arduino_uno/`, `hapticsight/hardware/`). **Read [docs/DEVELOPMENT_STATUS.md](docs/DEVELOPMENT_STATUS.md) first**; it records what
is done, what is pending, and why.

## Commands

```bash
python3.11 -m venv .venv && source .venv/bin/activate && pip install -e ".[dev]"
pytest                                   # all tests (GUI tests use SDL dummy driver)
ruff check . && ruff format --check .    # lint + format check (ruff format . to fix)
mypy                                     # type check (strict-ish, see pyproject.toml)
hapticsight-sim [-s N]                   # interactive simulator (scenario N, 0 = sandbox)
hapticsight-sim --headless -s N          # print a scenario timeline without a window
hapticsight-sim -s N --screenshot x.png --at T   # offscreen render (set SDL_VIDEODRIVER=dummy)
hapticsight-sim -s N --frames 300        # real window for 300 frames, prints achieved FPS
hapticsight-vision download-model yolox_tiny        # one-time, explicit network use
hapticsight-vision cameras                          # probe cameras / permission hints
hapticsight-vision live --front 0 --rear 1 --ultrasonic "S0=90"
hapticsight-vision live --front synthetic --rear synthetic --model synthetic   # no camera/model
hapticsight-vision replay --front VIDEO --export out.jsonl                     # deterministic
hapticsight-vision benchmark --system --front 0 --rear 1 --seconds 20
make -C firmware/arduino_uno test                   # native firmware unit tests (ASan/UBSan)
arduino-cli compile --fqbn arduino:avr:uno firmware/arduino_uno   # Uno build + memory report
hapticsight-hw link-loss --port virtual             # host <-> virtual Uno (real firmware logic)
hapticsight-hw run --port virtual --front off --rear off --headless --seconds 5
```

All four checks (pytest, ruff check, ruff format --check, mypy) must pass before committing.

## Layout

```
hapticsight/core/        models (Observation, UltrasonicReading, SensorHealth), geometry, layout contract, clocks
hapticsight/perception/  UltrasonicFrontEnd, RangeRateEstimator, ObservationBuffer, fuse()
hapticsight/risk/        RiskEngine, Severity, Motion, RiskReport
hapticsight/haptics/     motor mapping, PulsePattern table, HapticEncoder, PulseScheduler
hapticsight/protocol/    serial protocol v1 codec, StreamDecoder, LinkMonitor
hapticsight/hardware/    ArduinoBridge (handshake/reconnect state machine), serial_transport (pyserial),
                         virtual_arduino (fw_sim on a pty), inputs (REAL/SIMULATED sources),
                         runtime (HardwareRuntime), telemetry (JSONL), cli (hapticsight-hw)
firmware/arduino_uno/    Uno firmware: .ino = pin/ISR glue only; src/ = portable C++11 core;
                         test/ = native tests + fw_sim (virtual Uno); Makefile
hapticsight/pipeline.py  HapticSightPipeline, the ONE shared processing chain
hapticsight/config.py    HapticSightConfig aggregate + TOML loader (config/example.toml)
hapticsight/simulation/  world, simulated sensors, scenarios, engine (headless), cli, app (pygame),
                         manual_ultrasonic (SIMULATED ranges for live demos)
hapticsight/vision/      camera (threaded capture), detector + models (YOLOX ONNX), geometry
                         (pixel->bearing, calibration), tracker, observations, processor
                         (VisionSystem), recording (record/replay), live_app, cli, synthetic
tests/                   one file per layer + test_scenarios.py (regressions) + test_app_smoke.py
docs/                    ARCHITECTURE, HARDWARE_INTERFACE, PROTOCOL, COMPUTER_VISION, THIRD_PARTY,
                         DEVELOPMENT_STATUS
models/, recordings/     git-ignored: model weights and local camera recordings (never commit)
```

## Invariants (do not break)

1. **Layering.** Only `simulation/app.py` and `vision/live_app.py` import pygame; numpy/OpenCV/
   ONNX Runtime are imported only inside `vision/`. Nothing outside `simulation/` imports
   from `simulation/`. Perception/risk/haptics logic lives in the shared packages and is called via
   `HapticSightPipeline`; the simulator must not re-implement or bypass it (ground-truth mode
   bypasses *perception only*, via `process_observations`).
2. **Hardware contract.** Motor M0–M7 at 0°, 45°, … 315°; sensors S0–S3 at 0/90/180/270°;
   0° forward, clockwise. Pinned by `test_layout_contract_is_unchanged`. Changing it requires
   coordination with the hardware workstream: see docs/HARDWARE_INTERFACE.md change control.
3. **No fabricated data.** Never assign a distance to an observation without a measured range;
   never mark an object approaching without a measured closing speed; never treat missing data as
   clear (stale sensors → DEGRADED cue).
4. **Time.** All logic takes `now` (monotonic seconds) explicitly. Never call `time.time()` in
   logic; real runs use `MonotonicClock`, simulation and tests use `ManualClock`. Haptic timing
   must not depend on frame rate.
5. **Determinism.** Simulation randomness only through the engine's seeded `random.Random`.
   Scenario outcomes are asserted in `tests/test_scenarios.py`; if you change behaviour, update the
   scenario's `expected` text, the test, and the README table together.
6. **Protocol.** Lines ≤ 64 bytes, integer fields, XOR checksum. Changing message layout requires
   bumping `PROTOCOL_VERSION` and updating docs/PROTOCOL.md and pinned examples in tests.
7. **Honesty.** The Arduino firmware is not implemented and nothing has been validated on the vest.
   Docs must keep implemented vs planned clearly separated, and benchmarks must say whether they
   came from real cameras (which ones), prerecorded video, or synthetic input.
8. **Vision outputs only shared `Observation`s**: `source=CAMERA`, `range_status=UNKNOWN`,
   `distance_cm=None`, `closing_speed_cm_s=None`. Never turn pixel motion or box growth into a
   metric closing speed or distance. Never fuse weak or stale camera detections (see D21).
9. **Simulated ultrasonic data is labelled `SIMULATED`** everywhere (UI, logs, exports).
10. **Privacy / repo size.** Never commit model files, camera recordings, or third-party video.
    Tests use fakes and synthetic video only; CI must not need a webcam or a model download.
11. **Firmware.** `src/` stays hardware-independent C++11 (no Arduino headers, no `String`, no
    heap, no STL) so it compiles natively for tests; only `arduino_uno.ino` touches pins/ISRs.
    Keep the parser in lock-step with `hapticsight/protocol/codec.py` (parity tests enforce it)
    and the built-in pattern table equal to `DEFAULT_PATTERNS`. Keep global SRAM <= 75 %.
12. **Hardware honesty.** Pins in `hs_config.h` are PROVISIONAL; electrical parameters are
    unverified. Never claim sensors/motors were tested unless they were physically measured.
    Ultrasonic source kind (real / virtual / manual) comes from the source class, never from
    user text; the host sends nothing to the Arduino before a valid `$HLO`.
13. **Motor safety.** No command may drive motors by default; diagnostic motor output needs
    explicit enablement, one motor at a time, with a short timeout.

## Conventions

* Python 3.11+, full type annotations, `from __future__ import annotations`.
* Frozen `slots=True` dataclasses for values; per-layer `*Config` dataclasses with validation in
  `__post_init__`; enums are `StrEnum` / `IntEnum`.
* Units in names: `_cm`, `_deg`, `_s`, `_ms`, `_cm_s`. Bearings normalised with
  `normalize_bearing`; compare with `angular_distance` (never raw subtraction).
* Prefer explicit, explainable rules over weighted scores; add a human-readable reason string to
  `RiskAssessment.reasons` for any new rule.
* Small focused tests; use `tests/conftest.py::make_obs` for observations.
* Line length 100 (ruff).

## Agent workflow

* Work on a feature branch per milestone (M1 merged via PR #1; M2 is
  `feature/dual-camera-computer-vision`).
  **Never merge into `main`, force-push, or rewrite history without the owner's explicit approval.**
* Commit logically separated changes with descriptive messages.
* Update `docs/DEVELOPMENT_STATUS.md` (status, decisions, test results, next steps) at the end of
  every work session so another agent can take over without repeating work.
* Do not claim hardware validation, performance numbers, or test results you have not measured.
