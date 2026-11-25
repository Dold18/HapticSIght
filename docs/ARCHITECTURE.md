# Architecture

## 1. Data flow

```
 ┌──────────────────── sources (swappable) ──────────────────────┐
 │ simulation/sensors.py       hardware/arduino.py (future HW)   │
 │ SimulatedUltrasonicArray    ArduinoBridge                     │──► UltrasonicReading
 │ simulation/manual_ultrasonic.ManualUltrasonic (SIMULATED)     │──► UltrasonicReading
 │ SimulatedCamera             vision/ VisionSystem (real cams)  │──► Observation (bearing only)
 └───────────────────────────────────────────────────────────────┘
                              │
                              ▼
 hapticsight/pipeline.py  HapticSightPipeline.process(now, readings, detections)
   (live hardware: hardware/runtime.py HardwareRuntime.step -> ArduinoBridge.send)
   ├─ perception/ultrasonic.py  UltrasonicFrontEnd  → sector Observations + SensorHealth
   │                            RangeRateEstimator  → closing speed (or None)
   ├─ perception/buffer.py      ObservationBuffer   → latest camera detections, expiring
   ├─ perception/fusion.py      fuse()              → FUSED / raw Observations
   ├─ risk/engine.py            RiskEngine.assess() → RiskReport (sorted RiskAssessments)
   └─ haptics/encoder.py        HapticEncoder       → HapticFrame (8 ChannelCommands)
                              │
                              ▼
 haptics/scheduler.py PulseScheduler  (host model of firmware timing → 8 on/off states)
 hardware/arduino.py  ArduinoBridge.send()  → "$HAP,…" line          (future hardware)
```

The simulator ([`simulation/engine.py`](../hapticsight/simulation/engine.py)) owns a world, the
simulated sensors, one `HapticSightPipeline`, and one `PulseScheduler`. The Pygame app
([`simulation/app.py`](../hapticsight/simulation/app.py)) only reads engine state and forwards
user input. Nothing in `core/`, `perception/`, `risk/`, `haptics/`, `protocol/`, or `pipeline.py`
imports from `simulation/` or `pygame`.

## 2. Package responsibilities

| Package | Responsibility | Depends on |
|---------|----------------|------------|
| `core/` | `Observation`, `UltrasonicReading`, `SensorHealth`; geometry; layout contract; clocks | stdlib |
| `perception/` | ultrasonic interpretation, range-rate, camera buffering, fusion | core |
| `risk/` | severity, motion, time-to-contact, staleness/confidence gating, prioritisation | core |
| `haptics/` | bearing→motor mapping, pulse patterns, encoder, scheduler | core, risk |
| `protocol/` | serial protocol v1 messages, codec, stream framing, link monitor | stdlib |
| `hardware/` | device adapter interfaces; host-side Arduino bridge | core, haptics, protocol |
| `vision/` | camera capture, YOLOX detection, tracking, bearing geometry, live app, replay, CLI | core, pipeline (numpy, OpenCV, ONNX Runtime) |
| `hardware/` (M3) | pyserial transport, ArduinoBridge state machine, virtual Uno, labelled inputs, HardwareRuntime, telemetry, `hapticsight-hw` | protocol, pipeline (pyserial) |
| `firmware/arduino_uno/` | Uno firmware: portable C++ core in `src/`, pin/ISR glue in the `.ino` | none (C++11, avr-libc) |
| `config.py` | aggregates per-layer config dataclasses; TOML overrides | all layer configs |
| `pipeline.py` | the one shared processing chain | perception, risk, haptics |
| `simulation/` | world, simulated sensors, scenarios, headless engine, CLI, Pygame app | everything above |

## 3. Key interfaces

```python
# Every source produces these (core/models.py)
Observation(id, source, bearing_deg, timestamp, confidence, range_status,
            distance_cm=None, distance_uncertainty_cm=None, bearing_uncertainty_deg=0,
            closing_speed_cm_s=None, sensor_id=None, metadata={})
UltrasonicReading(sensor_id, timestamp, status, distance_cm=None)

# Shared chain (pipeline.py)
HapticSightPipeline(config).process(now, ultrasonic_readings, camera_observations) -> PipelineResult
HapticSightPipeline(config).process_observations(now, observations, sensor_health) -> PipelineResult
PipelineResult(timestamp, observations, sensor_health, report: RiskReport, frame: HapticFrame)

# Device adapters (hardware/interfaces.py)
UltrasonicSource.poll() -> list[UltrasonicReading]
CameraDetector.poll()   -> list[Observation]     # range_status=UNKNOWN, closing_speed=None
HapticSink.send(frame: HapticFrame) -> None
ByteTransport.read_available() / write(bytes) / close()
```

A future hardware runtime is then a loop: `bridge.poll()` + `camera.poll()` →
`pipeline.process(clock.now(), …)` → `bridge.send(result.frame)` at 10–20 Hz.

## 4. Design decisions

| # | Decision | Rationale |
|---|----------|-----------|
| D1 | **Range is optional; status is explicit** (`VALID` / `NO_ECHO` / `INVALID` / `UNKNOWN`). | A monocular camera cannot measure range. "No echo", "sensor fault", and "no data" mean different things and must not be conflated. |
| D2 | **Ultrasonic observations sit at the sensor boresight with bearing uncertainty = beam half-angle.** | An HC-SR04 reports the nearest echo anywhere in its cone. Four sensors give four sectors; the software does not invent eight. |
| D3 | **Closing speed only from measured range history** (windowed least squares; `None` until ≥ 4 samples over ≥ 0.2 s; reset on jumps or lost echo). | "Approaching" must come from real evidence, never from visibility or apparent size. |
| D4 | **Conservative distance** (`distance − uncertainty`) for banding. | When in doubt, warn slightly earlier. |
| D5 | **Time-to-contact can raise severity, never lower it.** | Fast approaches deserve earlier warnings; slow ones should not hide proximity. |
| D6 | **Stale or low-confidence observations stay visible in the report but are rated NONE.** | Explainability without acting on bad data. |
| D7 | **Lost sensor coverage is signalled (DEGRADED pattern), not silent.** | Missing data must never feel like "all clear". |
| D8 | **Conflict resolution = highest rank per motor** (severity first, then approach cue). | Simple, deterministic, explainable; no weighted scoring. |
| D9 | **Release hysteresis instead of debouncing.** Escalation is immediate; de-escalation waits `release_hold_s`. | Removes flicker from intermittent echoes without ever delaying a more urgent warning. |
| D10 | **Urgency encoded in timing** (rate, duty, double-tap for approach). | Hardware is on/off ERM via shift register: no amplitude control. |
| D11 | **Host sends pattern slots, firmware generates pulses.** | Keeps timing independent of USB jitter; fits Uno SRAM; one hex digit per motor. |
| D12 | **Fusion = simple gating, conservative on ambiguity.** A shared echo goes to every confident camera detection in the gate (flagged `shared_range`); stale echoes and stale or weak camera detections are never fused (see D21). | Explainable foundation; avoids attaching wrong-age data. |
| D13 | **Fixed-timestep simulation on a `ManualClock`; all logic takes `now` explicitly.** | Determinism, frame-rate independence, trivially testable. Real runs use `time.monotonic()`. |
| D14 | **Simulator has a ground-truth mode.** | Lets users see what perfect perception would do versus what four sensors and two cameras can actually support. |
| D15 | **Deterministic rules, no ML in the risk engine (v1).** | Required by the milestone brief; easy to test and reason about. |
| D16 | **Vision emits only the shared `Observation`** (CAMERA, range UNKNOWN, no closing speed); there is no parallel obstacle model. | Camera data goes through the same fusion → risk → haptics path as simulated cameras. |
| D17 | **YOLOX-Tiny (Apache-2.0) via ONNX Runtime + CoreML**, no PyTorch. | Best measured quality/latency trade-off on the M1 with a permissive licence; small footprint (COMPUTER_VISION.md §3). |
| D18 | **Newest-frame slots, one inference worker, rate cap per camera.** | Bounded latency (frames dropped, never queued); one model in memory; predictable CPU/thermal load. |
| D19 | **Greedy IoU tracker with motion prediction**; image-plane motion kept separate from metric motion. | Adequate at ≥ 10 fps, deterministic, dependency-free; box growth is not a range rate. |
| D20 | **Explicit per-detection bearing-uncertainty budget** (yaw, FOV, distortion, box). | Calibration quality flows honestly into motor spreading; uncalibrated cameras are visibly uncertain. |
| D21 | **Fused confidence = min(camera, ultrasonic)**; weak or stale camera detections are never fused; echoes are suppressed only by confident fusion (validated config invariant). | Fixes the PR #1 finding; a bad association can never remove an independent ultrasonic warning. |
| D22 | **Camera-only advisory cue is experimental and off by default** (slot 8, lowest rank, ≤ 1 channel). | Explores unranged awareness without implying distance or danger. |
| D23 | **Firmware = portable core + thin glue.** All behaviour in hardware-independent C++11 (`src/`); the `.ino` only does pins, ISRs, Serial, shift register. | Unit-testable natively with injected clocks/HAL; the same code runs in the virtual Uno. |
| D24 | **Staggered ultrasonic slots of 50 ms (5 Hz per sensor)**, echo edges time-stamped by pin-change interrupts, bounded state machine. | Cross-talk resistance over an arbitrary 10 Hz target; never blocks serial or motor timing. Host staleness/rate windows adjusted to 5 Hz. |
| D25 | **Host is silent until a version-checked `$HLO`**; DTR reset if none; input flushed *before* (never after) a reset. | Uno auto-reset + bootloader; telemetry alone does not prove compatibility; avoids losing the fresh `$HLO`. |
| D26 | **Firmware commits a command only after the entire line validates**; malformed lines never refresh the watchdog. | No partial updates; a garbled stream cannot keep stale commands alive. |
| D27 | **Fallback: one DEGRADED burst, then cardinal-motor warnings; missing/faulty/stale sensor = DEGRADED.** | Absence of data is never rendered as "clear"; wearer is told the system degraded. |
| D28 | **Source kind fixed by class** (real Arduino / virtual Arduino / manual) and propagated to UI colours, labels, logs. | A demo cannot present simulated distances as real measurements. |
| D29 | **Virtual Uno over a pty** runs the real firmware logic against the real pyserial path. | End-to-end host↔firmware testing in CI and demos before hardware exists (sensors simulated, clearly labelled). |

## 5. Determinism and timing

* `SimulationEngine` advances in fixed 20 ms ticks (50 Hz). `advance(real_dt)` accumulates
  wall time and runs whole ticks (capped at 10 per call), so render rate never changes results.
* Simulated ultrasonic sensors sample at 10 Hz and cameras at 15 Hz on the simulation clock.
* All randomness comes from one `random.Random` seeded from the scenario (or `--seed`).
* `PulseScheduler` computes motor state from elapsed time since a pattern started; the GUI only
  samples it.

## 6. Testing strategy

* Unit tests per layer (geometry, models, perception, risk, haptics, protocol, config).
* Pipeline integration tests with hand-made readings.
* Scenario regression tests run every predefined scenario headless and assert its documented
  outcome (`tests/test_scenarios.py`).
* Determinism tests (same seed ⇒ identical frame sequences; frame-rate independence).
* GUI smoke tests drive every control through the Pygame app with the SDL dummy video driver.
