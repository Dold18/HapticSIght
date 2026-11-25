# Development Status

_Last updated: 2026-11-24 · Milestones 1 (PR #1) and 2 (PR #2) merged to `main` · Milestone 3 on
`feature/arduino-firmware-serial-integration` (PR open, not merged)._

---

## Milestone 3: Arduino firmware and end-to-end serial integration: **complete in software; no physical hardware tested**

**What "complete in software" means here:** the firmware compiles for the Uno, its logic is
unit-tested natively, and it runs end-to-end against the Python host through the real pyserial
stack via the *virtual Uno* (the same firmware code compiled natively, with simulated sensors, on
a pseudo-terminal). **No Arduino, HC-SR04, 74HC595, ULN2803A, or motor was connected during this
milestone.** No real sensor, motor, or electrical behaviour has been observed.

### Acceptance criteria

| # | Criterion | Status | Evidence / note |
|---|-----------|--------|-----------------|
| 1 | Uno firmware builds | ✅ | `arduino-cli compile --fqbn arduino:avr:uno`: 9,302 B flash (28 %), 884 B SRAM (43 %); CI job |
| 2 | All protocol v1 messages supported | ✅ | HLO/US/ST sent; HAP/PAT parsed with full validation; parity with the Python codec on 1,200+ fuzzed lines |
| 3 | Four ultrasonic sensors scheduled and reported | ✅ (logic) | staggered 50 ms slots, interrupt-timed echoes; 5 Hz per sensor calculated and observed in the virtual Uno (4.9–5.1 Hz). **Not measured on real sensors** |
| 4 | Eight motor patterns individually controllable | ✅ (logic) | scheduler matches Python timing at every ms over 2 s; `motor-test` drives one motor. **No real motor driven** |
| 5 | Command expiry and ultrasonic-only fallback | ✅ (logic) | native tests + virtual Uno: FALLBACK ~TTL after the last command (measured 503 ms at TTL 500 ms), recovery on the next HAP |
| 6 | Real pyserial transport | ✅ | `PySerialTransport`; exercised over a pty against the virtual Uno. **Not yet opened on a real Arduino port** |
| 7 | Connection, version negotiation, reconnection | ✅ | `ArduinoBridge` state machine; unit + pty integration tests (handshake, DTR retry, incompatible version, link loss, board death, reconnect) |
| 8 | Real ultrasonic observations enter the shared pipeline | ✅ (path) | `HardwareRuntime` → `HapticSightPipeline`; tested with virtual-Uno data (labelled simulated) |
| 9 | Haptic decisions transmitted to the Arduino | ✅ (path) | HAP scheduling, acknowledgements via `$ST.last_hap_seq` |
| 10 | Diagnostics for serial, sensors, motors, fallback | ✅ | `hapticsight-hw ports / monitor / ultrasonic / motor-test / link-loss / run / virtual-arduino` |
| 11 | Automated tests pass | ✅ | see below |
| 12 | Docs explain status and remaining validation | ✅ | FIRMWARE.md, HARDWARE_BRINGUP.md, PROTOCOL.md, HARDWARE_INTERFACE.md §3a, README |
| 13 | Committed and pushed on the feature branch | ✅ | commit list below; PR opened |

### Test and quality status (2026-11-24, macOS 26.3.1, Python 3.11.18, Apple clang)

* `pytest`: **352 passed** locally with the YOLOX model installed; **350 passed, 2 skipped**
  without it (the CI configuration). New in M3: native firmware tests and parity
  (`test_firmware_native.py`), bridge state machine (`test_arduino_bridge.py`), and host ↔
  virtual Uno over a pty (`test_hardware_integration.py`; 3/3 consecutive runs green).
* Native firmware unit tests: **312 checks, 0 failures** (`make -C firmware/arduino_uno test`,
  with AddressSanitizer + UBSan).
* `ruff check .`, `ruff format --check .`: clean. `mypy`: no issues in 61 source files.
* CI: Python job (3.11/3.12, includes native and pty tests) + firmware job (native tests, Uno
  compile, SRAM ≤ 75 % gate). Results are on the PR.

### Key decisions (full log: ARCHITECTURE.md D23–D29, FIRMWARE.md)

* **Firmware structure:** portable C++11 core (`src/`, no Arduino headers, no heap/`String`/STL)
  plus a thin `.ino` for pins, ISRs, Serial, and the 74HC595. The same core runs in tests and in
  the virtual Uno.
* **Ultrasonic rate: 5 Hz per sensor, not 10 Hz.** Four sequential sensors in 50 ms slots (25 ms
  maximum echo + quiet time) favour cross-talk resistance. Echo edges are time-stamped by
  pin-change interrupts; nothing blocks. Host defaults were adjusted: ultrasonic stale 0.6 s,
  range-rate window 1.0 s, fusion echo age 0.45 s, risk stale 0.6 s, link timeout 0.75 s,
  simulator 5 Hz. All scenario outcomes are unchanged.
* **Handshake:** the host is silent until a valid `$HLO`, discards earlier telemetry, pulses DTR
  if no `$HLO` arrives, and flushes input *before* the pulse. That ordering was a bug found during
  this milestone (flushing after the reset raced with the fresh `$HLO`); it is fixed and has a
  regression test.
* **Fallback:** entry cue = one DEGRADED burst (≈310 ms) on all motors (configurable to cardinal
  motors only), then S0/S1/S2/S3 → M0/M2/M4/M6 with host bands and hysteresis. Error, missing, or
  stale readings → DEGRADED.
* **Provenance labels:** ultrasonic source kind (REAL / SIMULATED virtual / SIMULATED manual /
  DISCONNECTED) is fixed by class and shown in UI colour, range annotations, and every log record.
* **Pins (PROVISIONAL):** D2–D9 TRIG/ECHO pairs, D10/D11/D12 SER/SRCLK/RCLK, A0 = OE (active low,
  external pull-up required), D13 = status LED. Verified against the ATmega328P (PCINT on every
  echo pin, no Timer0 conflict, D13 avoided for the shift register because the bootloader
  blinks it). **Not compatible with the Uno R4.**

### Hardware / electrical verification blockers (owner + electronics workstream)

1. **No hardware yet.** Confirm the board revision is Uno R3 / ATmega328P (R4 would need a new
   hardware layer).
2. **Confirm the pin map** (HARDWARE_INTERFACE.md §3a) with CAD/electronics, including the OE
   pull-up and SRCLR tie-off.
3. **Motor electrical parameters unverified:** motor specs, voltage across a running motor
   (the ULN2803A drops about 0.7–1.2 V), start/run current for 1 and 8 motors, ULN2803A
   dissipation, polarity and order, buck converter regulation and headroom from the 5 V USB
   input. See the HARDWARE_BRINGUP.md §1 table.
4. **8-motor fallback cue** may exceed supply capacity: verify, or set `HS_FB_CUE_ALL_MOTORS 0`.
5. **Real HC-SR04 behaviour:** measured rate, accuracy, cross-talk with all four mounted, and
   behaviour of the specific modules (some clones hold echo high longer).
6. **Open owner decision:** whether the vest should start ultrasonic-only warnings when powered
   without a host (`HS_BOOT_TO_FALLBACK_MS`, off by default per protocol v1).
7. No project licence has been chosen; note that distributing compiled firmware links the
   LGPL-2.1 Arduino core (THIRD_PARTY.md).

### Next steps: Milestone 4 (recommended): hardware bring-up and calibration

1. Execute HARDWARE_BRINGUP.md steps 1–9 as components arrive; record results in its §5 table
   and here.
2. Tune from measurements: `HS_US_SLOT_MS` (cross-talk), `UltrasonicConfig` (beam width, noise),
   pattern timings (perceptibility on the real motors and supply).
3. Measure end-to-end latency (echo → motor) with a logic analyser or high-speed video.
4. Mount and calibrate the USB webcams on the vest (COMPUTER_VISION.md §6); record local walks
   for replay regression.
5. Decide the boot-without-host behaviour and the fallback cue current budget.

## Milestone 2: dual-camera computer vision: **merged (PR #2, `894aac1`)**

YOLOX-Tiny (Apache-2.0, ONNX Runtime + CoreML) on two concurrently captured cameras, IoU
tracking, calibrated pixel→bearing with explicit uncertainty, live diagnostic UI,
recording/replay, the fusion-confidence fix (D21), and an experimental advisory cue (off by
default). Measured on the M1 with the built-in camera + an iPhone Continuity Camera standing in
for the USB webcams: 15 fps inference per camera at ~10 ms, 20–30 ms capture→observation,
1.7 cores, < 300 MB. Details: COMPUTER_VISION.md, ARCHITECTURE.md D16–D22. Still open from M2:
vest webcams untested and uncalibrated; the COCO detector misses walls, poles, curbs, stairs, and
glass; per-camera tracking only; long-session thermal behaviour untested.

## Milestone 1: software foundation and simulator: **merged (PR #1, `08f62d3`)**

Shared observation model, ultrasonic interpretation and range-rate, gating fusion,
deterministic risk engine, 8-channel timing-coded haptic encoder with hysteresis and
degraded-coverage cue, protocol v1 host side, and the Pygame simulator with 10 deterministic,
regression-tested scenarios. Details: README, ARCHITECTURE.md D1–D15, PROTOCOL.md.

## Hand-off notes for the next agent (Claude Code or Codex)

* Read `CLAUDE.md` (invariants 1–13), then ARCHITECTURE.md, FIRMWARE.md, and
  HARDWARE_BRINGUP.md.
* Python on this Mac: `/opt/homebrew/bin/python3.11`; venv at `.venv`; models in `models/`
  (git-ignored). `arduino-cli` 1.6.0 with `arduino:avr` 1.8.8 is installed via Homebrew
  (avr-gcc 7.3.0).
* Without hardware: `hapticsight-hw link-loss --port virtual`,
  `hapticsight-hw run --port virtual --front synthetic --rear synthetic --model synthetic`.
  The virtual Uno binary is built automatically into `build/firmware-sim/` (git-ignored) from
  `firmware/arduino_uno/src` whenever the sources are newer.
* Changing the protocol: update `hapticsight/protocol/*` **and** `firmware/arduino_uno/src/hs_protocol.*`;
  the parity test fails otherwise. Changing default patterns: update both `DEFAULT_PATTERNS` and
  `hs_patterns.cpp` (tested).
* Camera permission was granted to VS Code/terminal on this Mac in M2; no USB webcams or Arduino
  were attached during M3 (`/dev/cu.*` showed only Bluetooth and debug ports).
* Never merge to `main` without the owner's approval.

## Commit history (Milestone 3)

```
d819f9b Add Arduino Uno firmware implementing serial protocol v1
9bba923 Align host timing assumptions with the firmware's 5 Hz sensor rate
4e56e41 Add pyserial transport and handshake-based Arduino bridge state machine
ff6f24a Add virtual Arduino and firmware/host parity tests
f897ca4 Add unified hardware runtime, hapticsight-hw diagnostics, and telemetry
```
Followed by the UI layout, CI, and documentation commits (see `git log`).
