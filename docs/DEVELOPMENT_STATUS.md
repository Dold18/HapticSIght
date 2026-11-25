# Development Status

_Last updated: 2026-11-24 · Branch: `feature/software-foundation-simulator` · Not merged to `main`._

## Milestone 1: software foundation and interactive simulator: **complete (simulation only)**

The repository was empty when Milestone 1 started (no commits on `main`, empty GitHub remote).
Everything below was built on the feature branch.

### Acceptance criteria

| # | Criterion | Status | Evidence |
|---|-----------|--------|----------|
| 1 | Installs and launches on macOS | ✅ | `pip install -e ".[dev]"` on Python 3.11.18, macOS (Apple Silicon); real window opened via `hapticsight-sim -s 7 --frames 300` |
| 2 | Simulator renders top-down wearer + 8 motor channels | ✅ | `simulation/app.py`; screenshots in `docs/images/` |
| 3 | Obstacles can be manipulated and simulated | ✅ | click / drag / arrows / velocity keys; `test_app_smoke.py` |
| 4 | Risk assessments update as conditions change | ✅ | live hazard list; `test_interactive_edits_while_paused_update_assessment` |
| 5 | Correct motors activate with appropriate pulse patterns | ✅ | `test_haptics.py`, `test_scenarios.py` |
| 6 | Multiple obstacles and sensor dropouts supported | ✅ | scenarios 4, 5, 8, 10 + tests |
| 7 | Shared logic independent of the GUI | ✅ | only `simulation/app.py` imports pygame; headless engine and tests |
| 8 | Automated tests pass | ✅ | 223 passed (see below) |
| 9 | Protocol contract documented and tested | ✅ | `docs/PROTOCOL.md`, `tests/test_protocol.py` |
| 10 | README and status reflect the implementation | ✅ | this file, README |
| 11 | Changes committed to the feature branch | ✅ | see commit list below |

### Test and quality status (measured 2026-11-24, macOS, Python 3.11.18)

* `pytest`: **223 passed** in ~1 s (unit, integration, 10 scenario regressions, determinism,
  GUI smoke with SDL dummy driver).
* `ruff check .` and `ruff format --check .`: clean.
* `mypy` (`disallow_untyped_defs`, `check_untyped_defs`): no issues in 35 source files.
* GitHub Actions workflow (`.github/workflows/ci.yml`, Ubuntu, Python 3.11/3.12) is committed
  but **has not yet run on GitHub** (see the push status in the hand-off section below).

### Measured performance (MacBook, not a benchmark)

* Real window, 60 FPS cap, 300 frames: **~51–56 FPS achieved**; ~2 s CPU per 6–8 s wall
  (~25–30 % of one core); max RSS ~140 MB.
* Offscreen per-frame cost (dummy driver, scenario 10): draw ≈ 2.4 ms, simulation ≈ 0.05 ms.
  The gap to 60 FPS comes from the frame limiter's sleep granularity, not rendering cost.

## Key decisions (summary; full log in [ARCHITECTURE.md §4](ARCHITECTURE.md#4-design-decisions))

* Observation model with explicit `RangeStatus`; cameras never carry range.
* Ultrasonic = 4 sectors with beam-width bearing uncertainty; no invented diagonal precision.
* Closing speed only from range history (least squares); `None` until evidence exists.
* Conservative distance (`d − uncertainty`); TTC can only escalate severity.
* Stale (> 0.5 s) / low-confidence (< 0.35) observations shown but never warn; lost sensor
  coverage drives a DEGRADED pattern rather than silence.
* Per-motor conflict resolution by rank; release hysteresis (0.35 s) instead of debouncing.
* Timing-coded patterns for on/off ERMs; host sends pattern slots, firmware will generate pulses.
* Fusion never uses stale echoes (bug found via scenario 8 and fixed in `c029e71`).
* CRITICAL+APPROACH pattern revised so every approach variant has duty ≥ its plain pattern.
* No license chosen yet; left to the project owner.

## Known limitations / open issues

1. **No hardware validation.** Sensor models (beam, noise, echo probability) are simplified and
   provisional; there is no multipath, cross-talk, or obstacle-to-obstacle occlusion.
2. **Arduino firmware not implemented.** Protocol v1 is specified and the host side tested only
   against an in-memory fake transport.
3. **Camera-only objects produce no vibration** (no range ⇒ no band). This is deliberate, but a
   soft or absorbent obstacle seen only by a camera is not signalled (scenario 10). A future option
   is a distinct "unranged presence" cue or monocular looming-based time-to-contact.
4. **Diagonal motors** are rarely driven in sensor mode (blind zones with 15° beams); they need
   camera fusion. Real beam width must be measured.
5. **Fusion** is a bearing gate only; with multiple camera objects in one beam the echo range is
   shared conservatively (flagged `shared_range`).
6. **Approach cue during crossing:** range rate is genuinely negative while a crossing object
   nears its closest point, so the approach cue briefly appears (scenario 7). This is
   physically correct for range rate but could be refined with lateral-motion tracking.
7. **Wearer is stationary** in the simulator; wearer motion is not modelled explicitly.
8. **Pattern timings and thresholds** are provisional and need perceptual testing.
9. GUI frame rate ~51–56 FPS at a 60 cap (SDL sleep granularity); acceptable, not optimised.

## Next steps: Milestone 2 (proposed): hardware link and firmware

1. Arduino Uno firmware implementing PROTOCOL.md: sequential HC-SR04 ranging, `$US`/`$ST`/`$HLO`,
   `$HAP` with TTL, `$PAT` table (defaults built in), fallback mode, 74HC595 output.
2. `pyserial`-backed `ByteTransport`; a `hapticsight-run` hardware-in-the-loop runtime
   (poll → `pipeline.process` → `bridge.send` at 10–20 Hz, `MonotonicClock`).
3. Serial-replay tooling: record real `$US` streams and replay them through the simulator/pipeline.
4. Bench characterisation of real HC-SR04 beam width/noise; update `UltrasonicConfig` defaults.
5. Simulator option to emulate the serial link (encode → decode round trip) and firmware
   fallback, so link loss can be demonstrated before hardware exists.

Later: camera capture + CPU-friendly detector (`CameraDetector`), calibrated bearing, improved
association/tracking, pattern user studies.

## Hand-off notes for the next agent

* Start with `CLAUDE.md` (invariants) and `docs/ARCHITECTURE.md` (interfaces).
* `hapticsight-sim --headless -s N` prints a timeline; it is the fastest way to inspect behaviour.
* Scenario expectations live in three places that must stay in sync: `scenarios.py` (`expected`),
  `tests/test_scenarios.py`, and the README table.
* Python on this Mac: `/opt/homebrew/bin/python3.11` (system `python3` is 3.9 and unsupported).
* The branch has not been merged into `main`; the owner must authorise any merge.

## Commit history (Milestone 1)

```
46f6649 Add project tooling: pyproject, ruff, mypy, pytest config
01d20ba Add core models, body-frame geometry, layout contract, and clocks
7102ea8 Add perception: ultrasonic front end, range-rate estimation, fusion
9458dbb Add deterministic risk engine with explainable reasons
f264895 Add haptic encoding: motor mapping, pulse patterns, encoder, scheduler
193a984 Add serial protocol v1 codec, link monitor, and Arduino bridge
a31515d Add shared processing pipeline and TOML configuration
c029e71 Do not fuse camera detections with stale ultrasonic echoes
caea38e Add simulation engine, simulated sensors, and ten regression scenarios
9e70b55 Add Pygame simulator smoke tests (SDL dummy driver)
```
Followed by the UI polish, pattern fix, and documentation/CI commits (see `git log`).
