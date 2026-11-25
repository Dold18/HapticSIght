# Hardware Interface Contract

This document is the agreement between the **software** workstream (this repository) and the
**CAD / hardware** workstream that is building the physical vest. The software encodes these
assignments in [`hapticsight/core/layout.py`](../hapticsight/core/layout.py) and pins them with a
contract test (`tests/test_geometry_and_models.py::test_layout_contract_is_unchanged`).

> **Change control.** Do not change any numbering, bearing, or bit assignment below without
> (1) agreement from the hardware workstream, (2) updating this file, `layout.py`, the contract
> test, and [PROTOCOL.md](PROTOCOL.md) if wire order is affected, and (3) recording the decision in
> [DEVELOPMENT_STATUS.md](DEVELOPMENT_STATUS.md).

Status legend: **Agreed** = fixed contract. **Proposed** = software-side proposal, still to be confirmed
by the hardware build. **TBD** = not yet decided.

---

## 1. Coordinate system (Agreed)

Body-relative, viewed from above, wearer at the origin, facing "up":

| Bearing | Direction |
|--------:|-----------|
| 0°      | directly forward |
| 90°     | directly right |
| 180°    | directly behind |
| 270°    | directly left |

* Angles increase **clockwise**. Canonical range is `[0°, 360°)`; every input is normalised
  (`-1°` → `359°`, `360°` → `0°`).
* Cartesian body frame (used by the simulator): `+x` = right, `+y` = forward, centimetres,
  so `bearing = atan2(x, y)`.
* **Distances are clearances**: the gap between the wearer's torso surface (where the sensors and
  motors are mounted) and the obstacle's nearest surface, in centimetres.

## 2. Vibration motors (Agreed)

Eight coin ERM motors at 45° intervals. Motor index = wire channel order = shift-register bit.

| Motor | Bearing | Position     | Owns bearings (half-open sector) |
|-------|--------:|--------------|----------------------------------|
| M0    | 0°      | front        | [337.5°, 22.5°)                  |
| M1    | 45°     | front-right  | [22.5°, 67.5°)                   |
| M2    | 90°     | right        | [67.5°, 112.5°)                  |
| M3    | 135°    | rear-right   | [112.5°, 157.5°)                 |
| M4    | 180°    | rear         | [157.5°, 202.5°)                 |
| M5    | 225°    | rear-left    | [202.5°, 247.5°)                 |
| M6    | 270°    | left         | [247.5°, 292.5°)                 |
| M7    | 315°    | front-left   | [292.5°, 337.5°)                 |

Sector boundaries belong to the **clockwise** neighbour (22.5° → M1, 337.5° → M0).

### Motor drive chain (Proposed)

| Shift register output | ULN2803A input → output | Motor |
|-----------------------|-------------------------|-------|
| 74HC595 QA (bit 0)    | IN1 → OUT1              | M0    |
| QB (bit 1)            | IN2 → OUT2              | M1    |
| QC (bit 2)            | IN3 → OUT3              | M2    |
| QD (bit 3)            | IN4 → OUT4              | M3    |
| QE (bit 4)            | IN5 → OUT5              | M4    |
| QF (bit 5)            | IN6 → OUT6              | M5    |
| QG (bit 6)            | IN7 → OUT7              | M6    |
| QH (bit 7)            | IN8 → OUT8              | M7    |

`PulseScheduler.output_byte()` already produces this byte (bit *k* = motor M*k*). Motors are
switched **fully on or off** only; there is no PWM intensity control in this design.

Still **TBD** by hardware: Arduino pin assignments for the 74HC595 (data/clock/latch, output
enable), flyback/decoupling details, motor supply current budget, and whether `OE` is wired so
firmware can force all motors off at boot.

## 3. Ultrasonic sensors (Agreed numbering)

Four HC-SR04 sensors, one per cardinal direction. Sensor index = order of range fields in the
`$US` protocol message.

| Sensor | Bearing | Position | Nearest motor |
|--------|--------:|----------|---------------|
| S0     | 0°      | front    | M0            |
| S1     | 90°     | right    | M2            |
| S2     | 180°    | rear     | M4            |
| S3     | 270°    | left     | M6            |

Provisional sensor model used by the software (configurable in `UltrasonicConfig`; to be
re-measured on the real vest):

| Parameter | Value | Note |
|-----------|-------|------|
| Beam half-angle | 15° | HC-SR04 nominal ~30° cone; real coverage depends on target size and material |
| Valid range | 2–400 cm | outside → error / no echo |
| Range uncertainty | ±2 cm | used conservatively by the risk engine |
| Update rate | 10 Hz per sensor | assumes sequential firing to avoid cross-talk (**Proposed**) |
| Stale after | 0.4 s | no reading for this long → sector coverage lost |

**Important limitation, by design:** four sensors give four *sectors* of range, not eight
directions. The software never invents range for the diagonal directions. With the default
15° beams there are blind zones around 45°, 135°, 225°, and 315°; the diagonal motors are
driven only when a camera supplies a bearing that is associated with an ultrasonic echo, when
bearing uncertainty genuinely spans a diagonal sector, or in the simulator's ground-truth mode.

Arduino pins for trigger/echo: **TBD** by hardware.

## 4. Cameras (mount bearings Agreed; optics unknown)

| Camera     | Mount bearing | Image right side faces | Optics |
|------------|--------------:|------------------------|--------|
| CAM_FRONT  | 0°            | wearer's right (→ ~30°) | **unknown**: measure via calibration |
| CAM_REAR   | 180°          | wearer's **left** (→ ~210°) | **unknown**: measure via calibration |

* The two USB webcams' models, resolutions, frame rates, and fields of view are not yet known
  and are **not hard-coded**. Software defaults to a *nominal* 60° HFOV with ±15° uncertainty
  until the calibration procedure in [COMPUTER_VISION.md §6](COMPUTER_VISION.md#6-calibration-procedure)
  is run on the mounted cameras. The two cameras may have different FOVs.
* Bearing = mount bearing + measured yaw offset + angle within the image (positive to the
  camera's right = clockwise). For the rear camera, the image's right side is the wearer's left.
  Mirrored webcams are supported (`mirrored = true`).
* **Mounting request for the hardware workstream:** keep each camera level (no roll) and aimed
  horizontally, record its mounting height and any deliberate yaw, and keep the mount rigid.
  Yaw errors shift every bearing one-for-one.
* Cameras connect to the MacBook over USB and produce **bearing-only** detections:
  `range_status = UNKNOWN`, `closing_speed = None`. Camera loss does not trigger the DEGRADED
  motor cue: cameras add bearing detail and never provide the only warning.
* The host's device-index → CAM_FRONT/CAM_REAR assignment is machine-specific
  (`config/cameras.local.toml`), because OpenCV index order is not guaranteed.

## 5. Software / hardware boundary

| Responsibility | MacBook (Python, this repo) | Arduino Uno (future firmware) |
|----------------|-----------------------------|-------------------------------|
| Fire HC-SR04s, measure echo time, convert to cm | — | ✔ |
| Report ranges (`$US`) and status (`$ST`) | parse & validate | ✔ |
| Camera capture, object detection, tracking, bearing | ✔ (Milestone 2) | — |
| Sensor fusion, risk assessment, haptic encoding | ✔ | — |
| Pattern table (timing) | defines, sends `$PAT` | stores (defaults built in) |
| Pulse timing of each motor | models it (`PulseScheduler`) | ✔ executes it |
| 74HC595 shifting, ULN2803A drive | — | ✔ |
| Command expiry (TTL) | sets TTL | ✔ enforces it |
| Ultrasonic-only fallback if host is lost | — | ✔ (see PROTOCOL.md §6) |

The host never drives motor timing directly; it sends *which pattern* each motor should play and
the firmware generates the pulses from its own `millis()` clock. This keeps timing independent of
USB latency and of host load.

## 6. Units and conventions on the wire

* Distances: integer centimetres.
* Times: integer milliseconds. Arduino timestamps are `millis()` modulo 2³²; the host uses its own
  monotonic receive time for staleness and only uses Arduino time for ordering/logging.
* Motor order: always M0 → M7. Sensor order: always S0 → S3.

See [PROTOCOL.md](PROTOCOL.md) for the message format.
