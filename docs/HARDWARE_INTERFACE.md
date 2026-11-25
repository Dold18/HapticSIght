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

Arduino pins for the 74HC595 are **Proposed** in §3a. Still **TBD** by electronics: flyback and
decoupling details, the motor supply and its current budget, and the confirmed `OE` pull-up
(see [HARDWARE_BRINGUP.md](HARDWARE_BRINGUP.md) for the verification checklist).

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
| Update rate | **5 Hz per sensor** | firmware fires one sensor per 50 ms slot (4 × 50 ms cycle) to limit cross-talk; calculated, not yet measured on hardware |
| Stale after | 0.6 s | no reading for this long → sector coverage lost |

**Important limitation, by design:** four sensors give four *sectors* of range, not eight
directions. The software never invents range for the diagonal directions. With the default
15° beams there are blind zones around 45°, 135°, 225°, and 315°; the diagonal motors are
driven only when a camera supplies a bearing that is associated with an ultrasonic echo, when
bearing uncertainty genuinely spans a diagonal sector, or in the simulator's ground-truth mode.

Arduino pins for trigger/echo: **Proposed** in §3a.

## 3a. Arduino Uno pin map (Proposed: needs CAD/electronics confirmation)

Defined in one place: [`firmware/arduino_uno/src/hs_config.h`](../firmware/arduino_uno/src/hs_config.h).
Target: **Arduino Uno R3 or a compatible ATmega328P board** (16 MHz, 5 V logic). The board
revision still needs confirming. The *Uno R4* (Renesas RA4M1) is **not** compatible with the
pin-change-interrupt glue in `arduino_uno.ino`; the portable core in `src/` would need a new
hardware layer.

| Pin | Function | Notes |
|-----|----------|-------|
| D0 / D1 | USB serial RX / TX | reserved: protocol link |
| D2 / D3 | S0 TRIG / ECHO | echo D3 = PCINT19 (port D group) |
| D4 / D5 | S1 TRIG / ECHO | echo D5 = PCINT21 |
| D6 / D7 | S2 TRIG / ECHO | echo D7 = PCINT23 |
| D8 / D9 | S3 TRIG / ECHO | echo D9 = PCINT1 (port B group) |
| D10 | 74HC595 SER (pin 14) | bit-banged, not hardware SPI |
| D11 | 74HC595 SRCLK (pin 11) | |
| D12 | 74HC595 RCLK / latch (pin 12) | |
| D13 | on-board LED: mode indicator | solid = HOST, 4 Hz blink = FALLBACK, 1 Hz = BOOT; also blinks during the bootloader, so it is deliberately **not** used for the shift register |
| A0 (D14) | 74HC595 OE (pin 13), active low | **requires an external pull-up (e.g. 10 kΩ to 5 V)** |
| A1–A5 | free | e.g. future battery-voltage sense |

Why this mapping works on the Uno:

* **Echo timing:** only one sensor is armed at a time, and its echo pin's pin-change interrupt
  stamps both edges with `micros()` (4 µs resolution). D3/D5/D7 share the PCINT2 vector and D9
  uses PCINT0; both call the same handler. No `pulseIn()`, so nothing blocks.
* **Timers:** D3/D5/D6/D9 are PWM-capable but used as plain GPIO; `analogWrite` is never called,
  so Timer0 (`millis()` / `micros()`) is unaffected.
* **Echo inputs use the internal pull-up:** a disconnected echo wire reads HIGH, which the
  firmware reports as a sensor error (`E`) instead of an occasional phantom reading.
* **Boot behaviour:** during reset and the bootloader every pin is a high-impedance input, so
  the firmware cannot control the 74HC595 then. With **OE pulled up externally** the register's
  outputs stay disabled (and the ULN2803A inputs undriven) until `setup()` has shifted in `0x00`,
  latched it, and only then driven OE low. Firmware alone cannot guarantee safe outputs during
  reset. Also recommended for electronics review: SRCLR (pin 10) tied to 5 V; optional pull-downs
  on the TRIG lines.
* HC-SR04 TRIG lines float during reset; any spurious ping is harmless.

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

| Responsibility | MacBook (Python, this repo) | Arduino Uno (`firmware/arduino_uno`, implemented, not hardware-validated) |
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
