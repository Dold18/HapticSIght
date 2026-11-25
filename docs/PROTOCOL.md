# HapticSight Serial Protocol — Version 1

Host (MacBook, Python) ⇄ Arduino Uno over USB serial.

**Status (Milestone 3):** both sides are implemented.
* Host: [`hapticsight/protocol/`](../hapticsight/protocol) (codec, framing, link monitor),
  [`hapticsight/hardware/arduino.py`](../hapticsight/hardware/arduino.py) (connection state
  machine) and [`serial_transport.py`](../hapticsight/hardware/serial_transport.py) (pyserial).
* Firmware: [`firmware/arduino_uno/`](../firmware/arduino_uno) (see [FIRMWARE.md](FIRMWARE.md)).
  It compiles for the Uno, and its protocol logic is tested natively and cross-checked against the
  Python codec. **It has not yet run on a physical Arduino**, and nothing here is validated on
  hardware. Sections marked *Firmware requirement* are implemented by that code.

## 1. Design goals

* Fits Arduino Uno constraints: 2 KB SRAM, 64-byte serial RX buffer → every line ≤ 64 bytes.
* Human-readable for debugging with a serial monitor.
* Cheap to validate: XOR checksum, fixed field counts, integer fields only.
* Fails safe: malformed input is discarded; motor commands expire; loss of the host falls back to
  local ultrasonic-only warnings, never to "all clear".

## 2. Framing

```
$<TYPE>,<field1>,<field2>,...*<CS>\n
```

| Element | Rule |
|---------|------|
| Start   | `$` |
| Body    | message type followed by comma-separated fields; printable ASCII only |
| `CS`    | XOR of every byte of the body (between `$` and `*`), as **2 hex digits**. Senders use uppercase. |
| End     | `\n` (a preceding `\r` is tolerated) |
| Length  | ≤ 64 bytes including `\n` |

Link settings: 115200 baud, 8N1 (implemented on both sides; not yet validated on hardware).

## 3. Messages: Arduino → host

### `HLO`: boot / version
```
$HLO,<protocol_version>,<firmware_version>*CS
$HLO,1,0.1.0*4B
```
Sent once at boot. `firmware_version` is 1–12 chars of `[A-Za-z0-9._-]`. The host refuses to send
motor commands if `protocol_version` ≠ 1, and re-sends the pattern table after every `HLO`
(the board rebooted).

**Handshake rules (host):** opening the USB serial port resets an Uno (DTR auto-reset), and the
bootloader runs for roughly 0.5–2 s. Data sent during that window can delay the sketch, so the host
**sends nothing until it has received a valid `$HLO`**. `$US`/`$ST` frames received before the
handshake are discarded; they show the board is alive but do not prove compatibility. If no `$HLO`
arrives within 3 s the host pulses DTR once to force a reset. It flushes its input *before* the
pulse, so the fresh `$HLO` is never discarded. If that also fails the link is reported as
`no_handshake` and retried later. A status clock that jumps backwards without an `$HLO` is treated
as an unnoticed reset, and the pattern table is re-sent.

### `US`: ultrasonic ranges
```
$US,<seq>,<t_ms>,<r0>,<r1>,<r2>,<r3>*CS
$US,1203,845210,85,X,E,212*2D
```
| Field | Range | Meaning |
|-------|-------|---------|
| `seq` | 0–65535, wraps | frame counter, used to count dropped frames |
| `t_ms` | 0–4294967295 | Arduino `millis()` at measurement |
| `r0..r3` | S0..S3 | integer cm `0–999`, or `X` = no echo within range, or `E` = sensor error/timeout |

The host stamps readings with its own monotonic **receive** time. Values below the valid minimum
(2 cm) are treated as errors and above the maximum (400 cm) as no echo.

The firmware sends one `US` frame per complete S0→S3 cycle. Sensors are fired one at a time in
50 ms slots, so each sensor updates at **5 Hz** (calculated from the slot timing; measured at
4.9–5.1 Hz with the virtual Uno, **not yet measured on hardware**). `E` covers: echo never rose
(sensor missing or unpowered), echo line already high before triggering (stuck or disconnected,
since echo inputs use pull-ups), or a reading under 2 cm. `X` means an echo pulse longer than
25 ms (nothing within about 430 cm).

### `ST`: status / heartbeat
```
$ST,<seq>,<t_ms>,<mode>,<last_hap_seq>,<rx_errors>*CS
$ST,310,845250,H,42,0*69
$ST,311,845500,F,-,3*4C
```
| Field | Meaning |
|-------|---------|
| `mode` | `B` boot (motors off, waiting for host) · `H` executing host commands · `F` fallback (ultrasonic-only) |
| `last_hap_seq` | seq of the last *accepted* `HAP`, or `-` if none |
| `rx_errors` | count of rejected lines, saturating at 65535 |

## 4. Messages: host → Arduino

### `HAP`: 8-channel haptic command
```
$HAP,<seq>,<ttl_ms>,<p0p1p2p3p4p5p6p7>*CS
$HAP,42,500,03007001*43
```
| Field | Rule |
|-------|------|
| `seq` | 0–65535, wraps |
| `ttl_ms` | 1–2000. The command expires `ttl_ms` after receipt. Values above 2000 are invalid so a faulty host can never latch motors on. |
| patterns | exactly 8 uppercase hex digits, one **pattern slot** per motor, M0 first |

The example means M1 = slot 3 (CRITICAL), M4 = slot 7 (DEGRADED), M7 = slot 1 (CAUTION), others off.

### `PAT`: pattern definition
```
$PAT,<slot>,<on_ms>,<off_ms>,<count>,<pause_ms>*CS
$PAT,2,150,0,1,350*58
```
Defines what a slot plays: a burst of `count` pulses of `on_ms`, separated by `off_ms`, followed by
`pause_ms` of silence, repeated. Slot 0 is permanently OFF and cannot be redefined. Ranges:
slot 1–15, times 0–65535 ms, count 0–9, and `on_ms > 0` whenever `count > 0`.

### Default pattern table

The firmware must ship with this table built in (`DEFAULT_PATTERNS` in
[`haptics/patterns.py`](../hapticsight/haptics/patterns.py)); the host also sends it with `$PAT`
on connect.

| Slot | Name | on | off | count | pause | Cycle | Duty | Feel |
|-----:|------|---:|----:|------:|------:|------:|-----:|------|
| 0 | OFF | – | – | 0 | – | – | 0 % | silent |
| 1 | CAUTION | 100 | 0 | 1 | 900 | 1000 ms | 10 % | 1 Hz tap |
| 2 | WARNING | 150 | 0 | 1 | 350 | 500 ms | 30 % | 2 Hz tap |
| 3 | CRITICAL | 180 | 0 | 1 | 70 | 250 ms | 72 % | 4 Hz near-continuous |
| 4 | CAUTION_APPROACH | 80 | 80 | 2 | 760 | 1000 ms | 16 % | 1 Hz double tap |
| 5 | WARNING_APPROACH | 100 | 60 | 2 | 240 | 500 ms | 40 % | 2 Hz double tap |
| 6 | CRITICAL_APPROACH | 100 | 30 | 2 | 20 | 250 ms | 80 % | 4 Hz double tap |
| 7 | DEGRADED | 50 | 80 | 3 | 1690 | 2000 ms | 8 % | sparse triple blip |
| 8 | ADVISORY *(experimental, optional)* | 40 | 0 | 1 | 2960 | 3000 ms | 1 % | single tick every 3 s |
| 9–15 | reserved | | | | | | | |

Slot 8 (added in Milestone 2, backwards compatible) is used only when the host's experimental
camera-only advisory cue is enabled; it is **off by default**. The host defines it with `$PAT` like
every other slot. *Firmware requirement:* a slot that has never been defined (no built-in
default and no `$PAT`) must play as OFF, so older firmware stays safe.

These timings are provisional and need tuning with the real motors and wearer feedback.

## 5. Timing, heartbeat, and expiry

| Rule | Value |
|------|-------|
| Host sends `HAP` | immediately when the pattern string changes (≥ 20 ms apart), else every 100 ms as a heartbeat |
| Host `HAP` TTL | 500 ms by default (tolerates 4 lost frames at 10 Hz) |
| Arduino sends `US` | once per 4-sensor cycle: every 200 ms (5 Hz) with the default 50 ms slots |
| Arduino sends `ST` | every 200 ms, and immediately on every mode change |
| Host link timeout | 0.75 s without any valid message → link lost, reconnect (`ArduinoBridge`) |
| Host ultrasonic staleness | a sensor without a reading for > 0.6 s is treated as lost coverage (DEGRADED cue) |

When the host link is down the ultrasonic readings stop arriving, so the host pipeline marks every
sensor STALE and the encoder requests the DEGRADED cue; it does not interpret silence as clear.

*Firmware requirement:* pattern phase restarts only when a channel's slot **changes**; re-sending
the same slot must not restart it. Pulse timing is generated from `millis()`.

## 6. Error handling and fallback (*Firmware requirement*)

1. **Malformed lines** (bad framing, bad checksum, wrong field count, out-of-range value, unknown
   type, over-long line): discard the whole line, increment `rx_errors`, and keep the previous
   state. An over-long line is discarded up to the next `\n`.
2. **Boot:** all motors off (mode `B`) until the first valid `HAP`. Send `HLO`.
3. **Command expiry → fallback:** if the current `HAP` command expires (TTL elapsed with no newer
   valid `HAP`), enter mode `F` immediately. Do not just switch the motors off: silence must not
   be read as "clear".
4. **Fallback mode (ultrasonic-only):**
   * Signal the mode change once: one burst of the DEGRADED pattern (≈ 310 ms with the default
     timing) on all 8 motors, then start local warnings. The rest of that pattern's 2 s cycle is
     silence, so the wait is not extended. All 8 motors starting at once may exceed the motor
     supply's capacity (**not verified**); `HS_FB_CUE_ALL_MOTORS 0` limits the cue to
     M0/M2/M4/M6.
   * Local warnings use 350 ms release hysteresis (like the host) and treat readings older than
     600 ms as missing.
   * Drive M0/M2/M4/M6 from S0/S1/S2/S3 with the default distance bands
     (< 50 cm CRITICAL, 50–99 WARNING, 100–200 CAUTION, else off) and slots 1–3.
   * A sensor that errors or times out drives its motor with DEGRADED.
   * Diagonal motors stay off: there is no sensor coverage there, and the wearer has already
     been told the system is degraded.
   * Missing camera information is never treated as clear surroundings.
5. **Recovery:** the first valid `HAP` returns the firmware to mode `H`.
6. **Atomicity:** a line is fully parsed and validated before anything is applied; a rejected
   line never partially changes motor commands or the pattern table, and it does not refresh the
   command watchdog.
7. **Boot without a host:** protocol v1 keeps mode `B` (motors off) until a host command arrives.
   The firmware has a compile-time option `HS_BOOT_TO_FALLBACK_MS` (default off) to start local
   ultrasonic warnings after a timeout instead. Enabling it is an open decision for the project
   owner.

## 7. Line-length budget

| Message | Worst case |
|---------|-----------:|
| `$US,65535,4294967295,999,999,999,999*3B\n` | 40 B |
| `$PAT,15,65535,65535,9,65535*64\n` | 31 B |
| `$HAP,65535,2000,FFFFFFFF*..\n` | 28 B |
| `$ST,65535,4294967295,F,65535,65535*..\n` | 38 B |

All lines fit within the Uno's 64-byte receive buffer.

## 8. Versioning

* `PROTOCOL_VERSION = 1` (in `hapticsight/protocol/messages.py`).
* Adding a new message type or new pattern slots is backwards compatible: receivers reject unknown
  types as malformed lines and continue.
* Changing a field's meaning, order, or count requires a version bump and coordinated firmware.

## 9. Host implementation map

| Concern | Code |
|---------|------|
| Message types and constants | `protocol/messages.py` |
| Encode / decode / validate | `protocol/codec.py` (`encode`, `decode`, `ProtocolError` hierarchy) |
| Byte stream → lines, overflow resync | `protocol/link.py::StreamDecoder` |
| Heartbeat timeout, dropped-frame count, firmware mode | `protocol/link.py::LinkMonitor` |
| Readings in, haptic frames out, handshake, reconnection | `hardware/arduino.py::ArduinoBridge` |
| Real serial port (pyserial), DTR reset | `hardware/serial_transport.py::PySerialTransport` |
| Firmware parser / formatter | `firmware/arduino_uno/src/hs_protocol.{h,cpp}` |
| Tests | `tests/test_protocol.py` (codec), `tests/test_arduino_bridge.py` (state machine), `tests/test_firmware_native.py` (C++ ↔ Python parity), `tests/test_hardware_integration.py` (host ↔ virtual Uno over a pty) |
