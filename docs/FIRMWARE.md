# Arduino Uno Firmware

Source: [`firmware/arduino_uno/`](../firmware/arduino_uno). Implements serial protocol v1
([PROTOCOL.md](PROTOCOL.md)): staggered HC-SR04 ranging, haptic pulse scheduling for eight motors
through a 74HC595, the BOOT/HOST/FALLBACK modes, and the ultrasonic-only fallback.

> **Status:** compiles for the Arduino Uno. The hardware-independent logic is unit-tested
> natively and cross-checked against the Python host. **It has not been run on a physical Arduino,
> sensors, or motors yet.** Pin assignments are provisional ([HARDWARE_INTERFACE.md §3a](HARDWARE_INTERFACE.md)).
> Electrical parameters are unverified ([HARDWARE_BRINGUP.md](HARDWARE_BRINGUP.md)).

## 1. Structure

```
firmware/arduino_uno/
  arduino_uno.ino        hardware glue only: pins, pin-change ISRs, Serial, 74HC595, LED
  src/hs_config.h        PROVISIONAL pin map, timing, thresholds, feature flags
  src/hs_protocol.*      line assembler, strict v1 parser (HAP/PAT), HLO/US/ST formatting
  src/hs_patterns.*      16-slot pattern table, pulse timing, 8-channel scheduler
  src/hs_ultrasonic.*    non-blocking staggered HC-SR04 state machine (HAL interface)
  src/hs_fallback.*      ultrasonic-only local warnings with hysteresis
  src/hs_firmware.*      modes, watchdog, command handling, outgoing frame scheduling
  test/test_main.cpp     native unit tests (312 checks)
  test/fake_hal.h        simulated HC-SR04 timing
  test/fw_sim.cpp        "virtual Uno": scripted mode for tests, realtime mode on a pty
  Makefile               make test | make sim | make compile
```

Everything in `src/` is plain C++11 with `<stdint.h>`/`<string.h>` only: no Arduino headers,
no `String`, no heap allocation, no STL. The same files are compiled by `avr-gcc` for the Uno and
by the host compiler for tests.

### Main loop (`arduino_uno.ino`)

1. Drain `Serial` (the RX buffer is 64 bytes) into `Firmware::onRxByte`.
2. `Firmware::update(millis(), micros())`: ultrasonic state machine, command watchdog, mode,
   slot selection.
3. Compute the motor byte; shift it out when it changes, and refresh every 100 ms.
4. Write at most one pending frame, **only if** `Serial.availableForWrite()` has room (never
   blocks).
5. Update the status LED.

The only busy-wait is the 10 µs HC-SR04 trigger pulse.

## 2. Ultrasonic acquisition and its real rate

* One sensor per **50 ms slot**, in the order S0, S1, S2, S3, so the cycle is 200 ms and **each
  sensor updates at 5 Hz** (20 measurements/s in total).
* Within a slot: if the echo line is already HIGH → `E` (stuck, or disconnected given the
  pull-up), and the sensor is **not** triggered. Otherwise the pin-change interrupt is armed and
  the 10 µs trigger sent. Rise and fall are time-stamped in the ISR with `micros()`.
* No rise within 4 ms → `E` (sensor missing or unpowered). Echo longer than 25 ms → `X` (nothing
  within about 430 cm; an HC-SR04 holds echo high ~38 ms when nothing returns). Otherwise
  distance = (echo µs + 29) / 58 cm, so < 2 cm → `E` and > 400 cm → `X`.
* After a cycle completes, one `$US` frame carries the four latest results.

**Why not 10 Hz?** Four sequential sensors at 10 Hz each would need 25 ms slots. That is barely
the 23 ms round trip at 400 cm, and leaves no quiet time for echoes from beyond the rated range
(or multipath) to die out before the next sensor fires, which is a classic source of cross-talk.
The 50 ms slot (25 ms maximum echo plus ≥ 25 ms quiet) favours cross-talk resistance, as the
milestone brief requires. `HS_US_SLOT_MS` can be tuned *after* measuring cross-talk on the vest.
The host's staleness (0.6 s) and range-rate window (1.0 s) were adjusted to 5 Hz accordingly.

Speed of sound is taken as ~343 m/s (20 °C); the error is about ±1.7 % per 10 °C.

## 3. Haptic scheduling

* Pattern table: slot 0 fixed OFF, slots 1–8 built-in (identical to `DEFAULT_PATTERNS` in
  Python, and tested), slots 9–15 undefined until `$PAT`. **Undefined slots play as OFF.**
* Per channel: slot, slot revision, start time. Re-sending the same slot keeps the phase; a
  different slot, or a redefinition via `$PAT`, restarts it. Elapsed time is `now - start` in
  unsigned 32-bit arithmetic, so it is correct across the 49.7-day `millis()` rollover.
* Bit *k* of the output byte drives motor M*k*: `shiftOut(MSBFIRST)` puts bit 7 on QH (M7) and
  bit 0 on QA (M0).
* Output is on/off only. Motor intensity, voltage, and current are electrical concerns and are
  **not verified** (see HARDWARE_BRINGUP.md).

## 4. Modes, watchdog, fallback

| Mode | Entered when | Motors |
|------|--------------|--------|
| `B` BOOT | reset | all OFF (until the first valid `$HAP`) |
| `H` HOST | any valid `$HAP` | as commanded |
| `F` FALLBACK | the latest `$HAP`'s TTL expires | entry cue (one DEGRADED burst, ≈310 ms, on all 8 motors), then local warnings |

Fallback local warnings: S0 → M0, S1 → M2, S2 → M4, S3 → M6, using the host's distance bands
(< 50 cm CRITICAL, < 100 WARNING, ≤ 200 CAUTION). A reading that is an error, missing, or older
than 600 ms → DEGRADED: missing data is never treated as clear. Fresh no-echo → OFF for that
cone only. Diagonal motors stay off. There is 350 ms release hysteresis.

Malformed lines (any framing, checksum, field, range, or type error; lines over 63 characters)
are discarded whole. They increment `rx_errors` (saturating at 65535, reported in `$ST`), never
partially apply, and never refresh the watchdog. `$ST` is sent every 200 ms and immediately on
every mode change.

Compile-time options in `hs_config.h`:

| Option | Default | Meaning |
|--------|---------|---------|
| `HS_US_SLOT_MS` | 50 | ultrasonic slot length (sets the 5 Hz per-sensor rate) |
| `HS_FB_CUE_ALL_MOTORS` | 1 | 0 = fallback entry cue on M0/M2/M4/M6 only (if the supply cannot start 8 motors at once) |
| `HS_BOOT_TO_FALLBACK_MS` | 0 (off) | >0 = start ultrasonic-only warnings this long after boot without a host (deviates from protocol v1 text; owner decision) |
| `HS_USE_STATUS_LED` | 1 | D13 mode indicator |

## 5. Memory and size (measured)

`arduino-cli compile --fqbn arduino:avr:uno` (arduino:avr 1.8.8, avr-gcc 7.3, -Os, LTO):

| | Used | Of | Share |
|--|-----:|---:|------:|
| Flash | 9,302 B | 32,256 B | 28 % |
| SRAM (globals) | 884 B | 2,048 B | 43 % (1,164 B left for the stack) |

SRAM includes the Arduino `Serial` RX/TX buffers (2 × 64 B). Firmware state is statically
allocated. CI fails the build if global SRAM use exceeds 75 %.

## 6. Build, flash, and connect

```bash
# one-time tool setup (macOS; see arduino-cli docs for other platforms)
brew install arduino-cli
arduino-cli core update-index
arduino-cli core install arduino:avr

# compile (from the repository root)
arduino-cli compile --fqbn arduino:avr:uno --warnings all firmware/arduino_uno

# find the board and flash it (close any serial monitor first: the port must be free)
arduino-cli board list
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno firmware/arduino_uno

# talk to it
hapticsight-hw ports
hapticsight-hw monitor --port /dev/cu.usbmodemXXXX            # raw + decoded traffic
```

Native builds (no Arduino needed):

```bash
make -C firmware/arduino_uno test      # 312 checks, AddressSanitizer + UBSan
make -C firmware/arduino_uno sim       # virtual Uno binary
hapticsight-hw monitor --port virtual  # host <-> virtual Uno over a pseudo-terminal
```

### Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| `upload`: "programmer is not responding" / sync errors | wrong port or board; port held by another program (monitor, IDE); faulty or charge-only cable; some clones need a different bootloader setting |
| no `/dev/cu.usbmodem*` | charge-only cable; CH340 clones appear as `/dev/cu.wchusbserial*` or `/dev/cu.usbserial-*` |
| `monitor` shows `no_handshake` | wrong port or baud; firmware not flashed; board not resetting on open (the host retries with a DTR pulse) |
| `$US` all `E` | sensors not powered/connected (echo pull-ups read HIGH), or TRIG/ECHO swapped |
| motors never move but mode is `H` | OE pull-up present but OE pin not driven (check A0 wiring), motor supply off, ULN2803A COM/GND wiring |
| board resets when motors start | supply droop or missing common ground or decoupling: an electrical issue, see HARDWARE_BRINGUP.md |

## 7. Tests

* `firmware/arduino_uno/test/test_main.cpp` (312 checks, run with ASan/UBSan): parser
  acceptance and rejection (checksum, truncation, overlong lines, invalid numbers including uint32
  overflow, undefined slots, unexpected types), formatting against the documented examples, line
  overflow and resync, pulse timing, phase persistence, `millis()`/`micros()` rollover, motor
  order, staggered scheduling and its 5 Hz rate, stuck/missing sensors, fallback bands and
  hysteresis, BOOT/HOST/FALLBACK transitions, rapid disconnect/reconnect cycles, malformed input
  during fallback, sequence wraparound, and `rx_errors` saturation.
* `tests/test_firmware_native.py` (pytest): runs the above, then drives the virtual Uno to check
  that the C++ parser agrees with the Python codec on 1,200+ generated and mutated lines, that
  frames decode on the host, that the pattern table matches the host, that motor on/off
  timing matches Python pattern timing at every millisecond over 2 s, and that mode transitions
  occur exactly at TTL expiry.
* `tests/test_hardware_integration.py`: host ↔ virtual Uno over a pty with the real pyserial
  transport (handshake, readings, acknowledgements, expiry, recovery, reset detection,
  reconnection, runtime, CLI, UI).

None of these tests exercise real pins, interrupts, the 74HC595, sensors, or motors.
