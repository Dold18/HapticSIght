# Hardware Bring-Up and Bench-Test Checklist

For the first power-up of the Arduino, sensors, shift register, driver, and motors. Nothing in
this document has been performed yet: **no physical hardware had been connected when this
software was written.** Record results in §5 as you go.

> ⚠️ HapticSight is an experimental prototype, not a safety device. Bench-test on a table, not
> on a person, until every electrical item below is verified.

## 1. Electrical rules (do not skip)

* **Never connect motors to Arduino GPIO pins.** Motors connect only to the ULN2803A outputs.
  Arduino pins drive logic (74HC595 inputs, HC-SR04 TRIG) and nothing else.
* **Motor power is separate from Arduino logic power.** The motors draw far more current than
  the Uno's 5 V pin should supply.
* **Common ground, no common positive.** Arduino GND, the 74HC595/ULN2803A GND, and the motor
  supply's negative must be connected. Do **not** connect the motor supply's positive rail to the
  Arduino's 5 V, and avoid two USB sources back-feeding each other through any positive
  connection.
* **ULN2803A COM pin (pin 10)** goes to the motor supply's positive rail, so the internal
  flyback diodes clamp the motors' inductive spikes. Verify against the datasheet and the final
  schematic.
* **74HC595 OE (pin 13) needs an external pull-up** (e.g. 10 kΩ to 5 V) so outputs are
  disabled during reset and the bootloader. Firmware drives OE low only after it has cleared the
  register. SRCLR (pin 10) to 5 V. Add 100 nF decoupling at each IC.
* **Do not assume an adjustable buck converter gives a stable 3 V** just because its knob was
  turned. Measure it under load. Note also that a **5 V USB input may not leave enough headroom**
  for some buck converters to regulate properly; check the converter's dropout specification.
* The ULN2803A is a Darlington array: expect roughly **0.7–1.2 V drop** at motor currents. A
  "3 V motor" on a 3 V supply may only see about 2 V. Set the supply by measuring the voltage
  **across a running motor**, not at the supply terminals.

### Values the electronics workstream must verify and record

| Item | How | Result |
|------|-----|--------|
| Motor model, rated voltage, rated and stall current | datasheet + bench measurement | |
| Motor supply voltage at no load / one motor / eight motors | multimeter at the ULN2803A COM pin | |
| Voltage across a running motor (one, and all eight) | multimeter across the motor terminals | |
| Start-up and running current, one motor and eight motors | bench supply or inline meter | |
| ULN2803A dissipation estimate (Vce(sat) × I × channels) | from measured values vs datasheet limit | |
| Motor polarity and wiring order (OUT1 → M0 … OUT8 → M7) | one-motor test, §3 step 6 | |
| OE pull-up present; outputs off during reset | §3 step 4 | |
| Buck converter regulation headroom from the 5 V input | measure output vs input under load | |
| HC-SR04 supply current (4 sensors) from the Uno 5 V | inline meter | |
| Board revision (Uno R3 / clone / ATmega328P confirmed) | board markings, `arduino-cli board list` | |

## 2. Software prerequisites

```bash
pip install -e ".[dev]"                     # includes pyserial
brew install arduino-cli && arduino-cli core update-index && arduino-cli core install arduino:avr
hapticsight-hw link-loss --port virtual     # sanity check of the host stack, no hardware needed
```

## 3. Step-by-step bench procedure

Stop at the first unexpected result. Use `--log recordings/bench` on the monitoring commands to
keep a JSONL record of each step.

1. **Bare board.** Connect only the Uno by USB. Flash:
   `arduino-cli upload -p <port> --fqbn arduino:avr:uno firmware/arduino_uno`.
   Then run `hapticsight-hw monitor --port <port> --seconds 10`. Expect `$HLO,1,0.3.0` and
   `$ST,…,B,…` every 200 ms, and `$US` with all four sensors `E` (echo pins pulled up, nothing
   attached). The LED blinks at 1 Hz (BOOT).
2. **Link behaviour without peripherals.** Run
   `hapticsight-hw link-loss --port <port> --hold 3 --stop 3 --resume 2`. Expect `B → H → F → H`.
   FALLBACK should start about TTL (500 ms) after the last command. The LED is solid in H and
   blinks at 4 Hz in F.
3. **One ultrasonic sensor** (5 V, GND, TRIG/ECHO per HARDWARE_INTERFACE §3a), then
   `hapticsight-hw ultrasonic --port <port>`. Check:
   * the reported per-sensor rate (expected about 5 Hz);
   * readings against a tape measure at 30, 50, 100, and 200 cm (record the error);
   * `X` when pointed at open space;
   * the sensor's slot turns `E` when unplugged.

   Repeat for each sensor, then all four together. **Cross-talk:** with all four mounted as on
   the vest, check for spurious short readings (`HS_US_SLOT_MS` can be tuned only on the basis of
   these measurements).
4. **Shift register, logic only (no ULN2803A, no motors).** Fit LEDs with series resistors on
   QA–QH, or use a logic analyser. Use `hapticsight-hw motor-test --port <port> --interactive`
   and drive each output once (`on 0 CAUTION 2` … `on 7 CAUTION 2`). Confirm QA = M0 … QH = M7.
   **Press the Uno reset button: every output must stay off through reset** (OE pull-up).
5. **ULN2803A with the motor supply, no motors.** Verify supply voltage and COM wiring, then
   repeat step 4, measuring each OUT pin (should sink to near GND when on).
6. **One motor.** Connect M0 only, then
   `hapticsight-hw motor-test --port <port> --motor 0 --pattern CAUTION --seconds 2 --yes`.
   Measure the voltage across the running motor and the current. Try `WARNING` and `CRITICAL`.
   Repeat for each motor position to confirm wiring order and polarity.
7. **All motors; fallback cue.** With every motor connected, run `link-loss` again. The
   FALLBACK entry cue starts all eight motors at once for about 310 ms: watch the supply voltage
   and check for Arduino resets (a reset shows as a new `$HLO` in `monitor`). If the supply cannot
   handle it, rebuild with `HS_FB_CUE_ALL_MOTORS 0` and record that decision.
8. **Fallback warnings.** With host commands stopped, hold an object in front of each sensor at
   150, 80, and 30 cm and confirm the corresponding cardinal motor plays CAUTION, WARNING, or
   CRITICAL. Disconnect a sensor and confirm its motor plays DEGRADED, not silence.
9. **Full system.** `hapticsight-hw run --port <port> --front 0 --rear 1 --log recordings/bench`.
   Confirm the UI shows **REAL** (green) ultrasonic data and the firmware mode. Walk a target
   toward each sensor and compare the motor that fires with the expected one.

## 4. Logging and evidence

* `hapticsight-hw monitor|run --log DIR` writes JSONL telemetry with raw serial lines,
  link-state changes, readings (with an explicit `simulated` flag), assessments, and haptic
  commands. No camera images are written unless `--record-video` is given.
* Keep logs and photos of each step locally (`recordings/` is git-ignored), and summarise the
  measured values in §5 and in `docs/DEVELOPMENT_STATUS.md`.

## 5. Results template (fill in during bring-up)

| Step | Date | Board / supply | Result | Notes |
|-----:|------|----------------|--------|-------|
| 1 | | | | |
| 2 | | | | |
| 3 | | | measured per-sensor rate: __ Hz; errors at 30/50/100/200 cm: __ | |
| 4 | | | output order OK? outputs off during reset? | |
| 5 | | | | |
| 6 | | | V across motor: __ V; I: __ mA | |
| 7 | | | supply droop at 8-motor cue: __ V; resets? | |
| 8 | | | | |
| 9 | | | | |
