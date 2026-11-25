# HapticSight firmware: Arduino Uno (ATmega328P)

Implements serial protocol v1 ([docs/PROTOCOL.md](../../docs/PROTOCOL.md)). Full documentation:
[docs/FIRMWARE.md](../../docs/FIRMWARE.md). Bench bring-up:
[docs/HARDWARE_BRINGUP.md](../../docs/HARDWARE_BRINGUP.md).

**Not yet run on physical hardware. Pin assignments in `src/hs_config.h` are provisional.**

```bash
arduino-cli compile --fqbn arduino:avr:uno --warnings all .     # from this directory
arduino-cli upload -p /dev/cu.usbmodemXXXX --fqbn arduino:avr:uno .
make test     # native unit tests (no Arduino needed)
make sim      # virtual Uno used by the Python tests and `hapticsight-hw --port virtual`
```
