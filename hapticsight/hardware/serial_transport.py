"""Real USB serial transport (pyserial) for :class:`ArduinoBridge`.

* Non-blocking, bounded reads: ``read_available`` never waits and returns at
  most ``max_read_bytes`` per call.
* Opening the port asserts DTR, which **resets an Arduino Uno** (auto-reset
  circuit); the bridge therefore stays silent until it receives ``$HLO``.
  ``pulse_reset`` toggles DTR to force another reset if no ``$HLO`` arrives.
* All pyserial / OS errors surface as :class:`TransportError` so the bridge can
  drop the connection and reconnect.
"""

from __future__ import annotations

import contextlib
import time
from dataclasses import dataclass
from typing import Any

DEFAULT_BAUD = 115200

# USB vendor IDs commonly seen on Uno boards and clones (identification aid only).
KNOWN_USB_VIDS: dict[int, str] = {
    0x2341: "Arduino",
    0x2A03: "Arduino (arduino.org)",
    0x1A86: "WCH CH340 (common on Uno clones)",
    0x0403: "FTDI",
    0x10C4: "Silicon Labs CP210x",
}


class TransportError(OSError):
    """The serial port failed (unplugged, permission denied, busy, ...)."""


@dataclass(frozen=True, slots=True)
class PortInfo:
    device: str
    description: str
    manufacturer: str
    vid: int | None
    pid: int | None
    serial_number: str
    likely_arduino: bool
    vendor_hint: str


def list_serial_ports() -> list[PortInfo]:
    from serial.tools import list_ports

    out = []
    for p in list_ports.comports():
        hint = KNOWN_USB_VIDS.get(p.vid, "") if p.vid is not None else ""
        text = f"{p.description} {p.manufacturer or ''}".lower()
        likely = bool(hint) or "arduino" in text or "usbmodem" in p.device.lower()
        out.append(
            PortInfo(
                device=p.device,
                description=p.description or "",
                manufacturer=p.manufacturer or "",
                vid=p.vid,
                pid=p.pid,
                serial_number=p.serial_number or "",
                likely_arduino=likely,
                vendor_hint=hint,
            )
        )
    out.sort(key=lambda p: (not p.likely_arduino, p.device))
    return out


class PySerialTransport:
    """``ByteTransport`` over a real serial port."""

    resets_on_open = True  # an Uno resets when DTR is asserted on open
    kind = "serial"

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUD,
        max_read_bytes: int = 4096,
        write_timeout_s: float = 0.05,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.max_read_bytes = max_read_bytes
        self.write_timeout_s = write_timeout_s
        self._ser: Any = None

    def open(self) -> None:
        import serial

        try:
            self._ser = serial.Serial(
                self.port, self.baudrate, timeout=0, write_timeout=self.write_timeout_s
            )
        except (serial.SerialException, OSError, ValueError) as exc:
            self._ser = None
            raise TransportError(f"cannot open {self.port}: {exc}") from exc

    @property
    def is_open(self) -> bool:
        return self._ser is not None and bool(self._ser.is_open)

    def read_available(self) -> bytes:
        import serial

        if self._ser is None:
            raise TransportError("port not open")
        try:
            waiting = self._ser.in_waiting
            if not waiting:
                return b""
            return bytes(self._ser.read(min(waiting, self.max_read_bytes)))
        except (serial.SerialException, OSError) as exc:
            raise TransportError(f"read failed on {self.port}: {exc}") from exc

    def write(self, data: bytes) -> None:
        import serial

        if self._ser is None:
            raise TransportError("port not open")
        try:
            self._ser.write(data)
        except serial.SerialTimeoutException as exc:
            raise TransportError(f"write timed out on {self.port} (device not reading?)") from exc
        except (serial.SerialException, OSError) as exc:
            raise TransportError(f"write failed on {self.port}: {exc}") from exc

    def pulse_reset(self) -> None:
        """Toggle DTR to reset the board (auto-reset circuit).

        Stale input is dropped *before* the reset, never after it: anything the
        board sends once it restarts (its ``$HLO``) must survive.
        """
        if self._ser is None:
            return
        with contextlib.suppress(OSError):
            self._ser.reset_input_buffer()
            self._ser.dtr = False
            time.sleep(0.1)
            self._ser.dtr = True  # some adapters/ptys ignore DTR

    def close(self) -> None:
        if self._ser is not None:
            with contextlib.suppress(OSError):
                self._ser.close()
            self._ser = None
