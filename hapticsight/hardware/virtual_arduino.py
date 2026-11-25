"""Virtual Arduino: the real firmware logic compiled natively, behind a pseudo-terminal.

``firmware/arduino_uno/src`` (the exact code that runs on the Uno, minus the
pin/ISR glue) is compiled with the host C++ compiler together with simulated
HC-SR04 sensors (``test/fw_sim.cpp``). The process is attached to a pty, so
the host connects to it with the normal :class:`PySerialTransport`.

**Everything it reports is SIMULATED.** It exists to test and demonstrate the
host <-> firmware integration before the hardware arrives; it says nothing
about real sensors, motors, or electrical behaviour.
"""

from __future__ import annotations

import contextlib
import os
import shutil
import subprocess
import sys
import tty
from pathlib import Path

from hapticsight.hardware.serial_transport import PySerialTransport

REPO_ROOT = Path(__file__).resolve().parents[2]
FIRMWARE_DIR = REPO_ROOT / "firmware" / "arduino_uno"
BUILD_DIR = REPO_ROOT / "build" / "firmware-sim"


def find_compiler() -> str | None:
    for name in (os.environ.get("CXX", ""), "c++", "clang++", "g++"):
        if name and shutil.which(name):
            return name
    return None


def build_virtual_firmware(force: bool = False, build_dir: Path = BUILD_DIR) -> Path:
    """Compile ``fw_sim`` if it is missing or older than any firmware source."""
    sources = [*sorted((FIRMWARE_DIR / "src").glob("*.cpp")), FIRMWARE_DIR / "test" / "fw_sim.cpp"]
    deps = [*sources, *(FIRMWARE_DIR / "src").glob("*.h"), FIRMWARE_DIR / "test" / "fake_hal.h"]
    binary = build_dir / "fw_sim"
    if not force and binary.exists():
        newest = max(p.stat().st_mtime for p in deps)
        if binary.stat().st_mtime >= newest:
            return binary
    cxx = find_compiler()
    if cxx is None:
        raise RuntimeError("no C++ compiler found (install Xcode Command Line Tools or g++)")
    build_dir.mkdir(parents=True, exist_ok=True)
    tmp = binary.with_suffix(".tmp")
    cmd = [cxx, "-std=c++11", "-O1", "-Wall", "-Wextra", "-o", str(tmp), *map(str, sources)]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"building the virtual firmware failed:\n{proc.stderr}")
    tmp.replace(binary)
    return binary


class VirtualArduino:
    """Runs the virtual Uno on a pty. Use :meth:`transport` to connect to it."""

    def __init__(self, ranges: str | None = None, binary: Path | None = None) -> None:
        if sys.platform == "win32":  # pragma: no cover
            raise RuntimeError("the virtual Arduino needs a POSIX pseudo-terminal")
        self.binary = binary or build_virtual_firmware()
        master, slave = os.openpty()
        tty.setraw(slave)
        tty.setraw(master)
        ctl_r, ctl_w = os.pipe()
        self._proc = subprocess.Popen(
            [str(self.binary), "--realtime", "--control-fd", str(ctl_r)],
            stdin=master,
            stdout=master,
            stderr=subprocess.DEVNULL,
            pass_fds=(ctl_r,),
            close_fds=True,
        )
        os.close(ctl_r)
        os.close(master)  # the child holds it now
        self._slave = slave  # keep open so the pty never reports hang-up to the child
        self._ctl = ctl_w
        self.port = os.ttyname(slave)
        self.ranges: dict[int, str] = {}
        if ranges:
            self.set_ranges(ranges)

    # ------------------------------------------------------------ control

    def _command(self, line: str) -> None:
        with contextlib.suppress(OSError):
            os.write(self._ctl, (line + "\n").encode())

    def set_range(self, sensor: int, value: str | float) -> None:
        """value: distance in cm, "X" (no echo), "NORISE" (missing), "STUCK"."""
        v = str(int(value)) if isinstance(value, (int, float)) else value.upper()
        self.ranges[sensor] = v
        self._command(f"range {sensor} {v}")

    def set_ranges(self, spec: str) -> None:
        """Parse ``"S0=80,S1=X,S2=NORISE"``."""
        for part in spec.split(","):
            key, _, value = part.partition("=")
            self.set_range(int(key.strip().upper().lstrip("S")), value.strip())

    def reset(self) -> None:
        """Simulate an MCU reset (what DTR does on a real Uno)."""
        self._command("reset")

    @property
    def alive(self) -> bool:
        return self._proc.poll() is None

    def transport(self) -> VirtualSerialTransport:
        return VirtualSerialTransport(self)

    def close(self) -> None:
        self._command("quit")
        try:
            self._proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait(timeout=2)
        for fd in (self._ctl, self._slave):
            with contextlib.suppress(OSError):
                os.close(fd)

    def __enter__(self) -> VirtualArduino:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class VirtualSerialTransport(PySerialTransport):
    """pyserial on the virtual board's pty; "reset" goes through the control pipe."""

    resets_on_open = False  # a pty has no DTR auto-reset: the bridge asks for one
    kind = "virtual"

    def __init__(self, board: VirtualArduino) -> None:
        super().__init__(board.port)
        self.board = board

    def pulse_reset(self) -> None:
        # Flush first, then reset: the board's fresh $HLO must not be discarded.
        if self._ser is not None:
            with contextlib.suppress(OSError):
                self._ser.reset_input_buffer()
        self.board.reset()
