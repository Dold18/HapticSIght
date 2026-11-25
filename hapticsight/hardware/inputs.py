"""Ultrasonic input sources for the live runtime, with unambiguous provenance.

The *kind* of a source is fixed by its class, never by user-supplied text, so a
demonstration cannot present synthetic distances as real measurements:

==========================  ======================================  ==========
kind                        what produces the distances             label
==========================  ======================================  ==========
``real-arduino``            physical HC-SR04s via the Uno firmware  REAL
``simulated-virtual``       virtual Uno (firmware logic + fake      SIMULATED
                            sensors) over a pseudo-terminal
``simulated-manual``        hand-set values in the host process     SIMULATED
==========================  ======================================  ==========
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol

from hapticsight.core.layout import ULTRASONIC_SENSORS
from hapticsight.core.models import RangeStatus, UltrasonicReading
from hapticsight.haptics.encoder import HapticFrame
from hapticsight.hardware.arduino import ArduinoBridge, BridgeStatus, LinkState
from hapticsight.hardware.virtual_arduino import VirtualArduino, VirtualSerialTransport
from hapticsight.simulation.manual_ultrasonic import ManualMode, ManualUltrasonic


class SourceKind(StrEnum):
    REAL_ARDUINO = "real-arduino"
    SIMULATED_VIRTUAL = "simulated-virtual-arduino"
    SIMULATED_MANUAL = "simulated-manual"

    @property
    def simulated(self) -> bool:
        return self is not SourceKind.REAL_ARDUINO


@dataclass(frozen=True, slots=True)
class SensorDisplay:
    text: str
    distance_cm: float | None
    status: str  # "echo" | "no_echo" | "error" | "none"


class UltrasonicInput(Protocol):
    kind: SourceKind

    def readings(self, now: float) -> list[UltrasonicReading]: ...

    def deliver(self, frame: HapticFrame, now: float) -> None: ...

    def display(self, sensor_id: str) -> SensorDisplay: ...

    def title(self) -> str: ...

    def bridge_status(self) -> BridgeStatus | None: ...

    @property
    def manual(self) -> ManualUltrasonic | None: ...

    def manual_changed(self) -> None: ...

    def close(self) -> None: ...


class ManualInput:
    """Hand-controlled SIMULATED ranges computed in-process (no firmware involved)."""

    kind = SourceKind.SIMULATED_MANUAL

    def __init__(self, manual: ManualUltrasonic | None = None) -> None:
        self._manual = manual or ManualUltrasonic()

    @property
    def manual(self) -> ManualUltrasonic:
        return self._manual

    def readings(self, now: float) -> list[UltrasonicReading]:
        return self._manual.readings(now)

    def deliver(self, frame: HapticFrame, now: float) -> None:
        pass  # no hardware: motors are virtual only

    def display(self, sensor_id: str) -> SensorDisplay:
        s = self._manual.sensors[sensor_id]
        if s.mode is ManualMode.ECHO:
            return SensorDisplay(s.describe(), s.distance_cm, "echo")
        if s.mode is ManualMode.NO_ECHO:
            return SensorDisplay("no echo", None, "no_echo")
        return SensorDisplay("dropout", None, "none")

    def title(self) -> str:
        return "ULTRASONIC: SIMULATED (manual values, no hardware)"

    def bridge_status(self) -> BridgeStatus | None:
        return None

    def manual_changed(self) -> None:
        pass

    def close(self) -> None:
        pass


class ArduinoInput:
    """Ranges from an Arduino bridge; haptic frames are sent back to it.

    With a :class:`VirtualArduino` the kind is SIMULATED_VIRTUAL and the hand
    controls (``manual``) set the virtual sensors' distances.
    """

    def __init__(self, bridge: ArduinoBridge, virtual: VirtualArduino | None = None) -> None:
        self.bridge = bridge
        self.virtual = virtual
        self.kind = SourceKind.SIMULATED_VIRTUAL if virtual else SourceKind.REAL_ARDUINO
        self._manual: ManualUltrasonic | None = None
        if virtual is not None:
            self._manual = ManualUltrasonic()
            for i, spec in enumerate(ULTRASONIC_SENSORS):
                v = virtual.ranges.get(i)
                if v is not None:
                    self._manual.set(spec.sensor_id, "X" if v in ("NORISE", "STUCK") else v)
        if virtual is None and isinstance(
            getattr(bridge, "transport", None), VirtualSerialTransport
        ):
            raise ValueError("a virtual transport must be declared with virtual=...")

    @property
    def manual(self) -> ManualUltrasonic | None:
        return self._manual

    def manual_changed(self) -> None:
        """Push hand-set values to the virtual board's simulated sensors."""
        if self.virtual is None or self._manual is None:
            return
        for i, spec in enumerate(ULTRASONIC_SENSORS):
            s = self._manual.sensors[spec.sensor_id]
            if s.mode is ManualMode.ECHO:
                self.virtual.set_range(i, round(s.distance_cm))
            elif s.mode is ManualMode.NO_ECHO:
                self.virtual.set_range(i, "X")
            else:
                self.virtual.set_range(i, "NORISE")  # sensor fault on the virtual board

    def readings(self, now: float) -> list[UltrasonicReading]:
        if self._manual is not None and self.virtual is not None:
            # advance any approach motion of the hand controls, then push it
            before = {k: (s.mode, round(s.distance_cm)) for k, s in self._manual.sensors.items()}
            self._manual.readings(now)
            after = {k: (s.mode, round(s.distance_cm)) for k, s in self._manual.sensors.items()}
            if before != after:
                self.manual_changed()
        return self.bridge.poll()

    def deliver(self, frame: HapticFrame, now: float) -> None:
        self.bridge.send(frame, now)

    def display(self, sensor_id: str) -> SensorDisplay:
        if self.bridge.state is not LinkState.CONNECTED:
            return SensorDisplay("no data", None, "none")
        r = self.bridge.last_readings.get(sensor_id)
        if r is None:
            return SensorDisplay("waiting", None, "none")
        if r.status is RangeStatus.VALID:
            return SensorDisplay(f"{r.distance_cm:.0f} cm", r.distance_cm, "echo")
        if r.status is RangeStatus.NO_ECHO:
            return SensorDisplay("no echo", None, "no_echo")
        return SensorDisplay("ERROR", None, "error")

    def title(self) -> str:
        st = self.bridge.status()
        if st.state is not LinkState.CONNECTED:
            return f"ULTRASONIC: DISCONNECTED ({st.state.value}): no data"
        where = st.port or "serial"
        if self.kind is SourceKind.SIMULATED_VIRTUAL:
            return f"ULTRASONIC: SIMULATED (virtual Arduino {where}, fw {st.firmware_version})"
        return f"ULTRASONIC: REAL (Arduino {where}, fw {st.firmware_version})"

    def bridge_status(self) -> BridgeStatus | None:
        return self.bridge.status()

    def close(self) -> None:
        self.bridge.close()
        if self.virtual is not None:
            self.virtual.close()
