"""Host-side Arduino bridge (protocol v1) over an injected byte transport.

Not yet validated against real firmware -- there is no firmware yet. It is
exercised in tests with an in-memory transport so the host half of the
contract is ready when the hardware is.

Responsibilities:

* Decode incoming lines -> :class:`UltrasonicReading` stamped with *host*
  monotonic receive time (the Arduino clock is only used for ordering/logs).
* Send the pattern table (``$PAT``) on start and whenever the board reboots
  (``$HLO``), then ``$HAP`` frames with a TTL.
* Refuse to drive motors if the firmware reports an incompatible protocol.
"""

from __future__ import annotations

from collections.abc import Mapping

from hapticsight.core.clock import Clock
from hapticsight.core.layout import ULTRASONIC_SENSORS
from hapticsight.core.models import RangeStatus, UltrasonicReading
from hapticsight.haptics.encoder import HapticFrame
from hapticsight.haptics.patterns import DEFAULT_PATTERNS, PatternId, PulsePattern
from hapticsight.hardware.interfaces import ByteTransport
from hapticsight.protocol.codec import encode
from hapticsight.protocol.link import LinkMonitor, StreamDecoder
from hapticsight.protocol.messages import (
    MAX_TTL_MS,
    PROTOCOL_VERSION,
    SEQ_MODULO,
    HapticCommandMessage,
    HelloMessage,
    PatternDefinitionMessage,
    RangeCode,
    UltrasonicMessage,
)


class ArduinoBridge:
    def __init__(
        self,
        transport: ByteTransport,
        clock: Clock,
        patterns: Mapping[PatternId, PulsePattern] = DEFAULT_PATTERNS,
        link_timeout_s: float = 0.5,
    ) -> None:
        self.transport = transport
        self.clock = clock
        self.patterns = patterns
        self.link = LinkMonitor(link_timeout_s)
        self.firmware_version: str | None = None
        self.incompatible = False
        self._decoder = StreamDecoder()
        self._hap_seq = 0

    def start(self) -> None:
        self._send_pattern_table()

    def _send_pattern_table(self) -> None:
        for pid, p in sorted(self.patterns.items()):
            if pid is PatternId.OFF:
                continue  # slot 0 is fixed OFF in firmware
            msg = PatternDefinitionMessage(int(pid), p.on_ms, p.off_ms, p.count, p.pause_ms)
            self.transport.write(encode(msg).encode("ascii"))

    def poll(self) -> list[UltrasonicReading]:
        now = self.clock.now()
        result = self._decoder.feed(self.transport.read_available())
        for _ in result.errors:
            self.link.on_error()
        readings: list[UltrasonicReading] = []
        for msg in result.messages:
            self.link.on_message(msg, now)
            if isinstance(msg, HelloMessage):
                self.firmware_version = msg.firmware_version
                self.incompatible = msg.protocol_version != PROTOCOL_VERSION
                if not self.incompatible:
                    self._send_pattern_table()  # board rebooted: re-sync table
            elif isinstance(msg, UltrasonicMessage):
                for spec, token in zip(ULTRASONIC_SENSORS, msg.ranges, strict=True):
                    readings.append(_to_reading(spec.sensor_id, token, now))
        return readings

    def send(self, frame: HapticFrame) -> None:
        if self.incompatible:
            return
        msg = HapticCommandMessage(
            seq=self._hap_seq,
            ttl_ms=min(frame.ttl_ms, MAX_TTL_MS),
            patterns=tuple(int(p) for p in frame.pattern_ids),
        )
        self._hap_seq = (self._hap_seq + 1) % SEQ_MODULO
        self.transport.write(encode(msg).encode("ascii"))


def _to_reading(sensor_id: str, token: int | RangeCode, now: float) -> UltrasonicReading:
    if token is RangeCode.NO_ECHO:
        return UltrasonicReading(sensor_id, now, RangeStatus.NO_ECHO)
    if token is RangeCode.ERROR:
        return UltrasonicReading(sensor_id, now, RangeStatus.INVALID)
    assert isinstance(token, int)
    return UltrasonicReading(sensor_id, now, RangeStatus.VALID, float(token))
