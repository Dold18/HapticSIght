"""Host-side Arduino bridge for protocol v1: connection state machine + I/O.

Lifecycle (``LinkState``)::

    DISCONNECTED --open ok--> AWAITING_HELLO --$HLO v1--> CONNECTED
         ^   (retry every          | no $HLO: DTR reset, wait again       |
         |    reconnect_interval)  v                                      | no valid
         +---------------------- NO_HANDSHAKE                             | message for
         +------------------------------------------------- LINK_LOST <---+ link_timeout
    $HLO with another protocol version -> INCOMPATIBLE (never drives motors)

Rules that keep demonstrations honest and the hardware safe:

* **Silent until handshake.** Opening the port resets an Uno; bytes sent while
  the bootloader runs can delay start-up, so nothing is written before a valid
  ``$HLO``. ``$US``/``$ST`` frames received before the handshake are counted
  and discarded: they do not prove the firmware is compatible.
* **Pattern resync** (``$PAT`` for every defined slot) after each ``$HLO``, and
  after a probable reset detected from the status clock going backwards.
* **Command scheduling:** a ``$HAP`` is sent immediately when the motor
  pattern changes (rate-limited to ``min_hap_interval_s``) and otherwise every
  ``hap_interval_s`` as a heartbeat; each carries a TTL so the firmware falls
  back to ultrasonic-only mode if the host goes quiet.
* Readings are stamped with **host** monotonic receive time.
* ``commands_enabled = False`` stops sending ``$HAP`` (link-loss demos) while
  still receiving telemetry.
"""

from __future__ import annotations

import contextlib
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from hapticsight.core.clock import Clock
from hapticsight.core.layout import ULTRASONIC_SENSORS
from hapticsight.core.models import RangeStatus, UltrasonicReading
from hapticsight.haptics.encoder import HapticFrame
from hapticsight.haptics.patterns import DEFAULT_PATTERNS, PatternId, PulsePattern
from hapticsight.hardware.serial_transport import TransportError
from hapticsight.protocol.codec import ProtocolError, encode
from hapticsight.protocol.link import LinkMonitor, StreamDecoder
from hapticsight.protocol.messages import (
    MAX_TTL_MS,
    PROTOCOL_VERSION,
    SEQ_MODULO,
    FirmwareMode,
    HapticCommandMessage,
    HelloMessage,
    Message,
    PatternDefinitionMessage,
    RangeCode,
    StatusMessage,
    UltrasonicMessage,
)

OFF_PATTERNS: tuple[int, ...] = (0,) * 8


class LinkState(StrEnum):
    DISCONNECTED = "disconnected"
    AWAITING_HELLO = "awaiting_hello"
    CONNECTED = "connected"
    INCOMPATIBLE = "incompatible"
    NO_HANDSHAKE = "no_handshake"
    LINK_LOST = "link_lost"


EventHook = Callable[[str, float, dict[str, Any]], None]


@dataclass(frozen=True, slots=True)
class BridgeStatus:
    state: LinkState
    detail: str
    port: str
    firmware_version: str | None
    firmware_mode: FirmwareMode | None
    last_rx_age_s: float | None
    us_rate_hz: float
    rx_messages: int
    decode_errors: int
    dropped_us_frames: int
    discarded_before_handshake: int
    resets_seen: int
    reconnects: int
    hap_sent: int
    last_hap_seq_sent: int | None
    last_hap_seq_acked: int | None
    firmware_rx_errors: int | None
    commands_enabled: bool

    @property
    def connected(self) -> bool:
        return self.state is LinkState.CONNECTED


class ArduinoBridge:
    def __init__(
        self,
        transport: Callable[[], Any] | Any,
        clock: Clock,
        patterns: Mapping[PatternId, PulsePattern] = DEFAULT_PATTERNS,
        *,
        hello_timeout_s: float = 3.0,
        reset_attempts: int = 1,
        link_timeout_s: float = 0.75,
        reconnect_interval_s: float = 2.0,
        hap_interval_s: float = 0.1,
        min_hap_interval_s: float = 0.02,
        on_event: EventHook | None = None,
        port_label: str = "",
    ) -> None:
        # Accept a ready transport (tests) or a factory (enables reconnection).
        self._factory = _as_factory(transport)
        self.clock = clock
        self.patterns = patterns
        self.hello_timeout_s = hello_timeout_s
        self.reset_attempts = reset_attempts
        self.reconnect_interval_s = reconnect_interval_s
        self.hap_interval_s = hap_interval_s
        self.min_hap_interval_s = min_hap_interval_s
        self.on_event = on_event
        self.port_label = port_label
        self.link = LinkMonitor(link_timeout_s)
        self.commands_enabled = True

        self.state = LinkState.DISCONNECTED
        self.detail = "not started"
        self.transport: Any = None
        self.firmware_version: str | None = None
        self.firmware_mode: FirmwareMode | None = None
        self.firmware_rx_errors: int | None = None
        self.last_hap_seq_acked: int | None = None
        self.rx_messages = 0
        self.discarded_before_handshake = 0
        self.resets_seen = 0
        self.reconnects = 0
        self.hap_sent = 0
        self.last_readings: dict[str, UltrasonicReading] = {}
        self.us_rate_hz = 0.0
        self._decoder = StreamDecoder()
        self._hap_seq = 0
        self._last_hap_t: float | None = None
        self._last_hap_patterns: tuple[int, ...] | None = None
        self._next_connect = 0.0
        self._hello_deadline = 0.0
        self._resets_left = reset_attempts
        self._last_us_rx: float | None = None
        self._last_status_ms: int | None = None
        self._ever_connected = False

    # ------------------------------------------------------------ events/state

    def _emit(self, kind: str, now: float, **data: Any) -> None:
        if self.on_event is not None:
            self.on_event(kind, now, data)

    def _set_state(self, state: LinkState, detail: str, now: float) -> None:
        if state is not self.state or detail != self.detail:
            self._emit("link_state", now, state=state.value, detail=detail)
        self.state, self.detail = state, detail

    def status(self) -> BridgeStatus:
        now = self.clock.now()
        age = None if self.link.last_rx is None else max(0.0, now - self.link.last_rx)
        return BridgeStatus(
            state=self.state,
            detail=self.detail,
            port=self.port_label,
            firmware_version=self.firmware_version,
            firmware_mode=self.firmware_mode,
            last_rx_age_s=age,
            us_rate_hz=self.us_rate_hz,
            rx_messages=self.rx_messages,
            decode_errors=self.link.decode_errors,
            dropped_us_frames=self.link.dropped_us_frames,
            discarded_before_handshake=self.discarded_before_handshake,
            resets_seen=self.resets_seen,
            reconnects=self.reconnects,
            hap_sent=self.hap_sent,
            last_hap_seq_sent=(self._hap_seq - 1) % SEQ_MODULO if self.hap_sent else None,
            last_hap_seq_acked=self.last_hap_seq_acked,
            firmware_rx_errors=self.firmware_rx_errors,
            commands_enabled=self.commands_enabled,
        )

    # ------------------------------------------------------------ connection

    def _disconnect(self, state: LinkState, detail: str, now: float) -> None:
        if self.transport is not None:
            with contextlib.suppress(OSError):
                self.transport.close()
        self.transport = None
        self._decoder = StreamDecoder()
        self.firmware_mode = None
        self.last_readings.clear()
        self._next_connect = now + self.reconnect_interval_s
        self._set_state(state, detail, now)

    def _try_connect(self, now: float) -> None:
        try:
            t = self._factory()
            if hasattr(t, "open") and not getattr(t, "is_open", False):
                t.open()
        except (TransportError, OSError) as exc:
            self._next_connect = now + self.reconnect_interval_s
            self._set_state(LinkState.DISCONNECTED, str(exc), now)
            return
        if self._ever_connected or self.state in (LinkState.LINK_LOST, LinkState.NO_HANDSHAKE):
            self.reconnects += 1
        self.transport = t
        self.link = LinkMonitor(self.link.timeout_s)
        self._last_status_ms = None
        self._resets_left = self.reset_attempts
        self._hello_deadline = now + self.hello_timeout_s
        if not getattr(t, "resets_on_open", True):
            self._pulse_reset(now)
        self._set_state(LinkState.AWAITING_HELLO, "port open; waiting for $HLO", now)

    def _pulse_reset(self, now: float) -> None:
        reset = getattr(self.transport, "pulse_reset", None)
        if reset is not None:
            reset()
            self.link.restart_sequence()
            self._emit("reset_pulse", now)

    def close(self) -> None:
        now = self.clock.now()
        if self.transport is not None and self.state is LinkState.CONNECTED:
            # Best effort: switch motors off rather than waiting for the TTL.
            with contextlib.suppress(TransportError):
                self._write_hap(OFF_PATTERNS, 200, now)
        self._disconnect(LinkState.DISCONNECTED, "closed", now)
        self._next_connect = float("inf")

    # ------------------------------------------------------------ receive

    def poll(self) -> list[UltrasonicReading]:
        """Service the link; return readings received since the last call.

        Readings are only returned after a successful version handshake.
        """
        now = self.clock.now()
        if self.transport is None:
            if now >= self._next_connect:
                self._try_connect(now)
            if self.transport is None:
                return []
        try:
            data = self.transport.read_available()
        except TransportError as exc:
            self._disconnect(LinkState.DISCONNECTED, str(exc), now)
            return []
        readings: list[UltrasonicReading] = []
        result = self._decoder.feed(data)
        for raw, item in result.lines:
            self._emit("rx_line", now, line=raw.decode("ascii", "replace"),
                       ok=not isinstance(item, ProtocolError))  # fmt: skip
        for _ in result.errors:
            self.link.on_error()
        for msg in result.messages:
            self.link.on_message(msg, now)
            self.rx_messages += 1
            readings.extend(self._handle(msg, now))
        self._check_timeouts(now)
        return readings

    def _handle(self, msg: Message, now: float) -> list[UltrasonicReading]:
        if isinstance(msg, HelloMessage):
            self.link.restart_sequence()
            self._on_hello(msg, now)
            return []
        if self.state is not LinkState.CONNECTED:
            self.discarded_before_handshake += 1
            return []
        if isinstance(msg, StatusMessage):
            if self._last_status_ms is not None and msg.arduino_ms + 1000 < self._last_status_ms:
                # Status clock went backwards without a $HLO: the board probably
                # reset and the $HLO was lost. Re-sync the pattern table.
                self.resets_seen += 1
                self.link.restart_sequence()
                self._send_pattern_table(now)
                self._emit("reset_detected", now, reason="status clock went backwards")
            self._last_status_ms = msg.arduino_ms
            self.firmware_mode = msg.mode
            self.firmware_rx_errors = msg.rx_errors
            self.last_hap_seq_acked = msg.last_hap_seq
            return []
        if isinstance(msg, UltrasonicMessage):
            if self._last_us_rx is not None and now > self._last_us_rx:
                inst = 1.0 / (now - self._last_us_rx)
                self.us_rate_hz = (
                    inst if self.us_rate_hz == 0 else 0.8 * self.us_rate_hz + 0.2 * inst
                )
            self._last_us_rx = now
            out = []
            for spec, token in zip(ULTRASONIC_SENSORS, msg.ranges, strict=True):
                r = _to_reading(spec.sensor_id, token, now)
                self.last_readings[spec.sensor_id] = r
                out.append(r)
            return out
        return []  # host->Arduino types are never expected here

    def _on_hello(self, msg: HelloMessage, now: float) -> None:
        self.firmware_version = msg.firmware_version
        if msg.protocol_version != PROTOCOL_VERSION:
            self._set_state(
                LinkState.INCOMPATIBLE,
                f"firmware speaks protocol v{msg.protocol_version}, host v{PROTOCOL_VERSION}",
                now,
            )
            return
        if self.state is LinkState.CONNECTED:
            self.resets_seen += 1  # unexpected reboot during a session
            self._emit("reset_detected", now, reason="$HLO during session")
        self._ever_connected = True
        self._last_hap_patterns = None  # force an immediate command after resync
        self._last_status_ms = None
        self._set_state(LinkState.CONNECTED, f"firmware {msg.firmware_version}", now)
        self._send_pattern_table(now)

    def _check_timeouts(self, now: float) -> None:
        if self.state is LinkState.AWAITING_HELLO and now >= self._hello_deadline:
            if self._resets_left > 0:
                self._resets_left -= 1
                self._pulse_reset(now)
                self._hello_deadline = now + self.hello_timeout_s
                self._set_state(LinkState.AWAITING_HELLO, "no $HLO; reset pulse sent", now)
            else:
                self._disconnect(LinkState.NO_HANDSHAKE,
                                 "no $HLO after reset: wrong port, firmware, or baud rate?",
                                 now)  # fmt: skip
        elif self.state is LinkState.CONNECTED and not self.link.is_alive(now):
            self._disconnect(LinkState.LINK_LOST, "no valid message within link timeout", now)

    # ------------------------------------------------------------ transmit

    def _write(self, line: str, now: float) -> None:
        self.transport.write(line.encode("ascii"))
        self._emit("tx_line", now, line=line.rstrip("\n"))

    def _send_pattern_table(self, now: float) -> None:
        try:
            for pid, p in sorted(self.patterns.items()):
                if pid is PatternId.OFF:
                    continue  # slot 0 is fixed OFF in firmware
                msg = PatternDefinitionMessage(int(pid), p.on_ms, p.off_ms, p.count, p.pause_ms)
                self._write(encode(msg), now)
        except TransportError as exc:
            self._disconnect(LinkState.DISCONNECTED, str(exc), now)

    def _write_hap(self, patterns: tuple[int, ...], ttl_ms: int, now: float) -> None:
        msg = HapticCommandMessage(self._hap_seq, min(max(ttl_ms, 1), MAX_TTL_MS), patterns)
        self._write(encode(msg), now)
        self._hap_seq = (self._hap_seq + 1) % SEQ_MODULO
        self.hap_sent += 1
        self._last_hap_t = now
        self._last_hap_patterns = patterns

    def send(self, frame: HapticFrame, now: float | None = None) -> bool:
        """Send (or skip) a command according to the schedule. True if written."""
        return self.send_patterns(tuple(int(p) for p in frame.pattern_ids), frame.ttl_ms, now)

    def send_patterns(
        self, patterns: tuple[int, ...], ttl_ms: int = 500, now: float | None = None
    ) -> bool:
        now = self.clock.now() if now is None else now
        if self.state is not LinkState.CONNECTED or not self.commands_enabled:
            return False
        since = None if self._last_hap_t is None else now - self._last_hap_t
        changed = patterns != self._last_hap_patterns
        due = since is None or since >= self.hap_interval_s
        if not due and not (changed and since is not None and since >= self.min_hap_interval_s):
            return False
        try:
            self._write_hap(patterns, ttl_ms, now)
        except TransportError as exc:
            self._disconnect(LinkState.DISCONNECTED, str(exc), now)
            return False
        return True


def _as_factory(transport: Any) -> Callable[[], Any]:
    if hasattr(transport, "read_available"):  # a ready transport instance

        def same() -> Any:
            return transport

        return same
    if not callable(transport):
        raise TypeError("transport must be a ByteTransport or a zero-argument factory")
    factory: Callable[[], Any] = transport
    return factory


def _to_reading(sensor_id: str, token: int | RangeCode, now: float) -> UltrasonicReading:
    if token is RangeCode.NO_ECHO:
        return UltrasonicReading(sensor_id, now, RangeStatus.NO_ECHO)
    if token is RangeCode.ERROR:
        return UltrasonicReading(sensor_id, now, RangeStatus.INVALID)
    assert isinstance(token, int)
    return UltrasonicReading(sensor_id, now, RangeStatus.VALID, float(token))
