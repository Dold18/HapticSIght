"""Byte-stream framing and link-health monitoring (host side)."""

from __future__ import annotations

from dataclasses import dataclass, field

from hapticsight.protocol.codec import FramingError, ProtocolError, decode
from hapticsight.protocol.messages import (
    MAX_LINE_BYTES,
    SEQ_MODULO,
    FirmwareMode,
    Message,
    StatusMessage,
    UltrasonicMessage,
)


@dataclass
class DecodeResult:
    messages: list[Message] = field(default_factory=list)
    errors: list[ProtocolError] = field(default_factory=list)
    # Every complete line in arrival order with its outcome (for monitors/logs).
    lines: list[tuple[bytes, Message | ProtocolError]] = field(default_factory=list)


class StreamDecoder:
    """Splits an incoming byte stream into lines and decodes them.

    * Lines end with ``\\n``; a preceding ``\\r`` is tolerated; blank lines are
      ignored.
    * If more than ``MAX_LINE_BYTES`` arrive without a newline, the partial line
      is discarded, one FramingError is reported, and input is skipped until the
      next newline (resynchronisation).
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._discarding = False

    def feed(self, data: bytes) -> DecodeResult:
        result = DecodeResult()
        for b in data:
            if b == 0x0A:  # '\n'
                if self._discarding:
                    self._discarding = False
                elif self._buf.strip():
                    raw = bytes(self._buf)
                    try:
                        msg = decode(raw)
                    except ProtocolError as exc:
                        result.errors.append(exc)
                        result.lines.append((raw, exc))
                    else:
                        result.messages.append(msg)
                        result.lines.append((raw, msg))
                self._buf.clear()
                continue
            if self._discarding:
                continue
            self._buf.append(b)
            if len(self._buf) >= MAX_LINE_BYTES:
                overflow = FramingError("line overflow; resynchronising")
                result.lines.append((bytes(self._buf[:16]) + b"...", overflow))
                self._buf.clear()
                self._discarding = True
                result.errors.append(overflow)
        return result


class LinkMonitor:
    """Tracks whether the peer is alive and counts lost / bad frames.

    The Arduino sends US frames at the sensor rate and an ST heartbeat at least
    every 250 ms; the link is considered down after ``timeout_s`` without any
    *valid* message. Times are host monotonic seconds.
    """

    def __init__(self, timeout_s: float = 0.75) -> None:
        self.timeout_s = timeout_s
        self.last_rx: float | None = None
        self.firmware_mode: FirmwareMode | None = None
        self.dropped_us_frames = 0
        self.decode_errors = 0
        self._last_us_seq: int | None = None

    def on_message(self, msg: Message, now: float) -> None:
        self.last_rx = now
        if isinstance(msg, UltrasonicMessage):
            if self._last_us_seq is not None:
                gap = (msg.seq - self._last_us_seq) % SEQ_MODULO
                if gap > 1:
                    self.dropped_us_frames += gap - 1
            self._last_us_seq = msg.seq
        elif isinstance(msg, StatusMessage):
            self.firmware_mode = msg.mode

    def restart_sequence(self) -> None:
        """Forget the last US sequence number (after a board reset or new session)."""
        self._last_us_seq = None

    def on_error(self) -> None:
        self.decode_errors += 1

    def is_alive(self, now: float) -> bool:
        return self.last_rx is not None and now - self.last_rx <= self.timeout_s
