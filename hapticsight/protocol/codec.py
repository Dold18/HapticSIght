"""Encoding / decoding of protocol v1 lines, with strict validation.

Malformed input never raises anything other than :class:`ProtocolError`
subclasses, and never yields a partially-valid message. Receivers must discard
the line and keep their previous state (see docs/PROTOCOL.md §Errors).
"""

from __future__ import annotations

import re

from hapticsight.protocol.messages import (
    CHANNEL_COUNT,
    MAX_LINE_BYTES,
    MAX_RANGE_FIELD_CM,
    MAX_TTL_MS,
    PATTERN_SLOTS,
    SEQ_MODULO,
    TIME_MODULO,
    FirmwareMode,
    HapticCommandMessage,
    HelloMessage,
    Message,
    PatternDefinitionMessage,
    RangeCode,
    StatusMessage,
    UltrasonicMessage,
)


class ProtocolError(ValueError):
    """Base class for all decode/encode failures."""


class FramingError(ProtocolError):
    pass


class ChecksumError(ProtocolError):
    pass


class FieldError(ProtocolError):
    pass


class UnknownMessageError(ProtocolError):
    pass


_FW_VERSION_RE = re.compile(r"^[A-Za-z0-9._-]{1,12}$")
_UINT_RE = re.compile(r"^[0-9]{1,10}$")
_PATTERNS_RE = re.compile(r"^[0-9A-F]{8}$")


def checksum(body: str) -> int:
    value = 0
    for byte in body.encode("ascii"):
        value ^= byte
    return value


def frame(body: str) -> str:
    """Wrap a message body as a full protocol line (including '\\n')."""
    line = f"${body}*{checksum(body):02X}\n"
    if len(line) > MAX_LINE_BYTES:
        raise FramingError(f"line exceeds {MAX_LINE_BYTES} bytes: {line!r}")
    return line


def _uint(token: str, name: str, maximum: int, minimum: int = 0) -> int:
    if not _UINT_RE.match(token):
        raise FieldError(f"{name}: expected unsigned integer, got {token!r}")
    value = int(token)
    if not minimum <= value <= maximum:
        raise FieldError(f"{name}: {value} outside [{minimum}, {maximum}]")
    return value


def _range_token(token: str) -> int | RangeCode:
    if token == RangeCode.NO_ECHO.value:
        return RangeCode.NO_ECHO
    if token == RangeCode.ERROR.value:
        return RangeCode.ERROR
    return _uint(token, "range", MAX_RANGE_FIELD_CM)


# ---------------------------------------------------------------- encoding


def encode(msg: Message) -> str:
    """Serialise a message to a protocol line. Validates field ranges."""
    if isinstance(msg, HapticCommandMessage):
        if len(msg.patterns) != CHANNEL_COUNT:
            raise FieldError(f"HAP needs {CHANNEL_COUNT} channels")
        if any(not 0 <= p < PATTERN_SLOTS for p in msg.patterns):
            raise FieldError("pattern slot out of range")
        if not 0 <= msg.seq < SEQ_MODULO or not 1 <= msg.ttl_ms <= MAX_TTL_MS:
            raise FieldError("HAP seq or ttl out of range")
        pats = "".join(f"{p:X}" for p in msg.patterns)
        return frame(f"HAP,{msg.seq},{msg.ttl_ms},{pats}")
    if isinstance(msg, PatternDefinitionMessage):
        _check_pattern_fields(msg)
        return frame(f"PAT,{msg.slot},{msg.on_ms},{msg.off_ms},{msg.count},{msg.pause_ms}")
    if isinstance(msg, UltrasonicMessage):
        if not 0 <= msg.seq < SEQ_MODULO or not 0 <= msg.arduino_ms < TIME_MODULO:
            raise FieldError("US seq or time out of range")
        toks = []
        for r in msg.ranges:
            if isinstance(r, RangeCode):
                toks.append(r.value)
            elif 0 <= r <= MAX_RANGE_FIELD_CM:
                toks.append(str(r))
            else:
                raise FieldError(f"range {r} out of range")
        return frame(f"US,{msg.seq},{msg.arduino_ms},{','.join(toks)}")
    if isinstance(msg, StatusMessage):
        last = "-" if msg.last_hap_seq is None else str(msg.last_hap_seq)
        return frame(
            f"ST,{msg.seq},{msg.arduino_ms},{msg.mode.value},{last},{min(msg.rx_errors, 65535)}"
        )
    if isinstance(msg, HelloMessage):
        if not _FW_VERSION_RE.match(msg.firmware_version):
            raise FieldError("invalid firmware version token")
        return frame(f"HLO,{msg.protocol_version},{msg.firmware_version}")
    raise UnknownMessageError(f"cannot encode {type(msg).__name__}")


def _check_pattern_fields(msg: PatternDefinitionMessage) -> None:
    if not 1 <= msg.slot < PATTERN_SLOTS:
        raise FieldError("PAT slot must be 1..15 (slot 0 is permanently OFF)")
    for name in ("on_ms", "off_ms", "pause_ms"):
        if not 0 <= getattr(msg, name) <= 65535:
            raise FieldError(f"PAT {name} out of range")
    if not 0 <= msg.count <= 9:
        raise FieldError("PAT count must be 0..9")
    if msg.count > 0 and msg.on_ms == 0:
        raise FieldError("PAT with pulses needs on_ms > 0")


# ---------------------------------------------------------------- decoding


def decode(line: str | bytes) -> Message:
    """Parse and validate one protocol line (trailing CR/LF optional)."""
    if isinstance(line, bytes):
        try:
            text = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise FramingError("non-ASCII bytes in line") from exc
    else:
        text = line
    text = text.rstrip("\r\n")
    if len(text) + 1 > MAX_LINE_BYTES:
        raise FramingError("line too long")
    if not text.isprintable():
        raise FramingError("non-printable characters in line")
    if not text.startswith("$"):
        raise FramingError("missing '$' start marker")
    star = text.rfind("*")
    if star == -1 or len(text) - star != 3:
        raise FramingError("missing or malformed '*CS' checksum suffix")
    body, cs_text = text[1:star], text[star + 1 :]
    try:
        received = int(cs_text, 16)
    except ValueError as exc:
        raise FramingError(f"checksum is not hex: {cs_text!r}") from exc
    if received != checksum(body):
        raise ChecksumError(f"checksum mismatch: got {received:02X}, want {checksum(body):02X}")

    fields = body.split(",")
    kind, args = fields[0], fields[1:]

    def expect(n: int) -> None:
        if len(args) != n:
            raise FieldError(f"{kind}: expected {n} fields, got {len(args)}")

    if kind == "US":
        expect(6)
        r = tuple(_range_token(t) for t in args[2:6])
        return UltrasonicMessage(
            seq=_uint(args[0], "seq", SEQ_MODULO - 1),
            arduino_ms=_uint(args[1], "t_ms", TIME_MODULO - 1),
            ranges=(r[0], r[1], r[2], r[3]),
        )
    if kind == "ST":
        expect(5)
        try:
            mode = FirmwareMode(args[2])
        except ValueError as exc:
            raise FieldError(f"ST: unknown mode {args[2]!r}") from exc
        return StatusMessage(
            seq=_uint(args[0], "seq", SEQ_MODULO - 1),
            arduino_ms=_uint(args[1], "t_ms", TIME_MODULO - 1),
            mode=mode,
            last_hap_seq=None if args[3] == "-" else _uint(args[3], "last_hap", SEQ_MODULO - 1),
            rx_errors=_uint(args[4], "rx_errors", 65535),
        )
    if kind == "HLO":
        expect(2)
        if not _FW_VERSION_RE.match(args[1]):
            raise FieldError("HLO: invalid firmware version token")
        return HelloMessage(_uint(args[0], "version", 255), args[1])
    if kind == "HAP":
        expect(3)
        if not _PATTERNS_RE.match(args[2]):
            raise FieldError("HAP: patterns must be 8 uppercase hex digits")
        return HapticCommandMessage(
            seq=_uint(args[0], "seq", SEQ_MODULO - 1),
            ttl_ms=_uint(args[1], "ttl_ms", MAX_TTL_MS, minimum=1),
            patterns=tuple(int(c, 16) for c in args[2]),
        )
    if kind == "PAT":
        expect(5)
        msg = PatternDefinitionMessage(
            slot=_uint(args[0], "slot", PATTERN_SLOTS - 1),
            on_ms=_uint(args[1], "on_ms", 65535),
            off_ms=_uint(args[2], "off_ms", 65535),
            count=_uint(args[3], "count", 9),
            pause_ms=_uint(args[4], "pause_ms", 65535),
        )
        _check_pattern_fields(msg)
        return msg
    raise UnknownMessageError(f"unknown message type {kind!r}")
