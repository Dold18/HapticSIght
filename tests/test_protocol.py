from __future__ import annotations

import pytest

from hapticsight.protocol.codec import (
    ChecksumError,
    FieldError,
    FramingError,
    ProtocolError,
    UnknownMessageError,
    checksum,
    decode,
    encode,
    frame,
)
from hapticsight.protocol.link import LinkMonitor, StreamDecoder
from hapticsight.protocol.messages import (
    MAX_LINE_BYTES,
    FirmwareMode,
    HapticCommandMessage,
    HelloMessage,
    PatternDefinitionMessage,
    RangeCode,
    StatusMessage,
    UltrasonicMessage,
)

ROUND_TRIP = [
    HapticCommandMessage(seq=42, ttl_ms=500, patterns=(0, 3, 0, 0, 7, 0, 0, 1)),
    HapticCommandMessage(seq=65535, ttl_ms=2000, patterns=(15,) * 8),
    PatternDefinitionMessage(slot=2, on_ms=150, off_ms=0, count=1, pause_ms=350),
    PatternDefinitionMessage(slot=15, on_ms=65535, off_ms=65535, count=9, pause_ms=65535),
    UltrasonicMessage(seq=7, arduino_ms=123456, ranges=(85, RangeCode.NO_ECHO, RangeCode.ERROR, 2)),
    UltrasonicMessage(seq=65535, arduino_ms=2**32 - 1, ranges=(999, 999, 999, 999)),
    StatusMessage(
        seq=1, arduino_ms=5000, mode=FirmwareMode.FALLBACK, last_hap_seq=None, rx_errors=3
    ),
    StatusMessage(seq=2, arduino_ms=5100, mode=FirmwareMode.HOST, last_hap_seq=41, rx_errors=0),
    HelloMessage(protocol_version=1, firmware_version="0.1.0-dev"),
]


@pytest.mark.parametrize("msg", ROUND_TRIP)
def test_round_trip(msg: object) -> None:
    line = encode(msg)  # type: ignore[arg-type]
    assert line.endswith("\n")
    assert len(line) <= MAX_LINE_BYTES
    assert decode(line) == msg
    assert decode(line.rstrip("\n") + "\r\n") == msg
    assert decode(line.encode("ascii")) == msg


def test_known_wire_format() -> None:
    """Pinned example from docs/PROTOCOL.md; firmware authors test against it."""
    msg = HapticCommandMessage(seq=42, ttl_ms=500, patterns=(0, 3, 0, 0, 7, 0, 0, 1))
    body = "HAP,42,500,03007001"
    assert encode(msg) == f"${body}*{checksum(body):02X}\n"
    assert encode(msg) == "$HAP,42,500,03007001*43\n"
    assert decode("$US,7,123456,85,X,E,2*" + f"{checksum('US,7,123456,85,X,E,2'):02X}") == (
        UltrasonicMessage(7, 123456, (85, RangeCode.NO_ECHO, RangeCode.ERROR, 2))
    )


@pytest.mark.parametrize(
    ("line", "error"),
    [
        ("HAP,1,500,00000000*00", FramingError),  # no '$'
        ("$HAP,1,500,00000000", FramingError),  # no checksum
        ("$HAP,1,500,00000000*0", FramingError),  # short checksum
        ("$HAP,1,500,00000000*ZZ", FramingError),
        ("$HAP,1,500,00000000*00", ChecksumError),
        ("$" + "A" * 70 + "*00", FramingError),  # too long
        ("$HAP,1\x00,500*00", FramingError),  # non-printable
    ],
)
def test_framing_errors(line: str, error: type[ProtocolError]) -> None:
    with pytest.raises(error):
        decode(line)


def framed(body: str) -> str:
    return f"${body}*{checksum(body):02X}"


@pytest.mark.parametrize(
    "body",
    [
        "HAP,1,500",  # field count
        "HAP,1,500,0000000",  # 7 channels
        "HAP,1,500,0000000g",  # bad hex
        "HAP,1,500,0000000a",  # lowercase hex not allowed
        "HAP,1,0,00000000",  # ttl must be >= 1
        "HAP,1,2001,00000000",  # ttl above safety cap
        "HAP,65536,500,00000000",  # seq overflow
        "HAP,-1,500,00000000",
        "PAT,0,100,0,1,0",  # slot 0 is fixed OFF
        "PAT,3,0,0,1,100",  # pulses need on time
        "PAT,3,100,0,10,100",  # count limit
        "US,1,100,85,X,E",  # 3 ranges
        "US,1,100,85,X,E,1000",  # range field limit
        "US,1,100,85,X,E,-5",
        "US,1,100,85,X,Q,5",  # unknown token
        "ST,1,100,Z,-,0",  # unknown mode
        "HLO,1,bad version!",
    ],
)
def test_field_errors(body: str) -> None:
    with pytest.raises(FieldError):
        decode(framed(body))


def test_unknown_message_type() -> None:
    with pytest.raises(UnknownMessageError):
        decode(framed("FOO,1"))


def test_encode_rejects_invalid_messages() -> None:
    with pytest.raises(FieldError):
        encode(HapticCommandMessage(seq=1, ttl_ms=500, patterns=(0,) * 7))
    with pytest.raises(FieldError):
        encode(HapticCommandMessage(seq=1, ttl_ms=5000, patterns=(0,) * 8))
    with pytest.raises(FieldError):
        encode(PatternDefinitionMessage(slot=0, on_ms=1, off_ms=0, count=1, pause_ms=0))
    with pytest.raises(FieldError):
        encode(UltrasonicMessage(seq=1, arduino_ms=0, ranges=(1000, 0, 0, 0)))
    with pytest.raises(FramingError):
        frame("X" * 80)


# ------------------------------------------------------------ stream + link


def test_stream_decoder_handles_split_and_garbage() -> None:
    good = encode(HelloMessage(1, "fw")).encode()
    dec = StreamDecoder()
    r1 = dec.feed(good[:5])
    assert r1.messages == [] and r1.errors == []
    r2 = dec.feed(good[5:] + b"\n\r\n$garbage*00\n")
    assert r2.messages == [HelloMessage(1, "fw")]
    assert len(r2.errors) == 1


def test_stream_decoder_resynchronises_after_overflow() -> None:
    dec = StreamDecoder()
    good = encode(HelloMessage(1, "fw")).encode()
    result = dec.feed(b"x" * 200 + b"\n" + good)
    assert result.messages == [HelloMessage(1, "fw")]
    assert len(result.errors) == 1  # one overflow error, not one per byte


def test_link_monitor_timeout_and_dropped_frames() -> None:
    link = LinkMonitor(timeout_s=0.5)
    assert not link.is_alive(0.0)
    link.on_message(UltrasonicMessage(65534, 0, (1, 1, 1, 1)), 1.0)
    link.on_message(UltrasonicMessage(1, 0, (1, 1, 1, 1)), 1.1)  # wrapped, lost 65535 & 0
    assert link.dropped_us_frames == 2
    assert link.is_alive(1.6)
    assert not link.is_alive(1.61)
    link.on_message(StatusMessage(0, 0, FirmwareMode.FALLBACK, None, 0), 2.0)
    assert link.firmware_mode is FirmwareMode.FALLBACK


# Arduino bridge (connection state machine) tests live in tests/test_arduino_bridge.py.
