"""HapticSight serial protocol v1 message types.

See docs/PROTOCOL.md for the full specification. Summary::

    $<TYPE>,<field>,...*<CS>\\n      CS = XOR of bytes between '$' and '*', 2 hex digits

    Arduino -> host:  HLO (boot/version), US (ultrasonic ranges), ST (status/heartbeat)
    Host -> Arduino:  HAP (8-channel pattern command with TTL), PAT (pattern definition)
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

PROTOCOL_VERSION = 1
MAX_LINE_BYTES = 64  # including the trailing '\n'; matches Uno's 64-byte RX buffer
MAX_TTL_MS = 2000  # firmware must reject longer TTLs so motors can never latch on
SEQ_MODULO = 65536
TIME_MODULO = 2**32  # Arduino millis() wraps after ~49.7 days
MAX_RANGE_FIELD_CM = 999
CHANNEL_COUNT = 8
PATTERN_SLOTS = 16  # hex digit per channel


class RangeCode(StrEnum):
    """Non-numeric ultrasonic range tokens."""

    NO_ECHO = "X"
    ERROR = "E"


class FirmwareMode(StrEnum):
    BOOT = "B"  # waiting for host, motors off
    HOST = "H"  # executing host HAP commands
    FALLBACK = "F"  # host link lost: local ultrasonic-only warnings


@dataclass(frozen=True, slots=True)
class HelloMessage:
    protocol_version: int
    firmware_version: str


@dataclass(frozen=True, slots=True)
class UltrasonicMessage:
    seq: int
    arduino_ms: int
    # One entry per sensor S0..S3: integer cm, or a RangeCode
    ranges: tuple[int | RangeCode, int | RangeCode, int | RangeCode, int | RangeCode]


@dataclass(frozen=True, slots=True)
class StatusMessage:
    seq: int
    arduino_ms: int
    mode: FirmwareMode
    last_hap_seq: int | None
    rx_errors: int


@dataclass(frozen=True, slots=True)
class HapticCommandMessage:
    seq: int
    ttl_ms: int
    patterns: tuple[int, ...]  # 8 pattern slot ids, M0 first


@dataclass(frozen=True, slots=True)
class PatternDefinitionMessage:
    slot: int
    on_ms: int
    off_ms: int
    count: int
    pause_ms: int


Message = (
    HelloMessage
    | UltrasonicMessage
    | StatusMessage
    | HapticCommandMessage
    | PatternDefinitionMessage
)
