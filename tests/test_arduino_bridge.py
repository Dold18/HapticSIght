"""ArduinoBridge connection state machine, with an in-memory scripted transport."""

from __future__ import annotations

import pytest

from hapticsight.core.clock import ManualClock
from hapticsight.core.models import RangeStatus
from hapticsight.haptics.patterns import DEFAULT_PATTERNS
from hapticsight.hardware.arduino import ArduinoBridge, LinkState
from hapticsight.hardware.serial_transport import TransportError
from hapticsight.protocol.codec import decode, encode
from hapticsight.protocol.messages import (
    FirmwareMode,
    HapticCommandMessage,
    HelloMessage,
    PatternDefinitionMessage,
    RangeCode,
    StatusMessage,
    UltrasonicMessage,
)


class FakeTransport:
    def __init__(self, resets_on_open: bool = True, fail_open: bool = False) -> None:
        self.incoming = bytearray()
        self.written: list[str] = []
        self.resets = 0
        self.closed = False
        self.resets_on_open = resets_on_open
        self.fail_open = fail_open
        self.fail_reads = False
        self.is_open = False

    def open(self) -> None:
        if self.fail_open:
            raise TransportError("cannot open /dev/fake: no such device")
        self.is_open = True

    def read_available(self) -> bytes:
        if self.fail_reads:
            raise TransportError("device unplugged")
        data, self.incoming = bytes(self.incoming), bytearray()
        return data

    def write(self, data: bytes) -> None:
        self.written.append(data.decode("ascii"))

    def pulse_reset(self) -> None:
        self.resets += 1

    def close(self) -> None:
        self.closed = True

    def feed(self, *msgs: object) -> None:
        for m in msgs:
            self.incoming += encode(m).encode()  # type: ignore[arg-type]

    def sent(self, kind: type) -> list[object]:
        return [m for m in (decode(x) for x in self.written) if isinstance(m, kind)]


HELLO = HelloMessage(1, "0.3.0")
US = UltrasonicMessage(1, 1000, (85, RangeCode.NO_ECHO, RangeCode.ERROR, 300))


def status(seq: int = 0, t: int = 1000, mode: FirmwareMode = FirmwareMode.HOST) -> StatusMessage:
    return StatusMessage(seq, t, mode, None, 0)


def make(transport: FakeTransport | None = None, **kw: object):  # type: ignore[no-untyped-def]
    t = transport or FakeTransport()
    clock = ManualClock(10.0)
    events: list[tuple[str, dict[str, object]]] = []
    bridge = ArduinoBridge(t, clock, on_event=lambda k, _t, d: events.append((k, d)), **kw)  # type: ignore[arg-type]
    return bridge, t, clock, events


def connect(bridge: ArduinoBridge, t: FakeTransport, clock: ManualClock) -> None:
    bridge.poll()  # opens the port
    t.feed(HELLO)
    bridge.poll()
    assert bridge.state is LinkState.CONNECTED


def test_silent_until_hello_and_pre_handshake_data_discarded() -> None:
    bridge, t, _clock, events = make()
    assert bridge.poll() == []
    assert bridge.state is LinkState.AWAITING_HELLO
    t.feed(status(), US)  # alive, but compatibility not yet proven
    assert bridge.poll() == []
    assert bridge.discarded_before_handshake == 2
    assert t.written == []  # nothing sent while the board may be in its bootloader
    assert not bridge.send_patterns((1,) * 8)
    t.feed(HELLO, US)
    readings = bridge.poll()
    assert bridge.state is LinkState.CONNECTED
    assert bridge.firmware_version == "0.3.0"
    pats = t.sent(PatternDefinitionMessage)
    assert len(pats) == len(DEFAULT_PATTERNS) - 1  # every slot except fixed OFF
    assert [(r.sensor_id, r.status, r.distance_cm) for r in readings] == [
        ("S0", RangeStatus.VALID, 85.0),
        ("S1", RangeStatus.NO_ECHO, None),
        ("S2", RangeStatus.INVALID, None),
        ("S3", RangeStatus.VALID, 300.0),
    ]
    assert all(r.timestamp == 10.0 for r in readings)  # host receive time
    assert ("link_state", {"state": "connected", "detail": "firmware 0.3.0"}) in events
    assert any(k == "rx_line" for k, _ in events)


def test_no_hello_triggers_reset_then_no_handshake_then_retry() -> None:
    bridge, t, clock, _ = make(hello_timeout_s=2.0, reset_attempts=1, reconnect_interval_s=1.0)
    bridge.poll()
    clock.advance(2.1)
    bridge.poll()
    assert t.resets == 1 and bridge.state is LinkState.AWAITING_HELLO
    clock.advance(2.1)
    bridge.poll()
    assert bridge.state is LinkState.NO_HANDSHAKE
    assert t.closed and "no $HLO" in bridge.detail
    clock.advance(1.1)
    bridge.poll()  # reconnect attempt
    assert bridge.state is LinkState.AWAITING_HELLO
    assert bridge.reconnects == 1


def test_transport_without_auto_reset_is_reset_on_open() -> None:
    bridge, t, _clock, _ = make(FakeTransport(resets_on_open=False))
    bridge.poll()
    assert t.resets == 1


def test_incompatible_firmware_never_receives_commands() -> None:
    bridge, t, _clock, _ = make()
    bridge.poll()
    t.feed(HelloMessage(2, "9.9.9"))
    bridge.poll()
    assert bridge.state is LinkState.INCOMPATIBLE
    assert "protocol v2" in bridge.detail
    t.feed(US)
    assert bridge.poll() == []
    assert not bridge.send_patterns((3,) * 8)
    assert t.written == []


def test_hap_scheduling_change_heartbeat_and_rate_limit() -> None:
    bridge, t, clock, _ = make(hap_interval_s=0.1, min_hap_interval_s=0.02)
    connect(bridge, t, clock)
    t.written.clear()
    a, b = (2, 0, 0, 0, 0, 0, 0, 0), (3, 0, 0, 0, 0, 0, 0, 0)
    assert bridge.send_patterns(a, 500)  # t=0: first command, immediate
    clock.advance(0.01)
    assert not bridge.send_patterns(b, 500)  # t=10 ms: changed, but < 20 ms rate limit
    clock.advance(0.015)
    assert bridge.send_patterns(b, 500)  # t=25 ms: changed and >= 20 ms -> immediate
    clock.advance(0.05)
    assert not bridge.send_patterns(b, 500)  # t=75 ms: unchanged, heartbeat not due
    clock.advance(0.06)
    assert bridge.send_patterns(b, 500)  # t=135 ms: 110 ms since last -> heartbeat
    haps = t.sent(HapticCommandMessage)
    assert [h.seq for h in haps] == [0, 1, 2]  # type: ignore[attr-defined]
    assert haps[1].patterns[0] == 3 and haps[1].ttl_ms == 500  # type: ignore[attr-defined]


def test_ttl_is_clamped_and_seq_wraps() -> None:
    bridge, t, clock, _ = make()
    connect(bridge, t, clock)
    bridge._hap_seq = 65535
    bridge.send_patterns((0,) * 8, 9999)
    clock.advance(0.2)
    bridge.send_patterns((0,) * 8, 500)
    haps = t.sent(HapticCommandMessage)
    assert [(h.seq, h.ttl_ms) for h in haps] == [(65535, 2000), (0, 500)]  # type: ignore[attr-defined]


def test_commands_can_be_paused_for_link_loss_demo() -> None:
    bridge, t, clock, _ = make()
    connect(bridge, t, clock)
    bridge.commands_enabled = False
    t.written.clear()
    assert not bridge.send_patterns((2,) * 8)
    assert t.written == []
    t.feed(status(mode=FirmwareMode.FALLBACK))
    bridge.poll()
    assert bridge.firmware_mode is FirmwareMode.FALLBACK  # still observed


def test_status_tracking_and_reset_detection() -> None:
    bridge, t, clock, events = make()
    connect(bridge, t, clock)
    t.feed(StatusMessage(5, 50_000, FirmwareMode.HOST, 41, 3))
    bridge.poll()
    st = bridge.status()
    assert st.firmware_mode is FirmwareMode.HOST
    assert st.last_hap_seq_acked == 41 and st.firmware_rx_errors == 3
    pats_before = len(t.sent(PatternDefinitionMessage))
    t.feed(StatusMessage(0, 120, FirmwareMode.BOOT, None, 0))  # clock went backwards
    bridge.poll()
    assert bridge.resets_seen == 1
    assert len(t.sent(PatternDefinitionMessage)) == 2 * pats_before  # resynced
    t.feed(HELLO)  # an explicit $HLO during a session is also a reset
    bridge.poll()
    assert bridge.resets_seen == 2
    assert any(k == "reset_detected" for k, _ in events)


def test_link_lost_then_reconnect() -> None:
    bridge, t, clock, _ = make(link_timeout_s=0.75, reconnect_interval_s=1.0)
    connect(bridge, t, clock)
    clock.advance(0.7)
    bridge.poll()
    assert bridge.state is LinkState.CONNECTED
    clock.advance(0.1)
    bridge.poll()
    assert bridge.state is LinkState.LINK_LOST
    assert bridge.last_readings == {}
    clock.advance(1.0)
    bridge.poll()
    assert bridge.state is LinkState.AWAITING_HELLO
    t.feed(HELLO)
    bridge.poll()
    assert bridge.state is LinkState.CONNECTED
    assert bridge.reconnects == 1


def test_transport_errors_disconnect_and_factory_reopens() -> None:
    made: list[FakeTransport] = []

    def factory() -> FakeTransport:
        tr = FakeTransport()
        made.append(tr)
        return tr

    clock = ManualClock()
    bridge = ArduinoBridge(factory, clock, reconnect_interval_s=0.5)
    bridge.poll()
    made[0].feed(HELLO)
    bridge.poll()
    made[0].fail_reads = True
    bridge.poll()
    assert bridge.state is LinkState.DISCONNECTED and "unplugged" in bridge.detail
    clock.advance(0.5)
    bridge.poll()
    assert len(made) == 2 and bridge.state is LinkState.AWAITING_HELLO


def test_open_failure_is_reported_and_retried() -> None:
    attempts = []

    def factory() -> FakeTransport:
        attempts.append(1)
        return FakeTransport(fail_open=True)

    clock = ManualClock()
    bridge = ArduinoBridge(factory, clock, reconnect_interval_s=0.5)
    bridge.poll()
    assert bridge.state is LinkState.DISCONNECTED and "no such device" in bridge.detail
    bridge.poll()
    assert len(attempts) == 1  # waits for the retry interval
    clock.advance(0.5)
    bridge.poll()
    assert len(attempts) == 2


def test_malformed_lines_counted_not_fatal() -> None:
    bridge, t, clock, _ = make()
    connect(bridge, t, clock)
    t.incoming += b"$US,garbage*00\n" + b"x" * 200 + b"\n"
    t.feed(US)
    readings = bridge.poll()
    assert len(readings) == 4
    assert bridge.status().decode_errors == 2


def test_close_sends_all_off_best_effort() -> None:
    bridge, t, clock, _ = make()
    connect(bridge, t, clock)
    t.written.clear()
    bridge.close()
    (hap,) = t.sent(HapticCommandMessage)
    assert hap.patterns == (0,) * 8  # type: ignore[attr-defined]
    assert t.closed and bridge.state is LinkState.DISCONNECTED


def test_us_rate_estimate() -> None:
    bridge, t, clock, _ = make()
    connect(bridge, t, clock)
    for i in range(20):
        t.feed(UltrasonicMessage(i, i * 200, (100, 100, 100, 100)))
        bridge.poll()
        clock.advance(0.2)
    assert bridge.status().us_rate_hz == pytest.approx(5.0, rel=0.01)


def test_rejects_non_transport() -> None:
    with pytest.raises(TypeError):
        ArduinoBridge(42, ManualClock())
