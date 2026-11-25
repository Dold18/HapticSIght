"""Host <-> firmware integration through the virtual Uno on a pseudo-terminal.

These run the *real* firmware logic (compiled natively), the real pyserial
transport, the ArduinoBridge state machine, the shared pipeline, the runtime,
the CLI, and the live UI. The sensors are simulated, so nothing here verifies
physical hardware.
"""

from __future__ import annotations

import os
import sys
import time
from collections.abc import Callable, Iterator
from pathlib import Path

import pytest

from hapticsight.core.clock import MonotonicClock
from hapticsight.core.models import RangeStatus
from hapticsight.hardware.arduino import ArduinoBridge, LinkState
from hapticsight.hardware.cli import main as hw_main
from hapticsight.hardware.inputs import ArduinoInput, ManualInput, SourceKind
from hapticsight.hardware.runtime import HardwareRuntime
from hapticsight.hardware.serial_transport import PySerialTransport, TransportError
from hapticsight.hardware.telemetry import TelemetryLogger, read_log
from hapticsight.hardware.virtual_arduino import VirtualArduino, find_compiler
from hapticsight.protocol.messages import FirmwareMode

pytestmark = [
    pytest.mark.skipif(find_compiler() is None, reason="no C++ compiler"),
    pytest.mark.skipif(sys.platform == "win32", reason="needs a POSIX pty"),
]


def run_until(bridge: ArduinoBridge, pred: Callable[[], bool], timeout: float = 5.0,
              each: Callable[[], object] | None = None) -> bool:  # fmt: skip
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        bridge.poll()
        if each is not None:
            each()
        if pred():
            return True
        time.sleep(0.005)
    return pred()


@pytest.fixture
def board() -> Iterator[VirtualArduino]:
    b = VirtualArduino("S0=85,S1=X,S2=NORISE,S3=212")
    yield b
    b.close()


@pytest.fixture
def bridge(board: VirtualArduino) -> Iterator[ArduinoBridge]:
    br = ArduinoBridge(board.transport, MonotonicClock(), port_label=board.port)
    yield br
    br.close()


def test_handshake_pattern_sync_and_readings(bridge: ArduinoBridge) -> None:
    assert run_until(bridge, lambda: bridge.state is LinkState.CONNECTED)
    assert bridge.firmware_version == "0.3.0"
    got: dict[str, object] = {}

    def collect() -> None:
        pass

    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        for r in bridge.poll():
            got[r.sensor_id] = (r.status, r.distance_cm)
        time.sleep(0.005)
    assert got == {
        "S0": (RangeStatus.VALID, 85.0),
        "S1": (RangeStatus.NO_ECHO, None),
        "S2": (RangeStatus.INVALID, None),
        "S3": (RangeStatus.VALID, 212.0),
    }
    st = bridge.status()
    assert 4.0 <= st.us_rate_hz <= 6.0  # firmware: 4 sensors x 50 ms = 5 Hz
    assert st.decode_errors == 0 and st.dropped_us_frames == 0
    assert st.firmware_mode is FirmwareMode.BOOT  # no command yet


def test_commands_ack_expiry_fallback_and_recovery(bridge: ArduinoBridge) -> None:
    assert run_until(bridge, lambda: bridge.state is LinkState.CONNECTED)
    pats = (2, 0, 0, 0, 0, 0, 0, 0)
    assert run_until(bridge, lambda: bridge.firmware_mode is FirmwareMode.HOST,
                     each=lambda: bridge.send_patterns(pats, 400))  # fmt: skip
    assert run_until(bridge, lambda: bridge.status().last_hap_seq_acked is not None,
                     each=lambda: bridge.send_patterns(pats, 400))  # fmt: skip
    bridge.commands_enabled = False
    t0 = time.monotonic()
    assert run_until(bridge, lambda: bridge.firmware_mode is FirmwareMode.FALLBACK, 3.0)
    assert 0.3 <= time.monotonic() - t0 <= 1.0  # TTL 400 ms after the last command
    bridge.commands_enabled = True
    assert run_until(bridge, lambda: bridge.firmware_mode is FirmwareMode.HOST,
                     each=lambda: bridge.send_patterns(pats, 400))  # fmt: skip
    assert bridge.status().firmware_rx_errors == 0


def test_unexpected_reset_is_detected_and_resynced(
    board: VirtualArduino, bridge: ArduinoBridge
) -> None:
    assert run_until(bridge, lambda: bridge.state is LinkState.CONNECTED)
    board.reset()
    assert run_until(bridge, lambda: bridge.resets_seen >= 1)
    assert bridge.state is LinkState.CONNECTED
    assert bridge.status().dropped_us_frames == 0  # seq restart is not "lost frames"


def test_flush_happens_before_reset_so_hello_survives(board: VirtualArduino) -> None:
    """Regression: flushing *after* the reset raced with the board's fresh $HLO."""
    for _ in range(5):
        br = ArduinoBridge(board.transport, MonotonicClock(), hello_timeout_s=1.0,
                           reset_attempts=0)  # fmt: skip
        try:
            assert run_until(br, lambda b=br: b.state is LinkState.CONNECTED, 1.5)
        finally:
            br.close()


def test_link_lost_when_board_dies_then_reconnects() -> None:
    boards = [VirtualArduino("S0=100")]

    def factory() -> PySerialTransport:
        if not boards[-1].alive:
            boards.append(VirtualArduino("S0=100"))
        return boards[-1].transport()

    br = ArduinoBridge(factory, MonotonicClock(), reconnect_interval_s=0.2)
    try:
        assert run_until(br, lambda: br.state is LinkState.CONNECTED)
        boards[-1].close()  # the "Arduino" disappears
        assert run_until(br, lambda: br.state is not LinkState.CONNECTED, 3.0)
        assert run_until(br, lambda: br.state is LinkState.CONNECTED, 5.0)
        assert br.reconnects >= 1
    finally:
        br.close()
        for b in boards:
            b.close()


def test_missing_port_is_reported_not_fatal() -> None:
    t = PySerialTransport("/dev/does-not-exist-hapticsight")
    with pytest.raises(TransportError):
        t.open()
    br = ArduinoBridge(lambda: PySerialTransport("/dev/does-not-exist-hapticsight"),
                       MonotonicClock())  # fmt: skip
    assert br.poll() == []
    assert br.state is LinkState.DISCONNECTED and "cannot open" in br.detail


# ------------------------------------------------------------ runtime + labels


def test_runtime_drives_firmware_from_shared_pipeline(
    board: VirtualArduino, tmp_path: Path
) -> None:
    board.set_ranges("S0=60,S1=X,S2=X,S3=X")
    logger = TelemetryLogger(tmp_path)
    br = ArduinoBridge(board.transport, MonotonicClock(), on_event=logger.hook)
    source = ArduinoInput(br, virtual=board)
    assert source.kind is SourceKind.SIMULATED_VIRTUAL and source.kind.simulated
    runtime = HardwareRuntime(source, logger=logger, loop_hz=20)
    try:
        runtime.run(duration_s=2.5)
        last = runtime.last
        assert last is not None
        assert last.result.frame.pattern_string()[0] == "2"  # M0 WARNING from S0 at 60 cm
        assert br.firmware_mode is FirmwareMode.HOST
        assert "SIMULATED" in source.title()
        assert source.display("S0").text == "60 cm"
    finally:
        runtime.close()
    log = read_log(logger.path)
    kinds = {r["kind"] for r in log}
    assert {"step", "readings", "tx_line", "rx_line", "link_state"} <= kinds
    readings = [r for r in log if r["kind"] == "readings"]
    assert all(r["source"] == "simulated-virtual-arduino" and r["simulated"] for r in readings)
    steps = [r for r in log if r["kind"] == "step"]
    assert any(r["haptics"].startswith("2") and r["hap_sent"] for r in steps)


def test_real_label_only_for_real_transport(board: VirtualArduino) -> None:
    br = ArduinoBridge(board.transport, MonotonicClock())
    run_until(br, lambda: br.state is LinkState.CONNECTED)
    try:
        with pytest.raises(ValueError):  # virtual transport may not be declared "real"
            ArduinoInput(br)
    finally:
        br.close()
    assert ManualInput().kind is SourceKind.SIMULATED_MANUAL
    assert SourceKind.REAL_ARDUINO.simulated is False
    real_title = ArduinoInput(
        ArduinoBridge(lambda: PySerialTransport("/dev/nope"), MonotonicClock())
    ).title()
    assert real_title.startswith("ULTRASONIC: DISCONNECTED")


def test_virtual_hand_controls_change_firmware_readings(board: VirtualArduino) -> None:
    br = ArduinoBridge(board.transport, MonotonicClock())
    source = ArduinoInput(br, virtual=board)
    try:
        assert source.manual is not None
        source.manual.set("S1", "75")
        source.manual_changed()
        end = time.monotonic() + 2.0
        while time.monotonic() < end and source.display("S1").text != "75 cm":
            source.readings(time.monotonic())
            time.sleep(0.01)
        assert source.display("S1").text == "75 cm"
    finally:
        source.close()


# ------------------------------------------------------------ CLI


def test_cli_link_loss_passes_on_virtual_board(capsys: pytest.CaptureFixture[str]) -> None:
    rc = hw_main(["link-loss", "--port", "virtual", "--hold", "1", "--stop", "1.5",
                  "--resume", "1", "--virtual-ranges", "S0=X"])  # fmt: skip
    out = capsys.readouterr().out
    assert rc == 0, out
    assert "PASS (HOST -> FALLBACK -> HOST)" in out
    assert "SIMULATED virtual Arduino" in out


def test_cli_motor_test_requires_explicit_enable(capsys: pytest.CaptureFixture[str]) -> None:
    assert hw_main(["motor-test", "--port", "virtual"]) == 2
    assert "refusing" in capsys.readouterr().out
    assert hw_main(["motor-test", "--port", "virtual", "--motor", "3", "--pattern", "warning",
                    "--seconds", "0.3", "--yes"]) == 0  # fmt: skip
    out = capsys.readouterr().out
    assert "ON  M3" in out and "OFF all motors" in out
    with pytest.raises(SystemExit):
        hw_main(["motor-test", "--port", "virtual", "--pattern", "OFF", "--yes"])


def test_cli_monitor_ultrasonic_and_headless_run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert hw_main(["monitor", "--port", "virtual", "--seconds", "1", "--keepalive",
                    "--log", str(tmp_path)]) == 0  # fmt: skip
    out = capsys.readouterr().out
    assert "<- $HLO,1,0.3.0" in out and "state=connected" in out
    assert hw_main(["ultrasonic", "--port", "virtual", "--seconds", "1.5",
                    "--virtual-ranges", "S0=85,S1=X"]) == 0  # fmt: skip
    out = capsys.readouterr().out
    assert "S0 front   85" in out and "SIMULATED" in out
    assert hw_main(["run", "--port", "virtual", "--front", "off", "--rear", "off", "--headless",
                    "--seconds", "1.5", "--log", str(tmp_path)]) == 0  # fmt: skip
    logs = sorted(tmp_path.glob("runtime-*.jsonl"))
    session = read_log(logs[-1])[0]
    assert session["kind"] == "session" and session["ultrasonic_simulated"] is True
    assert hw_main(["ports"]) == 0


def test_live_ui_with_virtual_arduino(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pygame = pytest.importorskip("pygame")
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.chdir(tmp_path)
    rc = hw_main(["run", "--port", "virtual", "--front", "off", "--rear", "off",
                  "--frames", "60", "--virtual-ranges", "S0=60"])  # fmt: skip
    assert rc == 0
    assert pygame.get_init() is False  # app shut pygame down cleanly
    assert not [p for p in os.listdir(tmp_path) if p.endswith(".png")]  # nothing written
