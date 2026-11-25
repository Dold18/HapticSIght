"""``hapticsight-hw``: Arduino link diagnostics and the unified live hardware runtime.

Every command accepts ``--port virtual`` to run against the virtual Uno (real
firmware logic + SIMULATED sensors on a pseudo-terminal) instead of hardware.

Safety: no command drives motors unless explicitly asked. ``motor-test``
needs ``--yes``, drives one motor at a time, and is limited to 10 s.
"""

from __future__ import annotations

import argparse
import select
import sys
import threading
import time
from collections import deque
from collections.abc import Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

from hapticsight.core.clock import MonotonicClock
from hapticsight.core.layout import MOTORS, ULTRASONIC_SENSORS
from hapticsight.core.models import RangeStatus
from hapticsight.haptics.patterns import PatternId
from hapticsight.hardware.arduino import ArduinoBridge, LinkState
from hapticsight.hardware.inputs import ArduinoInput
from hapticsight.hardware.serial_transport import DEFAULT_BAUD, PySerialTransport, list_serial_ports
from hapticsight.hardware.telemetry import TelemetryLogger
from hapticsight.hardware.virtual_arduino import VirtualArduino
from hapticsight.protocol.codec import ProtocolError, decode
from hapticsight.protocol.messages import FirmwareMode

MAX_MOTOR_TEST_S = 10.0
OFF = (0,) * 8


def _print(msg: str) -> None:
    print(msg, flush=True)


class Link:
    """A bridge plus (optionally) the virtual board behind it."""

    def __init__(self, args: argparse.Namespace, logger: TelemetryLogger | None = None) -> None:
        self.virtual: VirtualArduino | None = None
        self.clock = MonotonicClock()
        self.t0 = self.clock.now()
        self.logger = logger
        self.verbose = bool(getattr(args, "raw", False))
        if args.port == "virtual":
            self.virtual = VirtualArduino(
                getattr(args, "virtual_ranges", None) or "S0=150,S1=X,S2=X,S3=X"
            )
            factory: Any = self.virtual.transport
            label = f"virtual:{self.virtual.port}"
        else:
            port, baud = args.port, args.baud

            def factory() -> PySerialTransport:
                return PySerialTransport(port, baud)

            label = port
        self.bridge = ArduinoBridge(
            factory, self.clock, on_event=self._event, port_label=label,
            hello_timeout_s=args.hello_timeout,
        )  # fmt: skip
        self.label = label

    @property
    def simulated(self) -> bool:
        return self.virtual is not None

    def _event(self, kind: str, t: float, data: dict[str, Any]) -> None:
        if self.logger is not None:
            self.logger.log(kind, t, **data)
        rel = t - self.t0
        if kind == "link_state":
            _print(f"[{rel:7.3f}] link: {data['state']} ({data['detail']})")
        elif kind in ("reset_detected", "reset_pulse"):
            _print(f"[{rel:7.3f}] {kind.replace('_', ' ')} {data.get('reason', '')}")
        elif self.verbose and kind in ("rx_line", "tx_line"):
            arrow = "<-" if kind == "rx_line" else "->"
            line = data["line"]
            summary = ""
            if kind == "rx_line":
                try:
                    summary = f"   {decode(line)!r}"
                except ProtocolError as exc:
                    summary = f"   !! {type(exc).__name__}: {exc}"
            _print(f"[{rel:7.3f}] {arrow} {line}{summary}")

    def wait_connected(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.bridge.poll()
            if self.bridge.state is LinkState.CONNECTED:
                return True
            if self.bridge.state is LinkState.INCOMPATIBLE:
                return False
            time.sleep(0.01)
        return False

    def close(self) -> None:
        self.bridge.close()
        if self.virtual is not None:
            self.virtual.close()


def _banner(link: Link) -> None:
    kind = "SIMULATED virtual Arduino (no hardware)" if link.simulated else "REAL serial port"
    _print(f"port: {link.label}  [{kind}]")


def _status_line(link: Link) -> str:
    st = link.bridge.status()
    mode = st.firmware_mode.value if st.firmware_mode else "?"
    return (
        f"state={st.state.value} fw={st.firmware_version or '?'} mode={mode} "
        f"us={st.us_rate_hz:.1f}Hz rx={st.rx_messages} decode_err={st.decode_errors} "
        f"fw_rx_err={st.firmware_rx_errors if st.firmware_rx_errors is not None else '?'} "
        f"dropped_us={st.dropped_us_frames} hap_sent={st.hap_sent} "
        f"acked={st.last_hap_seq_acked if st.last_hap_seq_acked is not None else '-'}"
    )


def _connect_or_fail(link: Link, timeout: float) -> bool:
    _banner(link)
    if link.wait_connected(timeout):
        return True
    _print(f"no handshake: {_status_line(link)}")
    _print("check: correct port? firmware flashed? 115200 baud? cable data-capable?")
    return False


# ---------------------------------------------------------------- commands


def cmd_ports(args: argparse.Namespace) -> int:
    ports = list_serial_ports()
    if not ports:
        _print("no serial ports found")
    for p in ports:
        ids = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None and p.pid is not None else "-"
        mark = "  <- likely Arduino" if p.likely_arduino else ""
        _print(f"{p.device:<32} {ids:<10} {p.description} {p.vendor_hint}{mark}")
    _print(
        "\nmacOS: Uno boards usually appear as /dev/cu.usbmodem* (genuine) or "
        "/dev/cu.wchusbserial* / /dev/cu.usbserial-* (CH340 clones). Use the cu.* device."
    )
    return 0


def cmd_monitor(args: argparse.Namespace) -> int:
    args.raw = True
    logger = TelemetryLogger(args.log, "serial") if args.log else None
    link = Link(args, logger)
    try:
        _banner(link)
        if not args.keepalive:
            _print("listening only: no commands sent (the board stays in BOOT after a reset)")
        end = time.monotonic() + args.seconds
        next_status = 0.0
        while time.monotonic() < end:
            link.bridge.poll()
            if args.keepalive:
                link.bridge.send_patterns(OFF, 500)
            if time.monotonic() >= next_status:
                next_status = time.monotonic() + 1.0
                _print("   " + _status_line(link))
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        _print("final: " + _status_line(link))
        link.close()
        if logger:
            logger.close()
            _print(f"log: {logger.path}")
    return 0


def cmd_ultrasonic(args: argparse.Namespace) -> int:
    link = Link(args)
    history: dict[str, deque[str]] = {
        s.sensor_id: deque(maxlen=args.history) for s in ULTRASONIC_SENSORS
    }
    counts = dict.fromkeys(history, 0)
    try:
        if not _connect_or_fail(link, args.hello_timeout + 2):
            return 1
        t0 = time.monotonic()
        end = t0 + args.seconds
        next_print = t0 + 1.0
        while time.monotonic() < end:
            for r in link.bridge.poll():
                counts[r.sensor_id] += 1
                if r.status is RangeStatus.VALID:
                    history[r.sensor_id].append(f"{r.distance_cm:.0f}")
                else:
                    history[r.sensor_id].append("X" if r.status is RangeStatus.NO_ECHO else "E")
            if time.monotonic() >= next_print:
                next_print += 1.0
                el = time.monotonic() - t0
                kind = "SIMULATED" if link.simulated else "REAL"
                _print(
                    f"--- t={el:5.1f}s  frames {link.bridge.us_rate_hz:.2f} Hz  "
                    f"(X = no echo, E = sensor error) {kind}"
                )
                for spec in ULTRASONIC_SENSORS:
                    h = history[spec.sensor_id]
                    rate = counts[spec.sensor_id] / el
                    _print(f"  {spec.sensor_id} {spec.label:<5} {h[-1] if h else '-':>4}  "
                           f"{rate:4.1f} Hz   recent: {' '.join(h)}")  # fmt: skip
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
    return 0


def _pattern(name: str) -> PatternId:
    try:
        pid = PatternId[name.upper()]
    except KeyError as exc:
        names = ", ".join(p.name for p in PatternId if p is not PatternId.OFF)
        raise SystemExit(f"unknown pattern {name!r}; choose from {names}") from exc
    if pid is PatternId.OFF:
        raise SystemExit("choose a non-OFF pattern")
    return pid


def _drive(link: Link, patterns: tuple[int, ...], seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.bridge.poll()
        link.bridge.send_patterns(patterns, 300)
        time.sleep(0.01)


def _activate(link: Link, motor: int, pid: PatternId, seconds: float) -> None:
    seconds = min(seconds, MAX_MOTOR_TEST_S)
    pats = tuple(int(pid) if m == motor else 0 for m in range(8))
    _print(
        f"ON  {MOTORS[motor].name} ({MOTORS[motor].label}) pattern {pid.name} for {seconds:.1f}s"
    )
    try:
        _drive(link, pats, seconds)
    finally:
        _drive(link, OFF, 0.3)
        _print("OFF all motors")


def cmd_motor_test(args: argparse.Namespace) -> int:
    if not args.interactive and not args.yes:
        _print(
            "refusing to drive a motor without --yes (explicit enablement). "
            "Check the motor supply and wiring first; see docs/HARDWARE_BRINGUP.md."
        )
        return 2
    link = Link(args)
    try:
        if not _connect_or_fail(link, args.hello_timeout + 2):
            return 1
        _drive(link, OFF, 0.5)  # establish HOST mode with every motor OFF
        _print(_status_line(link))
        if not args.interactive:
            _activate(link, args.motor, _pattern(args.pattern), args.seconds)
            return 0
        _print("commands: on <motor 0-7> <PATTERN> [seconds<=10] | off | status | quit")
        while True:
            # keep HOST mode (all OFF) alive while waiting for input
            ready: list[Any] = []
            while not ready:
                link.bridge.poll()
                link.bridge.send_patterns(OFF, 300)
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            line = sys.stdin.readline()
            if not line:
                break
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "quit":
                break
            if parts[0] == "status":
                _print(_status_line(link))
            elif parts[0] == "off":
                _drive(link, OFF, 0.2)
            elif parts[0] == "on" and len(parts) >= 3:
                motor = int(parts[1])
                if not 0 <= motor < 8:
                    _print("motor must be 0-7")
                    continue
                secs = float(parts[3]) if len(parts) > 3 else 2.0
                _activate(link, motor, _pattern(parts[2]), secs)
            else:
                _print("?")
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        link.close()


def cmd_link_loss(args: argparse.Namespace) -> int:
    link = Link(args)
    pats: tuple[int, ...] = OFF
    if args.motor is not None:
        pats = tuple(int(_pattern(args.pattern)) if m == args.motor else 0 for m in range(8))
    try:
        if not _connect_or_fail(link, args.hello_timeout + 2):
            return 1
        _print("NOTE: on expiry the firmware plays its FALLBACK entry cue on all motors and then "
               "drives M0/M2/M4/M6 from local sensors.")  # fmt: skip
        modes: list[tuple[float, str]] = []
        last_mode = None
        t0 = time.monotonic()
        stop_at = t0 + args.hold
        resume_at = stop_at + args.stop
        end = resume_at + args.resume
        last_hap_t = None
        resumed_t = None
        announced_stop = announced_resume = False
        while time.monotonic() < end:
            now = time.monotonic()
            link.bridge.poll()
            sending = now < stop_at or now >= resume_at
            link.bridge.commands_enabled = sending
            if sending and link.bridge.send_patterns(pats, args.ttl):
                last_hap_t = now
                if now >= resume_at and resumed_t is None:
                    resumed_t = now
            mode = link.bridge.firmware_mode
            if mode is not None and mode != last_mode:
                modes.append((now - t0, mode.value))
                note = ""
                if mode is FirmwareMode.FALLBACK and last_hap_t is not None:
                    gap_ms = (now - last_hap_t) * 1000
                    note = f"  ({gap_ms:.0f} ms after the last command; TTL {args.ttl} ms)"
                if mode is FirmwareMode.HOST and resumed_t is not None:
                    note = f"  ({(now - resumed_t) * 1000:.0f} ms after commands resumed)"
                _print(f"[{now - t0:6.2f}s] firmware mode -> {mode.value}{note}")
                last_mode = mode
            if not announced_stop and now >= stop_at:
                announced_stop = True
                _print(f"[{now - t0:6.2f}s] host STOPS sending commands")
            if not announced_resume and now >= resume_at:
                announced_resume = True
                _print(f"[{now - t0:6.2f}s] host RESUMES commands")
            time.sleep(0.005)
        _print("transitions: " + ", ".join(f"{m}@{t:.2f}s" for t, m in modes))
        ok = [m for _, m in modes][-3:] == ["H", "F", "H"]
        _print(
            "result: " + ("PASS (HOST -> FALLBACK -> HOST)" if ok else "check transitions above")
        )
        return 0 if ok else 1
    except KeyboardInterrupt:
        return 0
    finally:
        link.bridge.commands_enabled = True
        link.close()


def cmd_virtual(args: argparse.Namespace) -> int:
    with VirtualArduino(args.ranges) as board:
        _print(f"virtual Arduino (SIMULATED sensors, real firmware logic) on {board.port}")
        _print(f"connect with: hapticsight-hw monitor --port {board.port}")
        try:
            end = time.monotonic() + args.seconds if args.seconds else float("inf")
            while time.monotonic() < end and board.alive:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    from hapticsight.config import HapticSightConfig
    from hapticsight.hardware.runtime import HardwareRuntime
    from hapticsight.pipeline import HapticSightPipeline

    logger = TelemetryLogger(args.log, "runtime") if args.log else None
    link = Link(args, logger)
    source = ArduinoInput(link.bridge, virtual=link.virtual)
    config = HapticSightConfig()
    if args.advisory:
        config = replace(config, haptics=replace(config.haptics, advisory_cue_enabled=True))
    vision = _make_vision(args)
    if logger is not None:
        logger.log(
            "session", 0.0,
            ultrasonic_source=source.kind.value, ultrasonic_simulated=source.kind.simulated,
            port=link.label, cameras={c.camera_id: str(c.device) for c in vision.config.cameras
                                      if c.enabled} if vision else {},
            loop_hz=args.loop_hz,
        )  # fmt: skip
    _banner(link)
    if args.headless:
        runtime = HardwareRuntime(source, HapticSightPipeline(config), vision, link.clock, logger,
                                  args.loop_hz)  # fmt: skip
        stop = threading.Event()
        if vision is not None:
            vision.start()
        printer = threading.Thread(target=_print_runtime, args=(runtime, link, stop), daemon=True)
        printer.start()
        try:
            runtime.run(args.seconds, stop)
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            runtime.close()
            if link.virtual is not None:
                link.virtual.close()
            if logger:
                _print(f"log: {logger.path} ({logger.records} records)")
        return 0
    from hapticsight.vision.live_app import LiveVisionApp
    from hapticsight.vision.processor import VisionSystem
    from hapticsight.vision.synthetic import MarkerDetector

    if vision is None:  # UI needs a vision system; run it with both cameras disabled
        from hapticsight.vision.config import VisionConfig

        cfg = VisionConfig()
        cfg = cfg.with_camera("CAM_FRONT", enabled=False).with_camera("CAM_REAR", enabled=False)
        vision = VisionSystem(cfg, MarkerDetector())
    app = LiveVisionApp(
        vision, config, source, detector_label=vision.detector.name,
        record_dir=Path(args.record_video) if args.record_video else None,
        fps=args.ui_fps, loop_hz=args.loop_hz, logger=logger,
    )  # fmt: skip
    try:
        app.run(max_frames=args.frames)
    finally:
        if link.virtual is not None:
            link.virtual.close()
    return 0


def _make_vision(args: argparse.Namespace) -> Any:
    """Vision system, or None if both cameras are off."""
    if args.front in (None, "off") and args.rear in (None, "off"):
        return None
    from hapticsight.vision.cli import _make_detector, build_vision_config
    from hapticsight.vision.processor import VisionSystem

    cfg = build_vision_config(args)
    return VisionSystem(cfg, _make_detector(cfg))


def _print_runtime(runtime: Any, link: Link, stop: threading.Event) -> None:
    while not stop.wait(1.0):
        last = runtime.last
        if last is None:
            continue
        top = last.result.report.top(1)
        hz = f"{top[0].severity.name} {top[0].id}" if top and top[0].is_actionable else "no hazard"
        _print(f"steps={runtime.steps} haptics={last.result.frame.pattern_string()} {hz} | "
               + _status_line(link))  # fmt: skip


# ---------------------------------------------------------------- parser


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hapticsight-hw", description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="command", required=True)

    def port(sp: argparse.ArgumentParser) -> None:
        sp.add_argument(
            "--port", required=True, help="serial device (e.g. /dev/cu.usbmodem1101) or 'virtual'"
        )
        sp.add_argument("--baud", type=int, default=DEFAULT_BAUD)
        sp.add_argument("--hello-timeout", type=float, default=3.0, help="seconds to wait for $HLO")
        sp.add_argument("--virtual-ranges", help='virtual board sensors, e.g. "S0=80,S1=X"')

    sp = sub.add_parser("ports", help="list serial ports")
    sp.set_defaults(func=cmd_ports)

    sp = sub.add_parser("monitor", help="raw + decoded serial traffic, link health")
    port(sp)
    sp.add_argument("--seconds", type=float, default=30.0)
    sp.add_argument("--keepalive", action="store_true", help="send all-OFF commands (HOST mode)")
    sp.add_argument("--log", help="write telemetry JSONL to this directory")
    sp.set_defaults(func=cmd_monitor)

    sp = sub.add_parser("ultrasonic", help="live distances, statuses, rates, history")
    port(sp)
    sp.add_argument("--seconds", type=float, default=30.0)
    sp.add_argument("--history", type=int, default=10)
    sp.set_defaults(func=cmd_ultrasonic)

    sp = sub.add_parser("motor-test", help="drive ONE motor with ONE pattern, briefly")
    port(sp)
    sp.add_argument("--motor", type=int, choices=range(8), default=0)
    sp.add_argument("--pattern", default="CAUTION")
    sp.add_argument("--seconds", type=float, default=2.0, help=f"max {MAX_MOTOR_TEST_S:.0f}")
    sp.add_argument("--yes", action="store_true", help="explicitly enable motor output")
    sp.add_argument("--interactive", action="store_true")
    sp.set_defaults(func=cmd_motor_test)

    sp = sub.add_parser("link-loss", help="stop host commands to demonstrate expiry + fallback")
    port(sp)
    sp.add_argument("--hold", type=float, default=3.0, help="seconds of normal commands")
    sp.add_argument("--stop", type=float, default=4.0, help="seconds without commands")
    sp.add_argument("--resume", type=float, default=2.0)
    sp.add_argument("--ttl", type=int, default=500)
    sp.add_argument("--motor", type=int, choices=range(8))
    sp.add_argument("--pattern", default="CAUTION")
    sp.set_defaults(func=cmd_link_loss)

    sp = sub.add_parser("virtual-arduino", help="run the virtual Uno on a pseudo-terminal")
    sp.add_argument("--ranges", default="S0=150,S1=X,S2=X,S3=X")
    sp.add_argument("--seconds", type=float)
    sp.set_defaults(func=cmd_virtual)

    sp = sub.add_parser("run", help="full system: cameras + Arduino -> pipeline -> motors")
    port(sp)
    sp.add_argument("--front", help="CAM_FRONT: index | video | synthetic | off")
    sp.add_argument("--rear", help="CAM_REAR: index | video | synthetic | off")
    sp.add_argument("--config", action="append", help="vision TOML (repeatable)")
    sp.add_argument("--model")
    sp.add_argument("--runtime", choices=["auto", "onnxruntime", "onnxruntime-coreml", "opencv"])
    sp.add_argument("--advisory", action="store_true")
    sp.add_argument("--loop-hz", type=float, default=15.0)
    sp.add_argument("--headless", action="store_true", help="no UI; print status each second")
    sp.add_argument("--seconds", type=float, help="stop after N seconds (headless)")
    sp.add_argument("--frames", type=int, help="stop after N UI frames")
    sp.add_argument("--ui-fps", type=int, default=30)
    sp.add_argument("--log", help="telemetry JSONL directory (no images)")
    sp.add_argument("--record-video", help="also record raw camera video to DIR (UI mode)")
    sp.set_defaults(func=cmd_run)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return int(args.func(args))
    except BrokenPipeError:  # e.g. output piped into `head`
        import os

        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
