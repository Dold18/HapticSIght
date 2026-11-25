"""Run the real firmware logic (firmware/arduino_uno/src) natively and cross-check it
against the Python host: protocol parsing, frame formatting, pattern table,
pulse timing, and BOOT/HOST/FALLBACK transitions. Needs a C++ compiler only
(no Arduino); skipped if none is available.
"""

from __future__ import annotations

import itertools
import random
import subprocess
from collections.abc import Iterator
from pathlib import Path

import pytest

from hapticsight.haptics.patterns import DEFAULT_PATTERNS, PatternId
from hapticsight.hardware.virtual_arduino import (
    FIRMWARE_DIR,
    build_virtual_firmware,
    find_compiler,
)
from hapticsight.protocol.codec import ProtocolError, checksum, decode, encode
from hapticsight.protocol.messages import (
    HapticCommandMessage,
    PatternDefinitionMessage,
    StatusMessage,
    UltrasonicMessage,
)

pytestmark = pytest.mark.skipif(find_compiler() is None, reason="no C++ compiler")


def test_native_unit_tests(tmp_path: Path) -> None:
    cxx = find_compiler()
    assert cxx
    exe = tmp_path / "test_main"
    sources = [
        *map(str, sorted((FIRMWARE_DIR / "src").glob("*.cpp"))),
        str(FIRMWARE_DIR / "test" / "test_main.cpp"),
    ]
    base = [cxx, "-std=c++11", "-Wall", "-Wextra", "-Werror", "-O1", "-o", str(exe), *sources]
    proc = subprocess.run([*base, "-fsanitize=address,undefined"], capture_output=True, text=True)
    if proc.returncode != 0:  # sanitizers unavailable on this toolchain
        proc = subprocess.run(base, capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stdout + run.stderr
    assert run.stdout.strip().endswith("0 failures")


class Sim:
    """Scripted virtual Uno (stdin/stdout command protocol, see test/fw_sim.cpp)."""

    def __init__(self, binary: Path) -> None:
        self.proc = subprocess.Popen(
            [str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1
        )
        self.boot = self._read_block()

    def _read_block(self) -> list[str]:
        assert self.proc.stdout is not None
        out = []
        for line in self.proc.stdout:
            line = line.rstrip("\n")
            if line == ".":
                return out
            out.append(line)
        raise RuntimeError("fw_sim exited")

    def cmd(self, line: str) -> list[str]:
        assert self.proc.stdin is not None
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        return self._read_block()

    def close(self) -> None:
        if self.proc.poll() is None:
            self.cmd("quit")
            self.proc.wait(timeout=5)


@pytest.fixture(scope="module")
def binary(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return build_virtual_firmware(force=True, build_dir=tmp_path_factory.mktemp("fwsim"))


@pytest.fixture
def sim(binary: Path) -> Iterator[Sim]:
    s = Sim(binary)
    yield s
    s.close()


def tx(lines: list[str]) -> list[str]:
    return [line[3:] for line in lines if line.startswith("TX ")]


# ------------------------------------------------------------ contract parity


def test_boot_frames_decode_on_host(sim: Sim) -> None:
    frames = [decode(x) for x in tx(sim.boot)]
    assert frames[0].protocol_version == 1  # type: ignore[union-attr]
    assert isinstance(frames[1], StatusMessage) and frames[1].mode.value == "B"


def test_pattern_table_matches_host_defaults(sim: Sim) -> None:
    rows = [line.split() for line in sim.cmd("patterns")]
    table = {int(r[1]): (int(r[2]), int(r[3]), int(r[4]), int(r[5]), int(r[6])) for r in rows}
    for pid, p in DEFAULT_PATTERNS.items():
        if pid is PatternId.OFF:
            assert table[0][0] == 0
            continue
        assert table[int(pid)] == (1, p.on_ms, p.off_ms, p.count, p.pause_ms), pid.name
    assert all(table[s][0] == 0 for s in range(9, 16))  # undefined until $PAT


def _expected(line: str) -> str:
    """What the firmware parser should answer, derived from the Python codec."""
    try:
        msg = decode(line)
    except ProtocolError:
        return "ERR"
    if isinstance(msg, HapticCommandMessage):
        return "OK HAP " + " ".join(map(str, (msg.seq, msg.ttl_ms, *msg.patterns)))
    if isinstance(msg, PatternDefinitionMessage):
        return f"OK PAT {msg.slot} {msg.on_ms} {msg.off_ms} {msg.count} {msg.pause_ms}"
    return "ERR"  # valid v1 but Arduino->host only: the firmware rejects it


def _cases(n: int) -> list[str]:
    rng = random.Random(20261124)
    good: list[str] = []
    for _ in range(n):
        if rng.random() < 0.6:
            pats = tuple(rng.randrange(16) for _ in range(8))
            m: object = HapticCommandMessage(rng.randrange(65536), rng.randint(1, 2000), pats)
        else:
            count = rng.randrange(10)
            on = rng.randint(1 if count else 0, 65535)
            m = PatternDefinitionMessage(rng.randint(1, 15), on, rng.randrange(65536), count,
                                         rng.randrange(65536))  # fmt: skip
        good.append(encode(m).rstrip("\n"))  # type: ignore[arg-type]
    cases = list(good)
    alphabet = "0123456789ABCDEFabcdef,*$-XHPTUSLO \t"
    for line in good:
        s = list(line)
        op = rng.randrange(6)
        i = rng.randrange(len(s))
        if op == 0:
            s[i] = rng.choice(alphabet)
        elif op == 1:
            del s[i]
        elif op == 2:
            s.insert(i, rng.choice(alphabet))
        elif op == 3:  # mutate the body and fix up the checksum: exercises field validation
            body = line[1 : line.rindex("*")]
            b = list(body)
            j = rng.randrange(len(b))
            b[j] = rng.choice("0123456789,ABCDEFX-")
            nb = "".join(b)
            s = list(f"${nb}*{checksum(nb):02X}")
        elif op == 4:
            s = list(line.lower())
        else:
            s = s + list("A" * rng.randrange(1, 40))
        cases.append("".join(s))
    for body in ["US,1,2,3,4,5,6", "ST,1,2,H,-,0", "HLO,1,fw", "HAP,1,500,00000000,1",
                 "PAT,1,1,1,1", "HAP,00042,0500,00000000", "HAP,1,2000,FFFFFFFF",
                 "PAT,15,65535,65535,9,65535", "PAT,1,0,0,0,0"]:  # fmt: skip
        cases.append(f"${body}*{checksum(body):02X}")
    return cases


def test_parser_parity_with_python_codec(sim: Sim) -> None:
    mismatches = []
    cases = _cases(600)
    for line in cases:
        got = sim.cmd(f"parse {line}")[0]
        want = _expected(line)
        if (got.startswith("ERR") and want != "ERR") or (not got.startswith("ERR") and got != want):
            mismatches.append((line, got, want))
    assert not mismatches, mismatches[:10]
    assert len(cases) > 1200


def test_firmware_frames_decode_and_report_ranges(sim: Sim) -> None:
    for cmd in ("range 0 85", "range 1 X", "range 2 NORISE", "range 3 STUCK"):
        sim.cmd(cmd)
    frames = [decode(x) for x in tx(sim.cmd("adv 1000"))]
    us = [f for f in frames if isinstance(f, UltrasonicMessage)]
    st = [f for f in frames if isinstance(f, StatusMessage)]
    # one frame per complete 4-sensor cycle: 5 Hz, frames exactly 200 ms apart
    assert [u.seq for u in us] == [0, 1, 2, 3]
    assert {b.arduino_ms - a.arduino_ms for a, b in itertools.pairwise(us)} == {200}
    last = us[-1].ranges
    assert last[0] == 85 and last[1].value == "X" and last[2].value == "E" and last[3].value == "E"  # type: ignore[union-attr]
    assert len(st) == 5 and all(s.mode.value == "B" for s in st)  # heartbeat every 200 ms


# ------------------------------------------------------------ behaviour parity


def motor_timeline(lines: list[str], start_byte: int, t0: int, t1: int) -> list[int]:
    events = [(int(p[1]), int(p[2], 16)) for p in (x.split() for x in lines) if p[0] == "MOT"]
    out, cur, k = [], start_byte, 0
    for t in range(t0, t1):
        while k < len(events) and events[k][0] <= t:
            cur = events[k][1]
            k += 1
        out.append(cur)
    return out


def test_pulse_timing_matches_python_patterns(sim: Sim) -> None:
    slots = (2, 6, 0, 1, 7, 5, 3, 4)
    pats = "".join(f"{s:X}" for s in slots)
    body = f"HAP,1,2000,{pats}"
    lines = sim.cmd(f"rx ${body}*{checksum(body):02X}")
    lines += sim.cmd("adv 1999")
    got = motor_timeline(lines, 0, 0, 1999)
    for t, byte in enumerate(got):
        expected = sum(
            1 << m for m, s in enumerate(slots) if DEFAULT_PATTERNS[PatternId(s)].is_on(t)
        )
        assert byte == expected, f"t={t} ms: firmware {byte:08b} != host {expected:08b}"


def test_mode_transitions_expiry_fallback_and_recovery(sim: Sim) -> None:
    sim.cmd("range 1 60")  # S1 -> M2 WARNING in fallback
    body = "HAP,9,300,00000000"
    lines = sim.cmd(f"rx ${body}*{checksum(body):02X}")
    lines += sim.cmd("adv 1000")
    modes = [(int(p[1]), p[2]) for p in (x.split() for x in lines) if p[0] == "MODE"]
    assert modes == [(0, "H"), (300, "F")]
    mot = [(int(p[1]), p[2]) for p in (x.split() for x in lines) if p[0] == "MOT"]
    assert (300, "FF") in mot  # entry cue on all 8 motors
    statuses = [decode(x) for x in tx(lines) if x.startswith("$ST")]
    assert any(s.mode.value == "F" and s.last_hap_seq == 9 for s in statuses)  # type: ignore[union-attr]
    state = sim.cmd("state")[0].split()
    assert state[1] == "F"
    # Entry cue = one DEGRADED burst (3 x 50 ms on, 80 ms gaps = 310 ms), then M2
    # is driven by the local fallback from S1 (WARNING: 150 ms on / 350 ms off).
    cue_end = 300 + 310
    assert all(b == "FF" or b == "00" for t, b in mot if 300 <= t < cue_end)
    assert (cue_end, "04") in mot and (cue_end + 150, "00") in mot
    # malformed input during fallback is counted and ignored
    sim.cmd("rx $HAP,10,300,00000000*00")
    sim.cmd("rxhex " + "41" * 100 + "0A")
    st = sim.cmd("state")[0].split()
    assert st[1] == "F" and int(st[2]) == 2
    body = "HAP,10,300,00000000"
    lines = sim.cmd(f"rx ${body}*{checksum(body):02X}")
    assert any(x.startswith("MODE") and x.endswith(" H") for x in lines)


def test_reset_announces_hello_again(sim: Sim) -> None:
    sim.cmd("adv 50")
    frames = [decode(x) for x in tx(sim.cmd("reset"))]
    assert frames[0].firmware_version  # type: ignore[union-attr]
