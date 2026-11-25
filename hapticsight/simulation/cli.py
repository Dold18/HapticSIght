"""Command-line entry point: ``hapticsight-sim`` / ``python -m hapticsight.simulation``.

Modes:
  (default)        interactive Pygame simulator
  --headless       run a scenario without graphics and print a timeline
  --list           list scenarios
  --screenshot P   render one frame offscreen to PNG P (after --at seconds)
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence

from hapticsight.config import HapticSightConfig, load_config
from hapticsight.core.format import format_bearing
from hapticsight.haptics.patterns import PatternId
from hapticsight.simulation.engine import PerceptionMode, SimulationEngine
from hapticsight.simulation.scenarios import ALL_SCENARIOS, get_scenario

_PATTERN_GLYPH = {
    PatternId.OFF: ".",
    PatternId.DEGRADED: "d",
    PatternId.CAUTION: "c",
    PatternId.CAUTION_APPROACH: "C",
    PatternId.WARNING: "w",
    PatternId.WARNING_APPROACH: "W",
    PatternId.CRITICAL: "x",
    PatternId.CRITICAL_APPROACH: "X",
}


def motor_glyphs(engine: SimulationEngine) -> str:
    return "".join(_PATTERN_GLYPH[p] for p in engine.result.frame.pattern_ids)


def timeline(engine: SimulationEngine, duration: float, interval: float) -> list[str]:
    lines = [
        f"Scenario {engine.scenario.number}: {engine.scenario.name}  "
        f"[mode={engine.mode.value}, seed={engine.seed}]",
        "motors M0..M7: . off  c caution  w warning  x critical  (UPPER = approaching)  d degraded",
        f"{'t(s)':>5}  {'M01234567':9}  top hazards",
    ]
    t_next = 0.0
    total_steps = round(duration * engine.sim_config.tick_hz)
    for i in range(total_steps + 1):
        if i:
            engine.step()
        if engine.time + 1e-9 >= t_next:
            t_next += interval
            hazards = []
            for a in engine.result.report.top(3):
                rng = (
                    f"{a.effective_distance_cm:.0f}cm"
                    if a.effective_distance_cm is not None
                    else "?"
                )
                tag = a.severity.name if a.is_actionable else ("stale" if a.is_stale else "-")
                hazards.append(
                    f"{a.id}@{format_bearing(a.bearing_deg)} {rng} {a.motion.value[:4]} {tag}"
                )
            degraded = [h.sensor_id for h in engine.result.report.degraded_sensors]
            extra = f"  degraded={','.join(degraded)}" if degraded else ""
            lines.append(
                f"{engine.time:5.1f}  {motor_glyphs(engine):9}  {'; '.join(hazards)}{extra}"
            )
    for t, note in engine.event_log:
        lines.append(f"event @ {t:.2f}s: {note}")
    return lines


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hapticsight-sim", description=__doc__.splitlines()[0])
    p.add_argument("--scenario", "-s", default="1", help="scenario number or key (0 = sandbox)")
    p.add_argument("--seed", type=int, default=None, help="override the scenario's RNG seed")
    p.add_argument("--config", help="TOML configuration override file")
    p.add_argument("--ground-truth", action="store_true", help="start in ground-truth mode")
    p.add_argument("--list", action="store_true", help="list scenarios and exit")
    p.add_argument("--headless", action="store_true", help="print a timeline, no window")
    p.add_argument("--duration", type=float, default=None, help="headless duration (s)")
    p.add_argument("--interval", type=float, default=0.5, help="headless print interval (s)")
    p.add_argument("--screenshot", metavar="PNG", help="save one rendered frame and exit")
    p.add_argument("--at", type=float, default=2.0, help="sim time for --screenshot (s)")
    p.add_argument("--fps", type=int, default=60, help="GUI frame-rate cap")
    p.add_argument("--frames", type=int, default=None, help="exit after N frames, report FPS")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        for s in ALL_SCENARIOS:
            print(f"{s.number:>2}  {s.key:<15} {s.name}")
        return 0
    try:
        scenario = get_scenario(args.scenario)
    except KeyError as exc:
        print(exc, file=sys.stderr)
        return 2
    config = load_config(args.config) if args.config else HapticSightConfig()
    engine = SimulationEngine(config, scenario=scenario, seed=args.seed)
    if args.ground_truth:
        engine.set_mode(PerceptionMode.GROUND_TRUTH)

    if args.headless:
        duration = args.duration if args.duration is not None else scenario.duration_s
        print("\n".join(timeline(engine, duration, args.interval)))
        return 0

    from hapticsight.simulation.app import SimulatorApp  # imports pygame lazily

    app = SimulatorApp(engine, fps=args.fps)
    if args.screenshot:
        app.save_screenshot(args.screenshot, at_time=args.at)
        print(f"saved {args.screenshot}")
        return 0
    app.run(max_frames=args.frames)
    if args.frames is not None:
        print(f"rendered {app.frames_rendered} frames, average {app.average_fps:.1f} fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
