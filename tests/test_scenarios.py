"""Regression tests: each predefined scenario produces its documented outcome.

These exercise the full shared pipeline (simulated sensors -> perception ->
risk -> haptic encoding -> pulse scheduling) without any graphics.
"""

from __future__ import annotations

from hapticsight.haptics.patterns import PatternId as P
from hapticsight.pipeline import PipelineResult
from hapticsight.simulation.engine import PerceptionMode, SimulationEngine
from hapticsight.simulation.scenarios import SCENARIOS, get_scenario


def run(key: str, ground_truth: bool = False) -> list[PipelineResult]:
    scenario = get_scenario(key)
    engine = SimulationEngine(scenario=scenario)
    if ground_truth:
        engine.set_mode(PerceptionMode.GROUND_TRUTH)
    return [engine.result, *engine.run_for(scenario.duration_s)]


def motor(results: list[PipelineResult], m: int) -> list[P]:
    return [r.frame.pattern_ids[m] for r in results]


def between(results: list[PipelineResult], t0: float, t1: float) -> list[PipelineResult]:
    return [r for r in results if t0 <= r.timestamp <= t1]


def others_off(results: list[PipelineResult], keep: set[int]) -> bool:
    return all(r.frame.pattern_ids[m] is P.OFF for r in results for m in range(8) if m not in keep)


def first_index(seq: list[P], value: P) -> int:
    return seq.index(value)


def test_all_ten_required_scenarios_exist() -> None:
    assert [s.number for s in SCENARIOS] == list(range(1, 11))
    assert all(s.expected and s.description for s in SCENARIOS)


def test_1_single_obstacle_ahead() -> None:
    res = run("ahead")
    assert set(motor(res, 0)) == {P.WARNING}
    assert others_off(res, {0})


def test_2_approach_from_right_escalates_with_cue() -> None:
    res = run("approach_right")
    m2 = motor(res, 2)
    assert others_off(res, {2})
    c, w, x = (
        first_index(m2, p) for p in (P.CAUTION_APPROACH, P.WARNING_APPROACH, P.CRITICAL_APPROACH)
    )
    assert c < w < x
    # Time-to-contact brings WARNING before the 100 cm distance band is reached.
    assert res[w].report.assessments[0].effective_distance_cm > 100  # type: ignore[operator]
    # Camera-blind side: only the ultrasonic sensor sees it.
    assert all(a.source.value == "ultrasonic" for r in res for a in r.report.assessments)
    # After stopping, approach cue ends but the distance keeps it CRITICAL.
    assert m2[-1] is P.CRITICAL


def test_3_obstacle_behind() -> None:
    res = run("behind")
    assert set(motor(res, 4)) == {P.WARNING}
    assert others_off(res, {4})


def test_4_two_directions_simultaneously() -> None:
    res = run("two_directions")
    for r in between(res, 0.5, 8.0):
        assert r.frame.pattern_ids[0] is P.CAUTION
        assert r.frame.pattern_ids[6] is P.WARNING
        assert r.report.assessments[0].bearing_deg == 270
    assert others_off(res, {0, 6})


def test_5_competing_obstacles_same_motor() -> None:
    res = run("same_motor")
    assert set(motor(res, 0)) == {P.WARNING}
    assert others_off(res, {0})
    late = res[-1]
    assert len(late.frame.channels[0].sources) == 2
    assert all(a.observation.metadata.get("shared_range") for a in late.report.actionable)

    gt = run("same_motor", ground_truth=True)[-1]
    sev = {a.id: a.severity.name for a in gt.report.assessments}
    assert sev == {"gt:A": "CAUTION", "gt:B": "WARNING"}
    assert gt.frame.channels[0].sources == ("gt:B",)


def test_6_moving_away_steps_down_without_approach_cue() -> None:
    res = run("moving_away")
    m0 = motor(res, 0)
    rank = {P.OFF: 0, P.CAUTION: 1, P.WARNING: 2, P.CRITICAL: 3}
    assert set(m0) <= set(rank)  # no approach patterns ever
    ranks = [rank[p] for p in m0]
    assert ranks == sorted(ranks, reverse=True)  # monotonically decreasing
    assert ranks[0] == 3 and ranks[-1] == 0
    assert {r.report.assessments[0].motion.value for r in between(res, 1.0, 3.0)} == {"receding"}


def test_7_crossing_in_front() -> None:
    res = run("crossing")
    assert all(r.frame.pattern_string() == "00000000" for r in between(res, 0.0, 3.0))
    assert all(r.frame.pattern_string() == "00000000" for r in between(res, 6.5, 9.0))
    assert any(p is not P.OFF for p in motor(between(res, 3.5, 5.5), 0))
    assert others_off(res, {7, 0, 1})

    gt = run("crossing", ground_truth=True)
    first_active = {
        m: next(i for i, r in enumerate(gt) if r.frame.pattern_ids[m] is not P.OFF)
        for m in (7, 0, 1)
    }
    assert first_active[7] < first_active[0] < first_active[1]


def test_8_dropout_signals_degraded_and_recovers() -> None:
    res = run("dropout")
    assert set(motor(between(res, 0.1, 2.0), 0)) == {P.WARNING}
    # Stale reading stops warning (0.5 s) + hold (0.35 s); never fabricated afterwards.
    assert set(motor(between(res, 2.9, 4.95), 0)) == {P.DEGRADED}
    during = between(res, 4.1, 4.9)
    for r in during:
        cams = [a for a in r.report.assessments if a.source.value == "camera"]
        assert cams and all(a.effective_distance_cm is None for a in cams)
    assert set(motor(between(res, 5.3, 6.4), 0)) == {P.WARNING}
    assert set(motor(between(res, 6.6, 8.4), 2)) == {P.DEGRADED}
    assert set(motor(between(res, 9.0, 10.0), 2)) == {P.OFF}


def test_9_boundary_wrap_only_front_motor() -> None:
    res = run("boundary")
    assert set(motor(res, 0)) == {P.WARNING}
    assert others_off(res, {0})
    bearings = [r.report.assessments[0].bearing_deg for r in res if r.report.assessments]
    assert any(b > 340 for b in bearings) and any(b < 20 for b in bearings)


def test_10_uncertain_observations_handled_conservatively() -> None:
    res = run("uncertain")
    assert set(motor(res, 0)) == {P.OFF}  # soft object: no range -> no invented band
    assert set(motor(res, 4)) == {P.OFF}  # low-confidence ghost ignored
    late = between(res, 0.5, 8.0)
    on = sum(r.frame.pattern_ids[2] is P.WARNING for r in late)
    assert on / len(late) > 0.8  # intermittent echo smoothed by hysteresis
    ids = {a.id: a for a in res[-1].report.assessments}
    assert ids["CAM_FRONT:SOFT"].proximity_severity is None
    assert ids["CAM_REAR:GHOST"].is_low_confidence
