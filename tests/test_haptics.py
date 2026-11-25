from __future__ import annotations

import pytest

from hapticsight.core.models import SensorHealth, SensorStatus
from hapticsight.haptics.encoder import HapticConfig, HapticEncoder, hazard_rank
from hapticsight.haptics.mapping import motor_for_bearing, motors_for_bearing
from hapticsight.haptics.patterns import DEFAULT_PATTERNS, PatternId, PulsePattern
from hapticsight.haptics.scheduler import PulseScheduler
from hapticsight.risk.engine import RiskEngine, RiskReport, Severity
from tests.conftest import make_obs

# ------------------------------------------------------------ mapping


@pytest.mark.parametrize(
    ("bearing", "motor"),
    [
        (0, 0),
        (22.4999, 0),
        (22.5, 1),  # boundary belongs to the clockwise sector
        (45, 1),
        (67.5, 2),
        (90, 2),
        (135, 3),
        (180, 4),
        (225, 5),
        (270, 6),
        (315, 7),
        (337.4999, 7),
        (337.5, 0),
        (359.9, 0),
        (360, 0),
        (-1, 0),
        (-22.5, 0),
        (-22.6, 7),
        (725, 0),
    ],
)
def test_motor_sector_boundaries(bearing: float, motor: int) -> None:
    assert motor_for_bearing(bearing) == motor


def test_every_motor_bearing_maps_to_itself() -> None:
    for m in range(8):
        assert motor_for_bearing(m * 45) == m


def test_uncertainty_spreads_only_into_reached_sectors() -> None:
    assert motors_for_bearing(0, 0) == (0,)
    assert motors_for_bearing(0, 15) == (0,)  # ±15° stays inside ±22.5°
    assert motors_for_bearing(0, 30) == (0, 1, 7)
    assert motors_for_bearing(350, 20) == (0, 7)  # wraps across 0°
    assert sorted(motors_for_bearing(0, 180)) == list(range(8))
    with pytest.raises(ValueError):
        motors_for_bearing(0, -1)


# ------------------------------------------------------------ patterns


def test_pattern_validation() -> None:
    with pytest.raises(ValueError):
        PulsePattern(on_ms=-1)
    with pytest.raises(ValueError):
        PulsePattern(on_ms=70000)
    with pytest.raises(ValueError):
        PulsePattern(on_ms=100, count=10)
    with pytest.raises(ValueError):
        PulsePattern(on_ms=0, count=1)
    PulsePattern(on_ms=0, count=0)


def test_pattern_timing() -> None:
    p = PulsePattern(on_ms=100, off_ms=50, count=2, pause_ms=250)
    assert p.cycle_ms == 500
    assert p.duty_cycle == pytest.approx(0.4)
    states = [p.is_on(t) for t in (0, 99, 100, 149, 150, 249, 250, 499, 500)]
    assert states == [True, True, False, False, True, True, False, False, True]


def test_continuous_and_off_patterns() -> None:
    cont = PulsePattern(on_ms=200)
    assert all(cont.is_on(t) for t in range(0, 1000, 37))
    assert cont.describe() == "continuous"
    off = DEFAULT_PATTERNS[PatternId.OFF]
    assert not any(off.is_on(t) for t in range(0, 1000, 37))
    assert off.describe() == "off"


def test_default_patterns_escalate_in_duty_and_rate() -> None:
    c, w, k = (
        DEFAULT_PATTERNS[p] for p in (PatternId.CAUTION, PatternId.WARNING, PatternId.CRITICAL)
    )
    assert c.duty_cycle < w.duty_cycle < k.duty_cycle
    assert c.cycle_ms > w.cycle_ms > k.cycle_ms
    for plain, approach in [
        (PatternId.CAUTION, PatternId.CAUTION_APPROACH),
        (PatternId.WARNING, PatternId.WARNING_APPROACH),
        (PatternId.CRITICAL, PatternId.CRITICAL_APPROACH),
    ]:
        assert DEFAULT_PATTERNS[approach].count == 2
        assert DEFAULT_PATTERNS[approach].cycle_ms == pytest.approx(
            DEFAULT_PATTERNS[plain].cycle_ms, rel=0.25
        )


# ------------------------------------------------------------ encoder


def report_for(*obs, now: float = 0.0, health=()) -> RiskReport:  # type: ignore[no-untyped-def]
    return RiskEngine().assess(list(obs), now, health)


def test_rank_ordering() -> None:
    assert hazard_rank(Severity.NONE, True) == 0
    assert hazard_rank(Severity.CAUTION, False) < hazard_rank(Severity.CAUTION, True)
    assert hazard_rank(Severity.CAUTION, True) < hazard_rank(Severity.WARNING, False)
    assert hazard_rank(Severity.CRITICAL, True) == 7


def test_single_obstacle_maps_to_correct_motor() -> None:
    frame = HapticEncoder().encode(report_for(make_obs(90, 70)), 0)
    assert frame.pattern_string() == "00200000"
    assert frame.channels[2].sources == ("o1",)


def test_simultaneous_warnings_from_multiple_directions() -> None:
    frame = HapticEncoder().encode(
        report_for(make_obs(0, 150, id="a"), make_obs(270, 40, id="b"), make_obs(135, 80, id="c")),
        0,
    )
    assert frame.pattern_ids[0] is PatternId.CAUTION
    assert frame.pattern_ids[3] is PatternId.WARNING
    assert frame.pattern_ids[6] is PatternId.CRITICAL
    assert sum(c.active for c in frame.channels) == 3


def test_conflict_resolution_highest_rank_wins() -> None:
    frame = HapticEncoder().encode(
        report_for(make_obs(10, 150, id="far"), make_obs(-8, 70, id="near")), 0
    )
    ch = frame.channels[0]
    assert ch.pattern_id is PatternId.WARNING
    assert ch.sources == ("near",)


def test_conflict_tie_lists_all_sources_and_approach_beats_plain() -> None:
    frame = HapticEncoder().encode(report_for(make_obs(5, 70, id="a"), make_obs(-5, 80, id="b")), 0)
    assert set(frame.channels[0].sources) == {"a", "b"}
    frame2 = HapticEncoder().encode(
        report_for(make_obs(5, 70, id="a"), make_obs(-5, 80, id="b", closing=20)), 0
    )
    assert frame2.channels[0].pattern_id is PatternId.WARNING_APPROACH
    assert frame2.channels[0].approaching


def test_approaching_cue_selects_approach_pattern() -> None:
    frame = HapticEncoder().encode(report_for(make_obs(180, 150, closing=20)), 0)
    assert frame.pattern_ids[4] is PatternId.CAUTION_APPROACH


def test_bearing_uncertainty_spreads_across_motors() -> None:
    frame = HapticEncoder().encode(report_for(make_obs(0, 70, bearing_unc=30)), 0)
    assert [int(p) for p in frame.pattern_ids] == [2, 2, 0, 0, 0, 0, 0, 2]
    no_spread = HapticEncoder(HapticConfig(spread_with_bearing_uncertainty=False))
    assert no_spread.encode(report_for(make_obs(0, 70, bearing_unc=30)), 0).pattern_string() == (
        "20000000"
    )


def test_release_hysteresis_suppresses_flicker_but_escalates_immediately() -> None:
    enc = HapticEncoder(HapticConfig(release_hold_s=0.35))
    assert enc.encode(report_for(make_obs(0, 70)), 0.0).pattern_ids[0] is PatternId.WARNING
    # Detection drops out for one update: channel holds WARNING.
    held = enc.encode(report_for(now=0.1), 0.1).channels[0]
    assert held.pattern_id is PatternId.WARNING
    assert held.held
    # Escalation is never delayed.
    assert enc.encode(report_for(make_obs(0, 30, t=0.2), now=0.2), 0.2).pattern_ids[0] is (
        PatternId.CRITICAL
    )
    # After the hold expires with nothing requested, the channel turns off.
    assert enc.encode(report_for(now=0.56), 0.56).pattern_ids[0] is PatternId.OFF


def test_hysteresis_steps_down_through_lower_requests() -> None:
    enc = HapticEncoder(HapticConfig(release_hold_s=0.3))
    enc.encode(report_for(make_obs(0, 30)), 0.0)  # CRITICAL
    assert enc.encode(report_for(make_obs(0, 150, t=0.2), now=0.2), 0.2).pattern_ids[0] is (
        PatternId.CRITICAL
    )
    assert enc.encode(report_for(make_obs(0, 150, t=0.4), now=0.4), 0.4).pattern_ids[0] is (
        PatternId.CAUTION
    )


def test_degraded_sensor_signalled_but_outranked_by_hazards() -> None:
    health = [
        SensorHealth("S0", 0, SensorStatus.STALE, 0.0, 1.0),
        SensorHealth("S1", 90, SensorStatus.FAULT, 1.0, 0.0),
        SensorHealth("S2", 180, SensorStatus.OK, 1.0, 0.0),
        SensorHealth("S3", 270, SensorStatus.NO_ECHO, 1.0, 0.0),
    ]
    frame = HapticEncoder().encode(report_for(make_obs(90, 70, t=1.0), now=1.0, health=health), 1)
    assert frame.pattern_ids[0] is PatternId.DEGRADED
    assert frame.channels[0].degraded
    assert frame.channels[0].sources == ("S0:stale",)
    assert frame.pattern_ids[2] is PatternId.WARNING  # hazard beats degraded
    assert frame.pattern_ids[4] is PatternId.OFF
    assert frame.pattern_ids[6] is PatternId.OFF  # NO_ECHO is not degraded
    quiet = HapticEncoder(HapticConfig(signal_degraded_coverage=False))
    assert quiet.encode(report_for(now=1.0, health=health), 1).pattern_ids[0] is PatternId.OFF


def test_stale_hazard_does_not_keep_warning_after_hold() -> None:
    engine = RiskEngine()
    enc = HapticEncoder(HapticConfig(release_hold_s=0.35))
    obs = make_obs(0, 30, t=0.0)
    assert enc.encode(engine.assess([obs], 0.0), 0.0).pattern_ids[0] is PatternId.CRITICAL
    # Not refreshed, but still within stale_after_s = 0.5: still a live warning.
    assert enc.encode(engine.assess([obs], 0.5), 0.5).pattern_ids[0] is PatternId.CRITICAL
    # Now stale: the risk engine rates it NONE; the encoder only holds briefly.
    held = enc.encode(engine.assess([obs], 0.6), 0.6).channels[0]
    assert held.pattern_id is PatternId.CRITICAL and held.held
    assert enc.encode(engine.assess([obs], 0.9), 0.9).pattern_ids[0] is PatternId.OFF


def test_low_confidence_does_not_drive_motor() -> None:
    frame = HapticEncoder().encode(report_for(make_obs(0, 30, confidence=0.1)), 0)
    assert frame.pattern_string() == "00000000"


def test_config_requires_complete_pattern_table() -> None:
    with pytest.raises(ValueError):
        HapticConfig(patterns={PatternId.OFF: DEFAULT_PATTERNS[PatternId.OFF]})
    with pytest.raises(ValueError):
        HapticConfig(command_ttl_ms=0)


# ------------------------------------------------------------ scheduler


def test_scheduler_follows_pattern_timing_and_ttl() -> None:
    enc = HapticEncoder()
    sched = PulseScheduler()
    frame = enc.encode(report_for(make_obs(0, 70)), 0.0)  # WARNING: 150 on / 350 pause
    sched.apply(frame, 0.0)
    assert sched.outputs(0.0)[0] is True
    assert sched.outputs(0.149)[0] is True
    assert sched.outputs(0.2)[0] is False
    assert sched.outputs(0.45)[0] is False
    # 0.5 s is the TTL: without a refresh, every motor goes off
    assert sched.outputs(0.5) == (False,) * 8
    assert sched.active_pattern(0, 0.5) is PatternId.OFF


def test_scheduler_same_pattern_does_not_restart_phase() -> None:
    enc = HapticEncoder()
    sched = PulseScheduler()
    sched.apply(enc.encode(report_for(make_obs(0, 70)), 0.0), 0.0)
    sched.apply(enc.encode(report_for(make_obs(0, 70, t=0.2), now=0.2), 0.2), 0.2)
    # If the phase had restarted at 0.2 the motor would be ON at 0.25.
    assert sched.outputs(0.25)[0] is False


def test_scheduler_new_pattern_restarts_phase_and_output_byte() -> None:
    enc = HapticEncoder()
    sched = PulseScheduler()
    sched.apply(enc.encode(report_for(make_obs(0, 150)), 0.0), 0.0)
    sched.apply(enc.encode(report_for(make_obs(0, 30, t=0.3), now=0.3), 0.3), 0.3)
    assert sched.active_pattern(0, 0.3) is PatternId.CRITICAL
    assert sched.outputs(0.3)[0] is True
    assert sched.output_byte(0.3) == 0b00000001


def test_scheduler_timing_is_independent_of_sampling_rate() -> None:
    """Sampling at 7 Hz or 1 kHz must see the same state at the same instants."""
    enc = HapticEncoder(HapticConfig(command_ttl_ms=10_000))
    sched = PulseScheduler()
    sched.apply(enc.encode(report_for(make_obs(0, 150, closing=20)), 0.0), 0.0)
    fine = {round(i / 1000, 3): sched.outputs(i / 1000)[0] for i in range(0, 3000)}
    for k in range(0, 21):
        t = round(k / 7, 3)
        if t in fine:
            assert sched.outputs(t)[0] == fine[t]
