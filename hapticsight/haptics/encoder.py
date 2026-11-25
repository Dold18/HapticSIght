"""Risk report -> eight motor channel commands.

Per update the encoder:

1. Maps each actionable assessment to the motor(s) covering its bearing
   (spreading across neighbouring sectors only when bearing uncertainty
   genuinely reaches them).
2. Requests a *rank* on each of those motors (higher = more urgent)::

       0 OFF  1 ADVISORY  2 DEGRADED  3 CAUTION  4 CAUTION+APPROACH
       5 WARNING  6 WARNING+APPROACH  7 CRITICAL  8 CRITICAL+APPROACH

   When several obstacles hit the same motor, the highest rank wins.
3. Requests DEGRADED on the motor facing any ultrasonic sensor whose coverage
   is lost (stale / fault / never seen), unless a hazard outranks it. Missing
   data is thereby signalled instead of being silently rendered as "clear".
4. Applies **release hysteresis**: escalation is immediate, but a channel only
   drops to a lower rank after that higher rank has not been requested for
   ``release_hold_s``. This stops flicker from noisy or intermittent detections
   without ever delaying a more urgent warning.

Stale or low-confidence observations are already rated Severity.NONE by the
risk engine, so they cannot drive a hazard pattern; once the hold expires the
channel falls back to DEGRADED (if coverage is lost) or OFF.

**Experimental advisory cue (disabled by default).** When
``advisory_cue_enabled`` is set, at most ``advisory_max_channels`` camera-only
objects (no measured range) that are confident, tracked for a while, and in
the forward path may request ADVISORY: a single 40 ms tick every 3 s, the
lowest rank. It never implies distance or danger, never overrides a hazard or
a degraded-coverage cue, and uses its own pattern slot (8), documented in
docs/PROTOCOL.md as optional.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

from hapticsight.core.geometry import angular_distance
from hapticsight.core.layout import MOTOR_COUNT
from hapticsight.core.models import RangeStatus, SourceType
from hapticsight.haptics.mapping import motor_for_bearing, motors_for_bearing
from hapticsight.haptics.patterns import DEFAULT_PATTERNS, PatternId, PulsePattern
from hapticsight.risk.engine import RiskAssessment, RiskReport, Severity

RANK_COUNT = 9
RANK_ADVISORY = 1
RANK_DEGRADED = 2
RANK_FIRST_HAZARD = 3

_RANK_TO_PATTERN: tuple[PatternId, ...] = (
    PatternId.OFF,
    PatternId.ADVISORY,
    PatternId.DEGRADED,
    PatternId.CAUTION,
    PatternId.CAUTION_APPROACH,
    PatternId.WARNING,
    PatternId.WARNING_APPROACH,
    PatternId.CRITICAL,
    PatternId.CRITICAL_APPROACH,
)


def hazard_rank(severity: Severity, approaching: bool) -> int:
    if severity is Severity.NONE:
        return 0
    return int(severity) * 2 + int(approaching) + 1


def rank_severity(rank: int) -> Severity:
    return Severity((rank - 1) // 2) if rank >= RANK_FIRST_HAZARD else Severity.NONE


def rank_approaching(rank: int) -> bool:
    return rank >= RANK_FIRST_HAZARD and rank % 2 == 0


def rank_to_pattern(rank: int) -> PatternId:
    return _RANK_TO_PATTERN[rank]


@dataclass(frozen=True, slots=True)
class HapticConfig:
    patterns: Mapping[PatternId, PulsePattern] = field(default_factory=lambda: DEFAULT_PATTERNS)
    release_hold_s: float = 0.35
    command_ttl_ms: int = 500
    signal_degraded_coverage: bool = True
    spread_with_bearing_uncertainty: bool = True
    # Experimental camera-only advisory cue (see module docstring). Off by default.
    advisory_cue_enabled: bool = False
    advisory_min_confidence: float = 0.6
    advisory_min_track_age_s: float = 1.0
    advisory_max_channels: int = 1
    advisory_forward_only: bool = True
    advisory_forward_half_angle_deg: float = 45.0

    def __post_init__(self) -> None:
        missing = [p.name for p in PatternId if p not in self.patterns]
        if missing:
            raise ValueError(f"pattern table missing entries: {missing}")
        if self.release_hold_s < 0:
            raise ValueError("release_hold_s must be non-negative")
        if not 0 < self.command_ttl_ms <= 65535:
            raise ValueError("command_ttl_ms must be in (0, 65535]")


@dataclass(frozen=True, slots=True)
class ChannelCommand:
    motor: int
    pattern_id: PatternId
    pattern: PulsePattern
    severity: Severity
    approaching: bool
    degraded: bool
    held: bool  # True when maintained only by release hysteresis
    advisory: bool  # experimental camera-only cue (no distance implied)
    sources: tuple[str, ...]

    @property
    def active(self) -> bool:
        return self.pattern_id is not PatternId.OFF


@dataclass(frozen=True, slots=True)
class HapticFrame:
    timestamp: float
    channels: tuple[ChannelCommand, ...]
    ttl_ms: int

    @property
    def pattern_ids(self) -> tuple[PatternId, ...]:
        return tuple(c.pattern_id for c in self.channels)

    def pattern_string(self) -> str:
        """Compact form used on the wire: one hex digit per motor, M0 first."""
        return "".join(f"{int(p):X}" for p in self.pattern_ids)


class HapticEncoder:
    def __init__(self, config: HapticConfig | None = None) -> None:
        self.config = config or HapticConfig()
        self.reset()

    def reset(self) -> None:
        # last time each rank was requested, per motor
        self._last_requested: list[list[float | None]] = [
            [None] * RANK_COUNT for _ in range(MOTOR_COUNT)
        ]
        self._last_sources: list[list[tuple[str, ...]]] = [
            [()] * RANK_COUNT for _ in range(MOTOR_COUNT)
        ]

    def _requests(self, report: RiskReport) -> tuple[list[int], list[list[str]]]:
        cfg = self.config
        ranks = [0] * MOTOR_COUNT
        sources: list[list[str]] = [[] for _ in range(MOTOR_COUNT)]
        for a in report.actionable:
            rank = hazard_rank(a.severity, a.is_approaching)
            unc = (
                a.observation.bearing_uncertainty_deg if cfg.spread_with_bearing_uncertainty else 0
            )
            for m in motors_for_bearing(a.bearing_deg, unc):
                if rank > ranks[m]:
                    ranks[m] = rank
                    sources[m] = [a.id]
                elif rank == ranks[m]:
                    sources[m].append(a.id)
        if cfg.signal_degraded_coverage:
            for h in report.degraded_sensors:
                m = motor_for_bearing(h.bearing_deg)
                if ranks[m] < RANK_DEGRADED:
                    ranks[m] = RANK_DEGRADED
                    sources[m] = [f"{h.sensor_id}:{h.status.value}"]
        if cfg.advisory_cue_enabled:
            for a in self.advisory_candidates(report)[: cfg.advisory_max_channels]:
                m = motor_for_bearing(a.bearing_deg)
                if ranks[m] < RANK_ADVISORY:
                    ranks[m] = RANK_ADVISORY
                    sources[m] = [a.id]
        return ranks, sources

    def advisory_candidates(self, report: RiskReport) -> list[RiskAssessment]:
        """Camera-only (unranged) objects eligible for the advisory cue, best first."""
        cfg = self.config
        out = []
        for a in report.assessments:
            obs = a.observation
            if obs.source is not SourceType.CAMERA or obs.range_status is not RangeStatus.UNKNOWN:
                continue
            if a.is_stale or a.is_low_confidence or obs.confidence < cfg.advisory_min_confidence:
                continue
            if float(obs.metadata.get("track_age_s", 0.0)) < cfg.advisory_min_track_age_s:
                continue
            off_axis = angular_distance(obs.bearing_deg, 0.0)
            if cfg.advisory_forward_only and off_axis > cfg.advisory_forward_half_angle_deg:
                continue
            out.append(a)
        out.sort(
            key=lambda a: (angular_distance(a.bearing_deg, 0.0), -a.observation.confidence, a.id)
        )
        return out

    def encode(self, report: RiskReport, now: float) -> HapticFrame:
        cfg = self.config
        requested, sources = self._requests(report)
        channels: list[ChannelCommand] = []
        for m in range(MOTOR_COUNT):
            last = self._last_requested[m]
            req = requested[m]
            if req > 0:
                last[req] = now
                self._last_sources[m][req] = tuple(sources[m])
            effective = req
            for r in range(RANK_COUNT - 1, req, -1):
                t = last[r]
                if t is not None and now - t <= cfg.release_hold_s:
                    effective = r
                    break
            pid = rank_to_pattern(effective)
            channels.append(
                ChannelCommand(
                    motor=m,
                    pattern_id=pid,
                    pattern=cfg.patterns[pid],
                    severity=rank_severity(effective),
                    approaching=rank_approaching(effective),
                    degraded=effective == RANK_DEGRADED,
                    held=effective != req,
                    advisory=effective == RANK_ADVISORY,
                    sources=self._last_sources[m][effective] if effective else (),
                )
            )
        return HapticFrame(now, tuple(channels), cfg.command_ttl_ms)
