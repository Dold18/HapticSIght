"""Deterministic, explainable obstacle risk assessment.

Each observation is rated independently with simple rules, then sorted by
relevance. Every assessment carries a list of human-readable ``reasons`` so the
simulator (and later logs) can show *why* a rating was produced.

Rules, in order:

1. **Age.** Older than ``stale_after_s`` -> stale, never produces a warning.
   Older than ``expire_after_s`` -> dropped from the report.
2. **Confidence.** Below ``min_confidence`` -> reported but never warns.
3. **Proximity band** (only when a range was measured). Uses the conservative
   distance ``distance - uncertainty``. Bands (defaults):
   ``d < 50`` CRITICAL, ``50 <= d < 100`` WARNING, ``100 <= d <= 200`` CAUTION,
   ``d > 200`` none. Range-less observations get **no** band.
4. **Motion** (only when closing speed is known). ``>= +approach`` APPROACHING,
   ``<= -approach`` RECEDING, otherwise STEADY. Visibility or apparent size is
   never treated as approach.
5. **Time to contact** for approaching, ranged obstacles may escalate severity
   (``ttc <= 1 s`` CRITICAL, ``<= 2 s`` WARNING, ``<= 3 s`` CAUTION). It never
   lowers severity.
6. **Priority**: severity, then approaching, then shorter time-to-contact, then
   shorter distance, then within the forward walking path, then confidence.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from enum import IntEnum, StrEnum

from hapticsight.core.geometry import bearing_within
from hapticsight.core.models import Observation, SensorHealth, SourceType


class Severity(IntEnum):
    NONE = 0
    CAUTION = 1
    WARNING = 2
    CRITICAL = 3


class Motion(StrEnum):
    APPROACHING = "approaching"
    RECEDING = "receding"
    STEADY = "steady"
    UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True)
class ProximityThresholds:
    caution_cm: float = 200.0
    warning_cm: float = 100.0
    critical_cm: float = 50.0

    def __post_init__(self) -> None:
        if not 0 < self.critical_cm < self.warning_cm < self.caution_cm:
            raise ValueError("thresholds must satisfy 0 < critical < warning < caution")

    def classify(self, distance_cm: float) -> Severity:
        if distance_cm < self.critical_cm:
            return Severity.CRITICAL
        if distance_cm < self.warning_cm:
            return Severity.WARNING
        if distance_cm <= self.caution_cm:
            return Severity.CAUTION
        return Severity.NONE


@dataclass(frozen=True, slots=True)
class RiskConfig:
    thresholds: ProximityThresholds = field(default_factory=ProximityThresholds)
    approach_speed_cm_s: float = 15.0
    ttc_critical_s: float = 1.0
    ttc_warning_s: float = 2.0
    ttc_caution_s: float = 3.0
    min_confidence: float = 0.35
    stale_after_s: float = 0.6
    expire_after_s: float = 2.0
    forward_path_half_angle_deg: float = 30.0
    conservative_distance: bool = True

    def __post_init__(self) -> None:
        if not 0 < self.ttc_critical_s <= self.ttc_warning_s <= self.ttc_caution_s:
            raise ValueError("TTC thresholds must satisfy 0 < critical <= warning <= caution")
        if not 0 < self.stale_after_s <= self.expire_after_s:
            raise ValueError("require 0 < stale_after_s <= expire_after_s")
        if self.approach_speed_cm_s <= 0:
            raise ValueError("approach_speed_cm_s must be positive")


@dataclass(frozen=True, slots=True)
class RiskAssessment:
    observation: Observation
    severity: Severity
    proximity_severity: Severity | None  # None when range is unknown
    motion: Motion
    effective_distance_cm: float | None
    time_to_contact_s: float | None
    age_s: float
    is_stale: bool
    is_low_confidence: bool
    in_forward_path: bool
    reasons: tuple[str, ...]

    @property
    def id(self) -> str:
        return self.observation.id

    @property
    def bearing_deg(self) -> float:
        return self.observation.bearing_deg

    @property
    def source(self) -> SourceType:
        return self.observation.source

    @property
    def is_actionable(self) -> bool:
        return self.severity > Severity.NONE

    @property
    def is_approaching(self) -> bool:
        return self.motion is Motion.APPROACHING

    def priority_key(self) -> tuple[float, ...]:
        """Sort key: smaller is more relevant."""
        inf = math.inf
        return (
            -int(self.severity),
            -int(self.is_approaching and self.is_actionable),
            self.time_to_contact_s if self.time_to_contact_s is not None else inf,
            self.effective_distance_cm if self.effective_distance_cm is not None else inf,
            -int(self.in_forward_path),
            -self.observation.confidence,
        )


@dataclass(frozen=True, slots=True)
class RiskReport:
    timestamp: float
    assessments: tuple[RiskAssessment, ...]  # most relevant first
    sensor_health: tuple[SensorHealth, ...] = ()
    expired_count: int = 0

    @property
    def actionable(self) -> tuple[RiskAssessment, ...]:
        return tuple(a for a in self.assessments if a.is_actionable)

    @property
    def max_severity(self) -> Severity:
        return max((a.severity for a in self.assessments), default=Severity.NONE)

    @property
    def degraded_sensors(self) -> tuple[SensorHealth, ...]:
        return tuple(h for h in self.sensor_health if h.status.is_degraded)

    def top(self, n: int) -> tuple[RiskAssessment, ...]:
        return self.assessments[:n]


def _ttc_severity(ttc: float, cfg: RiskConfig) -> Severity:
    if ttc <= cfg.ttc_critical_s:
        return Severity.CRITICAL
    if ttc <= cfg.ttc_warning_s:
        return Severity.WARNING
    if ttc <= cfg.ttc_caution_s:
        return Severity.CAUTION
    return Severity.NONE


class RiskEngine:
    def __init__(self, config: RiskConfig | None = None) -> None:
        self.config = config or RiskConfig()

    def classify_motion(self, closing_speed_cm_s: float | None) -> Motion:
        if closing_speed_cm_s is None:
            return Motion.UNKNOWN
        if closing_speed_cm_s >= self.config.approach_speed_cm_s:
            return Motion.APPROACHING
        if closing_speed_cm_s <= -self.config.approach_speed_cm_s:
            return Motion.RECEDING
        return Motion.STEADY

    def assess_one(self, obs: Observation, now: float) -> RiskAssessment:
        cfg = self.config
        reasons: list[str] = []
        age = obs.age(now)

        effective: float | None = None
        proximity: Severity | None = None
        if obs.has_range and obs.distance_cm is not None:
            effective = obs.distance_cm
            if cfg.conservative_distance and obs.distance_uncertainty_cm:
                effective = max(0.0, obs.distance_cm - obs.distance_uncertainty_cm)
            proximity = cfg.thresholds.classify(effective)
            reasons.append(f"range {effective:.0f} cm -> {proximity.name}")
        else:
            reasons.append(f"range {obs.range_status.value}: no proximity band")

        motion = self.classify_motion(obs.closing_speed_cm_s)
        if obs.closing_speed_cm_s is not None:
            reasons.append(f"closing {obs.closing_speed_cm_s:+.0f} cm/s -> {motion.value}")

        ttc: float | None = None
        severity = proximity or Severity.NONE
        if motion is Motion.APPROACHING and effective is not None and obs.closing_speed_cm_s:
            ttc = effective / obs.closing_speed_cm_s
            ttc_sev = _ttc_severity(ttc, cfg)
            if ttc_sev > severity:
                reasons.append(f"time-to-contact {ttc:.1f} s escalates to {ttc_sev.name}")
                severity = ttc_sev

        stale = age > cfg.stale_after_s
        low_conf = obs.confidence < cfg.min_confidence
        if stale:
            reasons.append(f"stale ({age:.2f} s old): not used for warnings")
            severity = Severity.NONE
        if low_conf:
            reasons.append(
                f"confidence {obs.confidence:.2f} < {cfg.min_confidence:.2f}: not used for warnings"
            )
            severity = Severity.NONE

        return RiskAssessment(
            observation=obs,
            severity=severity,
            proximity_severity=proximity,
            motion=motion,
            effective_distance_cm=effective,
            time_to_contact_s=ttc,
            age_s=age,
            is_stale=stale,
            is_low_confidence=low_conf,
            in_forward_path=bearing_within(obs.bearing_deg, 0.0, cfg.forward_path_half_angle_deg),
            reasons=tuple(reasons),
        )

    def assess(
        self,
        observations: Iterable[Observation],
        now: float,
        sensor_health: Sequence[SensorHealth] = (),
    ) -> RiskReport:
        assessed: list[RiskAssessment] = []
        expired = 0
        for obs in observations:
            if obs.age(now) > self.config.expire_after_s:
                expired += 1
                continue
            assessed.append(self.assess_one(obs, now))
        assessed.sort(key=lambda a: (*a.priority_key(), a.id))
        return RiskReport(now, tuple(assessed), tuple(sensor_health), expired)
