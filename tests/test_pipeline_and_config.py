from __future__ import annotations

from pathlib import Path

import pytest

from hapticsight.config import HapticSightConfig, config_from_dict, load_config
from hapticsight.core.models import RangeStatus, SourceType, UltrasonicReading
from hapticsight.haptics.patterns import PatternId
from hapticsight.pipeline import HapticSightPipeline
from hapticsight.risk.engine import Severity
from tests.conftest import make_obs


def test_config_overrides_nested_fields_and_patterns() -> None:
    cfg = config_from_dict(
        {
            "risk": {"thresholds": {"caution_cm": 180}, "min_confidence": 0.5},
            "haptics": {"release_hold_s": 0.5, "patterns": {"WARNING": {"on_ms": 120}}},
        }
    )
    assert cfg.risk.thresholds.caution_cm == 180
    assert cfg.risk.thresholds.warning_cm == 100  # untouched default
    assert cfg.risk.min_confidence == 0.5
    assert cfg.haptics.release_hold_s == 0.5
    assert cfg.haptics.patterns[PatternId.WARNING].on_ms == 120
    assert cfg.haptics.patterns[PatternId.WARNING].pause_ms == 350


def test_config_rejects_unknown_keys_and_invalid_values() -> None:
    with pytest.raises(KeyError):
        config_from_dict({"risk": {"thresholdz": {}}})
    with pytest.raises(ValueError):
        config_from_dict({"risk": {"thresholds": {"warning_cm": 500}}})


def test_example_config_file_loads() -> None:
    path = Path(__file__).resolve().parents[1] / "config" / "example.toml"
    cfg = load_config(path)
    assert isinstance(cfg, HapticSightConfig)
    assert cfg.risk.thresholds.caution_cm == 200


def us(sensor: str, t: float, d: float | None) -> UltrasonicReading:
    if d is None:
        return UltrasonicReading(sensor, t, RangeStatus.NO_ECHO)
    return UltrasonicReading(sensor, t, RangeStatus.VALID, d)


def all_sensors(t: float, s0: float | None = None) -> list[UltrasonicReading]:
    return [us("S0", t, s0), us("S1", t, None), us("S2", t, None), us("S3", t, None)]


def test_pipeline_end_to_end_ultrasonic_only() -> None:
    p = HapticSightPipeline()
    result = p.process(0.0, all_sensors(0.0, s0=80))
    assert result.report.max_severity is Severity.WARNING
    assert result.frame.pattern_string() == "20000000"


def test_pipeline_approach_detected_from_ultrasonic_history() -> None:
    p = HapticSightPipeline()
    result = None
    for i in range(8):
        t = i * 0.1
        result = p.process(t, all_sensors(t, s0=190 - 4 * i))  # 40 cm/s toward wearer
    assert result is not None
    top = result.report.assessments[0]
    assert top.is_approaching
    assert result.frame.pattern_ids[0] is PatternId.CAUTION_APPROACH


def test_pipeline_camera_only_does_not_warn_but_is_reported() -> None:
    p = HapticSightPipeline()
    cam = make_obs(10, None, id="CAM_FRONT:1", source=SourceType.CAMERA, confidence=0.9)
    result = p.process(0.0, all_sensors(0.0), [cam])
    assert [a.id for a in result.report.assessments] == ["CAM_FRONT:1"]
    assert result.report.max_severity is Severity.NONE
    assert result.frame.pattern_string() == "00000000"


def test_pipeline_fusion_moves_warning_to_camera_bearing() -> None:
    p = HapticSightPipeline()
    cam = make_obs(24, None, id="CAM_FRONT:1", source=SourceType.CAMERA, confidence=0.9)
    # Echo on S0 at 70 cm; camera says the object is at 24° -> M1 gets the fused warning.
    # 24° is outside the 15° beam, so the raw S0 echo is kept too (M0).
    result = p.process(0.0, all_sensors(0.0, s0=70), [cam])
    assert result.frame.pattern_ids[1] is PatternId.WARNING
    assert result.frame.pattern_ids[0] is PatternId.WARNING


def test_pipeline_sensor_dropout_signals_degraded_instead_of_clear() -> None:
    p = HapticSightPipeline()
    p.process(0.0, all_sensors(0.0, s0=80))
    # S0 stops reporting; others continue.
    result = None
    for i in range(1, 11):
        t = i * 0.1
        result = p.process(t, [us("S1", t, None), us("S2", t, None), us("S3", t, None)])
    assert result is not None
    assert result.frame.pattern_ids[0] is PatternId.DEGRADED
    assert [h.sensor_id for h in result.report.degraded_sensors] == ["S0"]


def test_pipeline_reset_clears_state() -> None:
    p = HapticSightPipeline()
    p.process(0.0, all_sensors(0.0, s0=30))
    p.reset()
    result = p.process(0.0, all_sensors(0.0))
    assert result.frame.pattern_string() == "00000000"
