"""Top-level configuration aggregate and optional TOML loading.

Each layer owns its own config dataclass; this module only groups them. A TOML
file may override any subset of fields, e.g.::

    [risk.thresholds]
    caution_cm = 180

    [haptics]
    release_hold_s = 0.5

    [haptics.patterns.WARNING]
    on_ms = 120
    pause_ms = 380

Unknown keys raise an error so typos are not silently ignored.
"""

from __future__ import annotations

import dataclasses
import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from hapticsight.haptics.encoder import HapticConfig
from hapticsight.haptics.patterns import DEFAULT_PATTERNS, PatternId, PulsePattern
from hapticsight.perception.fusion import FusionConfig
from hapticsight.perception.ultrasonic import RangeRateConfig, UltrasonicConfig
from hapticsight.risk.engine import ProximityThresholds, RiskConfig


@dataclass(frozen=True, slots=True)
class HapticSightConfig:
    ultrasonic: UltrasonicConfig = field(default_factory=UltrasonicConfig)
    range_rate: RangeRateConfig = field(default_factory=RangeRateConfig)
    fusion: FusionConfig = field(default_factory=FusionConfig)
    risk: RiskConfig = field(default_factory=RiskConfig)
    haptics: HapticConfig = field(default_factory=HapticConfig)
    camera_expire_after_s: float = 0.5

    def __post_init__(self) -> None:
        # Cross-layer invariants that keep fusion from hiding valid warnings.
        if self.fusion.min_camera_confidence < self.risk.min_confidence:
            raise ValueError(
                "fusion.min_camera_confidence must be >= risk.min_confidence, otherwise a "
                "fused observation could be ignored while its echo is suppressed"
            )
        if max(self.fusion.max_ultrasonic_age_s, self.fusion.max_camera_age_s) >= (
            self.risk.stale_after_s
        ):
            raise ValueError("fusion age limits must be < risk.stale_after_s")


def _override(instance: Any, data: dict[str, Any], path: str) -> Any:
    names = {f.name: f for f in dataclasses.fields(instance)}
    changes: dict[str, Any] = {}
    for key, value in data.items():
        if key not in names:
            raise KeyError(f"unknown config key {path}{key}")
        current = getattr(instance, key)
        if isinstance(value, dict) and key == "patterns":
            table = dict(current)
            for pname, pdata in value.items():
                pid = PatternId[pname]
                table[pid] = _override(
                    table.get(pid, DEFAULT_PATTERNS[pid]), pdata, f"{path}{key}."
                )
            changes[key] = table
        elif isinstance(value, dict):
            changes[key] = _override(current, value, f"{path}{key}.")
        else:
            changes[key] = value
    return dataclasses.replace(instance, **changes)


def config_from_dict(data: dict[str, Any]) -> HapticSightConfig:
    result: HapticSightConfig = _override(HapticSightConfig(), data, "")
    return result


def load_config(path: str | Path) -> HapticSightConfig:
    with open(path, "rb") as fh:
        return config_from_dict(tomllib.load(fh))


__all__ = [
    "DEFAULT_PATTERNS",
    "FusionConfig",
    "HapticConfig",
    "HapticSightConfig",
    "ProximityThresholds",
    "PulsePattern",
    "RangeRateConfig",
    "RiskConfig",
    "UltrasonicConfig",
    "config_from_dict",
    "load_config",
]
