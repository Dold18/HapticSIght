from __future__ import annotations

import os
from typing import Any

import pytest

from hapticsight.core.models import Observation, RangeStatus, SourceType

# Any test that touches pygame must never open a real window.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")


def make_obs(
    bearing: float = 0.0,
    distance: float | None = 150.0,
    *,
    id: str = "o1",
    t: float = 0.0,
    confidence: float = 1.0,
    closing: float | None = None,
    source: SourceType = SourceType.SIMULATION,
    bearing_unc: float = 0.0,
    distance_unc: float | None = None,
    **kwargs: Any,
) -> Observation:
    return Observation(
        id=id,
        source=source,
        bearing_deg=bearing,
        timestamp=t,
        confidence=confidence,
        range_status=RangeStatus.VALID if distance is not None else RangeStatus.UNKNOWN,
        distance_cm=distance,
        distance_uncertainty_cm=distance_unc,
        bearing_uncertainty_deg=bearing_unc,
        closing_speed_cm_s=closing,
        **kwargs,
    )


@pytest.fixture
def obs_factory():  # type: ignore[no-untyped-def]
    return make_obs
