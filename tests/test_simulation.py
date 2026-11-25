from __future__ import annotations

import random

import pytest

from hapticsight.core.geometry import polar_to_cartesian
from hapticsight.core.layout import CAMERAS
from hapticsight.core.models import RangeStatus, SourceType
from hapticsight.haptics.patterns import PatternId
from hapticsight.perception.ultrasonic import UltrasonicConfig
from hapticsight.simulation.engine import PerceptionMode, SimulationEngine
from hapticsight.simulation.scenarios import ScenarioEvent, get_scenario
from hapticsight.simulation.sensors import (
    SensorMode,
    SimSensorConfig,
    SimulatedCamera,
    SimulatedUltrasonicArray,
)
from hapticsight.simulation.world import (
    CONTACT_CLEARANCE_CM,
    WEARER_RADIUS_CM,
    SimObstacle,
    World,
    ground_truth_observations,
)


def obstacle_at(bearing: float, clearance: float, id: str = "A", **kw: object) -> SimObstacle:
    x, y = polar_to_cartesian(bearing, clearance + 20 + WEARER_RADIUS_CM)
    return SimObstacle(id=id, x=x, y=y, **kw)  # type: ignore[arg-type]


# ------------------------------------------------------------ world


def test_world_geometry_and_closing_speed() -> None:
    o = obstacle_at(90, 100, vx=-50.0)
    assert o.bearing_deg == pytest.approx(90)
    assert o.clearance_cm == pytest.approx(100)
    assert o.closing_speed_cm_s == pytest.approx(50)
    tangential = obstacle_at(0, 100, vx=50.0)
    assert tangential.closing_speed_cm_s == pytest.approx(0)


def test_world_stops_obstacles_at_contact_and_after_time() -> None:
    w = World()
    w.add(obstacle_at(0, 30, id="A", vy=-100.0))
    w.add(obstacle_at(90, 300, id="B", vx=-100.0, stop_after_s=0.5))
    for _ in range(100):
        w.step(0.01)
    a, b = w.get("A"), w.get("B")
    assert a is not None and b is not None
    assert a.clearance_cm == pytest.approx(CONTACT_CLEARANCE_CM)
    assert a.speed == 0
    assert b.clearance_cm == pytest.approx(250, abs=1.5)
    with pytest.raises(ValueError):
        w.add(obstacle_at(0, 10, id="A"))


def test_ground_truth_observations_are_exact() -> None:
    w = World()
    w.add(obstacle_at(45, 120, vx=-10.0))
    (obs,) = ground_truth_observations(w, 1.0)
    assert obs.source is SourceType.SIMULATION
    assert obs.bearing_deg == pytest.approx(45)
    assert obs.distance_cm == pytest.approx(120)


# ------------------------------------------------------------ sensors


def make_array(noise: float = 0.0) -> SimulatedUltrasonicArray:
    return SimulatedUltrasonicArray(
        UltrasonicConfig(beam_half_angle_deg=15),
        SimSensorConfig(ultrasonic_noise_cm=noise),
        random.Random(0),
    )


def readings_by_sensor(arr: SimulatedUltrasonicArray, w: World) -> dict[str, object]:
    return {r.sensor_id: r for r in arr.sample(w, 0.0, force=True)}


def test_ultrasonic_has_blind_zones_between_sensors() -> None:
    w = World()
    w.add(obstacle_at(45, 150))  # diagonal, small, far: outside every beam
    r = readings_by_sensor(make_array(), w)
    assert all(x.status is RangeStatus.NO_ECHO for x in r.values())  # type: ignore[attr-defined]


def test_ultrasonic_reports_nearest_echo_in_beam() -> None:
    w = World()
    w.add(obstacle_at(5, 150, id="far"))
    w.add(obstacle_at(-5, 70, id="near"))
    r = readings_by_sensor(make_array(), w)["S0"]
    assert r.status is RangeStatus.VALID  # type: ignore[attr-defined]
    assert r.distance_cm == pytest.approx(70)  # type: ignore[attr-defined]


def test_wide_close_object_overlaps_beam() -> None:
    w = World()
    w.add(obstacle_at(30, 40, radius_cm=40.0))  # centre outside ±15°, body overlaps beam
    assert readings_by_sensor(make_array(), w)["S0"].status is RangeStatus.VALID  # type: ignore[attr-defined]


def test_ultrasonic_modes_and_soft_obstacles() -> None:
    w = World()
    w.add(obstacle_at(0, 80, echo_probability=0.0))
    arr = make_array()
    arr.modes["S1"] = SensorMode.DROPOUT
    arr.modes["S2"] = SensorMode.FAULT
    r = readings_by_sensor(arr, w)
    assert r["S0"].status is RangeStatus.NO_ECHO  # type: ignore[attr-defined]
    assert "S1" not in r  # dropout: nothing at all
    assert r["S2"].status is RangeStatus.INVALID  # type: ignore[attr-defined]


def test_ultrasonic_sampling_rate() -> None:
    arr = make_array()
    w = World()
    emitted = sum(bool(arr.sample(w, i * 0.02)) for i in range(50))  # 1 s at 50 Hz
    assert emitted == 5  # default matches the firmware: 4 sensors x 50 ms slots = 5 Hz


def test_camera_reports_bearing_only_within_fov() -> None:
    cfg = SimSensorConfig(camera_miss_probability=0.0, camera_bearing_noise_deg=0.0)
    cam = SimulatedCamera(CAMERAS[0], cfg, random.Random(0))
    w = World()
    w.add(obstacle_at(10, 120, id="in"))
    w.add(obstacle_at(90, 120, id="out"))
    w.add(obstacle_at(0, 100, id="hidden", camera_visible=False))
    (det,) = cam.sample(w, 0.0, force=True)
    assert det.id == "CAM_FRONT:in"
    assert det.range_status is RangeStatus.UNKNOWN
    assert det.distance_cm is None and det.closing_speed_cm_s is None
    cam.enabled = False
    assert cam.sample(w, 1.0, force=True) == []


# ------------------------------------------------------------ engine


def run_patterns(engine: SimulationEngine, seconds: float) -> list[tuple[float, str]]:
    return [(r.timestamp, r.frame.pattern_string()) for r in engine.run_for(seconds)]


@pytest.mark.parametrize("number", range(0, 11))
def test_every_scenario_is_deterministic(number: int) -> None:
    a = SimulationEngine(scenario=get_scenario(number))
    b = SimulationEngine(scenario=get_scenario(number))
    assert run_patterns(a, 4.0) == run_patterns(b, 4.0)
    # reset() reproduces the run exactly
    a.reset()
    c = SimulationEngine(scenario=get_scenario(number))
    assert run_patterns(a, 2.0) == run_patterns(c, 2.0)


def test_seed_changes_noise_but_not_structure() -> None:
    a = SimulationEngine(scenario=get_scenario(1), seed=1)
    b = SimulationEngine(scenario=get_scenario(1), seed=2)
    ra, rb = a.run_for(2.0), b.run_for(2.0)
    assert [x.frame.pattern_string() for x in ra] == [x.frame.pattern_string() for x in rb]
    da = [x.report.assessments[0].effective_distance_cm for x in ra]
    db = [x.report.assessments[0].effective_distance_cm for x in rb]
    assert da != db


def test_fixed_timestep_independent_of_frame_rate() -> None:
    a = SimulationEngine(scenario=get_scenario(2))
    b = SimulationEngine(scenario=get_scenario(2))
    for _ in range(180):  # 60 fps for 3 s
        a.advance(1 / 60)
    while b.time < a.time - 1e-9:
        b.advance(0.1)  # 10 fps
    assert b.time == pytest.approx(a.time)
    assert a.result.frame.pattern_string() == b.result.frame.pattern_string()
    assert a.motor_outputs() == b.motor_outputs()


def test_advance_caps_catch_up_and_respects_pause() -> None:
    e = SimulationEngine()
    assert e.advance(5.0) == e.sim_config.max_steps_per_advance
    e.paused = True
    assert e.advance(1.0) == 0


def test_interactive_edits_while_paused_update_assessment() -> None:
    e = SimulationEngine()
    e.paused = True
    o = e.add_obstacle(0.0, 120.0)  # clearance 80 -> WARNING
    assert e.result.frame.pattern_ids[0] is PatternId.WARNING
    e.move_obstacle(o.id, 0.0, 70.0)  # clearance 30 -> CRITICAL
    assert e.result.frame.pattern_ids[0] is PatternId.CRITICAL
    e.remove_obstacle(o.id)
    e.paused = False
    e.run_for(1.0)
    assert e.result.frame.pattern_string() == "00000000"


def test_events_and_toggles() -> None:
    e = SimulationEngine(scenario=get_scenario(1))
    e.apply_event(ScenarioEvent(0, "camera_enabled", "CAM_FRONT", False, "front cam off"))
    assert not e.camera("CAM_FRONT").enabled
    assert e.toggle_camera("CAM_FRONT")
    assert e.cycle_ultrasonic_mode("S0") is SensorMode.DROPOUT
    assert e.cycle_ultrasonic_mode("S0") is SensorMode.FAULT
    assert e.cycle_ultrasonic_mode("S0") is SensorMode.ENABLED
    assert e.toggle_mode() is PerceptionMode.GROUND_TRUTH
    assert e.result.observations[0].source is SourceType.SIMULATION
    assert len(e.event_log) == 5
