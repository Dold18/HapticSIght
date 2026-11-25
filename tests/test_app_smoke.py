"""Headless smoke tests for the Pygame front end (SDL dummy video driver)."""

from __future__ import annotations

from pathlib import Path

import pytest

pygame = pytest.importorskip("pygame")

from hapticsight.simulation.app import ENV_W, WINDOW_H, SimulatorApp  # noqa: E402
from hapticsight.simulation.cli import main  # noqa: E402
from hapticsight.simulation.engine import PerceptionMode, SimulationEngine  # noqa: E402
from hapticsight.simulation.scenarios import get_scenario  # noqa: E402


def key(k: int, mod: int = 0) -> pygame.event.Event:
    return pygame.event.Event(pygame.KEYDOWN, key=k, mod=mod)


def test_app_handles_every_control_and_renders() -> None:
    engine = SimulationEngine(scenario=get_scenario(4))
    app = SimulatorApp(engine, fps=0)
    try:
        center = (ENV_W // 2, WINDOW_H // 2)
        click = (center[0] + 120, center[1] - 160)
        n_before = len(engine.world.obstacles)
        app.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=click))
        assert len(engine.world.obstacles) == n_before + 1
        new_id = app.selected
        app.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=(click[0], click[1] + 40)))
        app.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=click))
        moved = engine.world.get(new_id)  # type: ignore[arg-type]
        assert moved is not None
        app.handle_event(key(pygame.K_UP))
        app.handle_event(key(pygame.K_RIGHT, pygame.KMOD_SHIFT))
        assert moved.vx == 10
        app.handle_event(key(pygame.K_a))
        assert moved.closing_speed_cm_s > 0
        app.handle_event(key(pygame.K_v))
        assert moved.speed == 0
        app.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=3, pos=click))
        app.handle_event(pygame.event.Event(pygame.MOUSEWHEEL, x=0, y=1))
        for k in (pygame.K_SPACE, pygame.K_PERIOD, pygame.K_SPACE, pygame.K_h, pygame.K_g,
                  pygame.K_RIGHTBRACKET, pygame.K_LEFTBRACKET, pygame.K_TAB, pygame.K_n,
                  pygame.K_DELETE):  # fmt: skip
            app.handle_event(key(k))
            app.draw()
        assert engine.mode is PerceptionMode.GROUND_TRUTH
        app.handle_event(key(pygame.K_1, pygame.KMOD_SHIFT))
        assert engine.ultrasonic.modes["S0"].value == "dropout"
        app.handle_event(key(pygame.K_5, pygame.KMOD_SHIFT))
        assert not engine.camera("CAM_FRONT").enabled
        for k in (pygame.K_0, pygame.K_7, pygame.K_e, pygame.K_r):
            app.handle_event(key(k))
            app.draw()
        assert engine.scenario.key == "sandbox"
        app.handle_event(key(pygame.K_8))
        app.run(max_frames=20)
    finally:
        pygame.quit()


def test_quit_event_stops_loop() -> None:
    app = SimulatorApp(SimulationEngine(), fps=0)
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    app.run(max_frames=100)
    assert not app.running


def test_cli_screenshot_headless_and_list(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    out = tmp_path / "shot.png"
    assert main(["-s", "2", "--screenshot", str(out), "--at", "1.0"]) == 0
    assert out.stat().st_size > 10_000
    assert main(["--headless", "-s", "1", "--duration", "1"]) == 0
    assert "Obstacle directly ahead" in capsys.readouterr().out
    assert main(["--list"]) == 0
    assert main(["-s", "99", "--headless"]) == 2
