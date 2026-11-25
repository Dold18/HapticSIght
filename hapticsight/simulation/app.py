"""Interactive Pygame front end.

This module only *renders* engine state and translates input into engine
calls. All perception, risk, and haptic logic lives in the shared packages and
runs inside :class:`SimulationEngine`; nothing here influences motor decisions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import pygame

from hapticsight.core.format import format_bearing, format_distance
from hapticsight.core.geometry import polar_to_cartesian
from hapticsight.core.layout import CAMERAS, MOTORS, ULTRASONIC_SENSORS
from hapticsight.core.models import RangeStatus, SensorStatus, SourceType
from hapticsight.haptics.patterns import PatternId
from hapticsight.risk.engine import RiskAssessment, Severity
from hapticsight.simulation.engine import PerceptionMode, SimulationEngine
from hapticsight.simulation.scenarios import SANDBOX, SCENARIOS, get_scenario
from hapticsight.simulation.sensors import SensorMode
from hapticsight.simulation.world import WEARER_RADIUS_CM, SimObstacle

Color = tuple[int, int, int]

WINDOW_W, WINDOW_H = 1360, 860
ENV_W = 860
PANEL_X = ENV_W
PANEL_W = WINDOW_W - ENV_W

BG: Color = (16, 19, 26)
PANEL_BG: Color = (23, 27, 36)
GRID: Color = (44, 50, 64)
TEXT: Color = (222, 226, 235)
DIM: Color = (130, 138, 155)
FAINT: Color = (80, 87, 102)
ACCENT: Color = (90, 200, 250)
US_BEAM: Color = (60, 190, 170)
US_READING: Color = (110, 255, 220)
CAM_COLOR: Color = (120, 150, 255)
OBSTACLE: Color = (200, 205, 215)
SELECT: Color = (90, 200, 250)

PATTERN_COLORS: dict[PatternId, Color] = {
    PatternId.OFF: (62, 68, 82),
    PatternId.DEGRADED: (165, 120, 255),
    PatternId.CAUTION: (240, 205, 70),
    PatternId.CAUTION_APPROACH: (240, 205, 70),
    PatternId.WARNING: (255, 145, 45),
    PatternId.WARNING_APPROACH: (255, 145, 45),
    PatternId.CRITICAL: (245, 65, 65),
    PatternId.CRITICAL_APPROACH: (245, 65, 65),
}
SEVERITY_COLORS: dict[Severity, Color] = {
    Severity.NONE: DIM,
    Severity.CAUTION: PATTERN_COLORS[PatternId.CAUTION],
    Severity.WARNING: PATTERN_COLORS[PatternId.WARNING],
    Severity.CRITICAL: PATTERN_COLORS[PatternId.CRITICAL],
}
PATTERN_NAMES: dict[PatternId, str] = {
    PatternId.OFF: "off",
    PatternId.DEGRADED: "DEGRADED",
    PatternId.CAUTION: "CAUTION",
    PatternId.CAUTION_APPROACH: "CAUTION+APPR",
    PatternId.WARNING: "WARNING",
    PatternId.WARNING_APPROACH: "WARNING+APPR",
    PatternId.CRITICAL: "CRITICAL",
    PatternId.CRITICAL_APPROACH: "CRITICAL+APPR",
}

SPEEDS = (0.25, 0.5, 1.0, 2.0)
HELP_LINES = (
    ("Mouse", ""),
    ("  Left-click empty space", "add obstacle"),
    ("  Left-drag obstacle", "move it"),
    ("  Right-click obstacle", "delete it"),
    ("  Wheel", "zoom"),
    ("Selected obstacle", ""),
    ("  Tab", "select next"),
    ("  Arrows", "move 10 cm"),
    ("  Shift+Arrows", "change velocity 10 cm/s"),
    ("  A / V", "approach wearer / stop"),
    ("  Delete", "remove"),
    ("Simulation", ""),
    ("  Space / .", "pause-resume / single step"),
    ("  R", "reset scenario"),
    ("  1-9, 0", "scenario 1-10"),
    ("  E", "empty sandbox"),
    ("  N", "new obstacle ahead"),
    ("  [ / ]", "slower / faster"),
    ("  G", "ground truth <-> sensor perception"),
    ("Sensors", ""),
    ("  Shift+1..4", "cycle S0..S3: ok / dropout / fault"),
    ("  Shift+5 / Shift+6", "front / rear camera on-off"),
    ("Other", ""),
    ("  H", "toggle this help"),
    ("  Esc / Q", "quit"),
)


@dataclass
class View:
    cx: float = ENV_W / 2
    cy: float = WINDOW_H / 2
    scale: float = 0.88  # px per cm

    def to_screen(self, x: float, y: float) -> tuple[int, int]:
        return round(self.cx + x * self.scale), round(self.cy - y * self.scale)

    def to_world(self, sx: float, sy: float) -> tuple[float, float]:
        return (sx - self.cx) / self.scale, (self.cy - sy) / self.scale

    def polar(self, bearing: float, dist_from_center_cm: float) -> tuple[int, int]:
        x, y = polar_to_cartesian(bearing, dist_from_center_cm)
        return self.to_screen(x, y)


class SimulatorApp:
    def __init__(self, engine: SimulationEngine, fps: int = 60) -> None:
        pygame.init()
        pygame.display.set_caption("HapticSight Simulator")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        self.engine = engine
        self.fps = fps
        self.view = View()
        self.clock = pygame.time.Clock()
        self.font = _font(14)
        self.small = _font(12)
        self.bold = _font(15, bold=True)
        self.title = _font(20, bold=True)
        self._text_cache: dict[tuple[int, str, Color], pygame.Surface] = {}
        self.selected: str | None = None
        self.dragging = False
        self.show_help = False
        self.speed_index = SPEEDS.index(1.0)
        self.running = True
        self.overlay = pygame.Surface((ENV_W, WINDOW_H), pygame.SRCALPHA)
        self.frames_rendered = 0
        self._elapsed_s = 0.0

    @property
    def average_fps(self) -> float:
        return self.frames_rendered / self._elapsed_s if self._elapsed_s > 0 else 0.0

    # ------------------------------------------------------------ main loop

    def run(self, max_frames: int | None = None) -> None:
        frames = 0
        try:
            while self.running:
                real_dt = self.clock.tick(self.fps) / 1000.0
                if frames:
                    self._elapsed_s += real_dt
                for event in pygame.event.get():
                    self.handle_event(event)
                self.engine.advance(real_dt * SPEEDS[self.speed_index])
                self.draw()
                pygame.display.flip()
                frames += 1
                self.frames_rendered = frames
                if max_frames is not None and frames >= max_frames:
                    break
        finally:
            pygame.quit()

    def save_screenshot(self, path: str, at_time: float) -> None:
        try:
            self.engine.run_for(at_time)
            obstacles = self.engine.world.obstacles
            self.selected = obstacles[0].id if obstacles else None
            self.draw()
            pygame.image.save(self.screen, path)
        finally:
            pygame.quit()

    # ------------------------------------------------------------ input

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            self._on_key(event.key, event.mod)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.pos[0] < ENV_W:
            self._on_mouse_down(event.button, event.pos)
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging and self.selected:
            x, y = self.view.to_world(*event.pos)
            self.engine.move_obstacle(self.selected, x, y)
        elif event.type == pygame.MOUSEWHEEL:
            factor = 1.1**event.y
            self.view.scale = min(3.0, max(0.35, self.view.scale * factor))

    def _hit(self, pos: tuple[int, int]) -> SimObstacle | None:
        wx, wy = self.view.to_world(*pos)
        tolerance = 8 / self.view.scale
        for o in reversed(self.engine.world.obstacles):
            if math.hypot(o.x - wx, o.y - wy) <= o.radius_cm + tolerance:
                return o
        return None

    def _on_mouse_down(self, button: int, pos: tuple[int, int]) -> None:
        hit = self._hit(pos)
        if button == 1:
            if hit is None:
                x, y = self.view.to_world(*pos)
                if math.hypot(x, y) < WEARER_RADIUS_CM + 25:
                    return  # do not spawn inside the wearer
                hit = self.engine.add_obstacle(x, y)
            self.selected = hit.id
            self.dragging = True
        elif button == 3 and hit is not None:
            self.engine.remove_obstacle(hit.id)
            if self.selected == hit.id:
                self.selected = None

    def _on_key(self, key: int, mod: int) -> None:
        eng = self.engine
        shift = bool(mod & pygame.KMOD_SHIFT)
        digit_keys = [pygame.K_0, pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4,
                      pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9]  # fmt: skip
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif key in (pygame.K_h, pygame.K_SLASH):
            self.show_help = not self.show_help
        elif key == pygame.K_SPACE:
            eng.paused = not eng.paused
            if eng.paused:
                eng.refresh()
        elif key == pygame.K_PERIOD and eng.paused:
            eng.step()
        elif key == pygame.K_r:
            eng.reset()
            self.selected = None
        elif key == pygame.K_g:
            eng.toggle_mode()
        elif key == pygame.K_e:
            self._load(SANDBOX.number)
        elif key in digit_keys:
            n = digit_keys.index(key)
            if shift:
                self._sensor_toggle(n)
            else:
                self._load(10 if n == 0 else n)
        elif key == pygame.K_LEFTBRACKET:
            self.speed_index = max(0, self.speed_index - 1)
        elif key == pygame.K_RIGHTBRACKET:
            self.speed_index = min(len(SPEEDS) - 1, self.speed_index + 1)
        elif key == pygame.K_n:
            self.selected = eng.add_obstacle(0.0, 150.0 + WEARER_RADIUS_CM + 20).id
        elif key == pygame.K_TAB:
            ids = [o.id for o in eng.world.obstacles]
            if ids:
                i = ids.index(self.selected) + 1 if self.selected in ids else 0
                self.selected = ids[i % len(ids)]
        elif self.selected and (o := eng.world.get(self.selected)) is not None:
            self._on_selected_key(o, key, shift)

    def _on_selected_key(self, o: SimObstacle, key: int, shift: bool) -> None:
        eng = self.engine
        arrows = {
            pygame.K_LEFT: (-10.0, 0.0),
            pygame.K_RIGHT: (10.0, 0.0),
            pygame.K_UP: (0.0, 10.0),
            pygame.K_DOWN: (0.0, -10.0),
        }
        if key in arrows:
            dx, dy = arrows[key]
            if shift:
                eng.set_velocity(o.id, o.vx + dx, o.vy + dy)
            else:
                eng.move_obstacle(o.id, o.x + dx, o.y + dy)
        elif key == pygame.K_v:
            eng.set_velocity(o.id, 0.0, 0.0)
        elif key == pygame.K_a:
            d = max(o.center_distance, 1e-6)
            eng.set_velocity(o.id, -o.x / d * 50.0, -o.y / d * 50.0)
        elif key in (pygame.K_DELETE, pygame.K_BACKSPACE):
            eng.remove_obstacle(o.id)
            self.selected = None

    def _sensor_toggle(self, n: int) -> None:
        if 1 <= n <= len(ULTRASONIC_SENSORS):
            self.engine.cycle_ultrasonic_mode(ULTRASONIC_SENSORS[n - 1].sensor_id)
        elif n in (5, 6):
            self.engine.toggle_camera(CAMERAS[n - 5].camera_id)
        if self.engine.paused:
            self.engine.refresh()

    def _load(self, number: int) -> None:
        paused = self.engine.paused
        self.engine.load_scenario(get_scenario(number))
        self.engine.paused = paused
        self.selected = None

    # ------------------------------------------------------------ text helpers

    def text(
        self,
        surface: pygame.Surface,
        s: str,
        pos: tuple[float, float],
        color: Color = TEXT,
        font: pygame.font.Font | None = None,
        anchor: str = "topleft",
    ) -> pygame.Rect:
        f = font or self.font
        key = (id(f), s, color)
        img = self._text_cache.get(key)
        if img is None:
            if len(self._text_cache) > 2000:
                self._text_cache.clear()
            img = f.render(s, True, color)
            self._text_cache[key] = img
        rect = img.get_rect(**{anchor: (round(pos[0]), round(pos[1]))})
        surface.blit(img, rect)
        return rect

    def wrapped(
        self,
        s: str,
        x: float,
        y: float,
        width: int,
        color: Color = TEXT,
        font: pygame.font.Font | None = None,
        prefix: str = "",
    ) -> float:
        f = font or self.font
        words = s.split()
        line = prefix
        indent = " " * len(prefix)
        for w in words:
            candidate = f"{line}{w}" if line.strip() == "" else f"{line} {w}"
            if f.size(candidate)[0] > width and line.strip():
                self.text(self.screen, line, (x, y), color, f)
                y += f.get_linesize()
                line = indent + w
            else:
                line = candidate
        if line.strip():
            self.text(self.screen, line, (x, y), color, f)
            y += f.get_linesize()
        return y

    # ------------------------------------------------------------ drawing

    def draw(self) -> None:
        self.screen.fill(BG)
        self.overlay.fill((0, 0, 0, 0))
        motor_on = self.engine.motor_outputs()
        self._draw_grid()
        self._draw_cameras()
        self._draw_ultrasonic()
        self._draw_obstacles()
        self._draw_perceived()
        self.screen.blit(self.overlay, (0, 0))
        self._draw_wearer(motor_on)
        self._draw_env_legend()
        self._draw_panel(motor_on)
        if self.show_help:
            self._draw_help()

    def _ring_px(self, clearance_cm: float) -> int:
        return round((WEARER_RADIUS_CM + clearance_cm) * self.view.scale)

    def _draw_grid(self) -> None:
        v = self.view
        center = (round(v.cx), round(v.cy))
        for m in MOTORS:  # motor sector boundaries
            b = m.bearing_deg + 22.5
            p = v.polar(b, 2000)
            _dashed_line(self.screen, GRID, center, p, 4, 8)
        rings = [
            (400, FAINT, "400"),
            (300, FAINT, "300"),
            (200, SEVERITY_COLORS[Severity.CAUTION], "200 caution"),
            (100, SEVERITY_COLORS[Severity.WARNING], "100 warning"),
            (50, SEVERITY_COLORS[Severity.CRITICAL], "50 critical"),
        ]
        for cm, color, label in rings:
            r = self._ring_px(cm)
            faded = _mix(color, BG, 0.62)
            pygame.draw.circle(self.screen, faded, center, r, 1)
            lx, ly = v.cx + r * 0.7071, v.cy + r * 0.7071  # rear-right diagonal: usually empty
            self.text(self.screen, label, (lx + 3, ly), faded, self.small, "topleft")

    def _wedge(
        self, bearing: float, half: float, r0_px: float, r1_px: float
    ) -> list[tuple[float, float]]:
        v = self.view
        pts: list[tuple[float, float]] = []
        steps = max(4, int(half))
        for i in range(steps + 1):
            b = math.radians(bearing - half + 2 * half * i / steps)
            pts.append((v.cx + r1_px * math.sin(b), v.cy - r1_px * math.cos(b)))
        for i in range(steps, -1, -1):
            b = math.radians(bearing - half + 2 * half * i / steps)
            pts.append((v.cx + r0_px * math.sin(b), v.cy - r0_px * math.cos(b)))
        return pts

    def _draw_cameras(self) -> None:
        eng = self.engine
        cfg = eng.sim_config.sensors
        r1 = self._ring_px(min(cfg.camera_max_range_cm, 2000))
        for cam in eng.cameras:
            half = cfg.camera_hfov_deg / 2
            color = CAM_COLOR if cam.enabled else (110, 60, 70)
            for edge in (-half, half):
                p0 = self.view.polar(cam.spec.bearing_deg + edge, WEARER_RADIUS_CM)
                p1 = self.view.polar(cam.spec.bearing_deg + edge, r1 / self.view.scale)
                _dashed_line(self.screen, _mix(color, BG, 0.45), p0, p1, 6, 6)
            if eng.mode is PerceptionMode.SENSORS:
                for det in cam.last_detections:  # bearing-only rays: no range!
                    p0 = self.view.polar(det.bearing_deg, WEARER_RADIUS_CM + 10)
                    p1 = self.view.polar(det.bearing_deg, r1 / self.view.scale)
                    pygame.draw.line(self.overlay, (*CAM_COLOR, 110), p0, p1, 1)

    def _draw_ultrasonic(self) -> None:
        eng = self.engine
        health = {h.sensor_id: h for h in eng.result.sensor_health}
        cfg = eng.config.ultrasonic
        r0 = self._ring_px(0)
        r1 = self._ring_px(cfg.max_range_cm)
        for spec in ULTRASONIC_SENSORS:
            mode = eng.ultrasonic.modes[spec.sensor_id]
            h = health.get(spec.sensor_id)
            degraded = mode is not SensorMode.ENABLED or (h is not None and h.status.is_degraded)
            if mode is SensorMode.FAULT:
                fill = (220, 70, 70, 34)
            elif degraded:
                fill = (120, 120, 130, 26)
            else:
                fill = (*US_BEAM, 34)
            poly = self._wedge(spec.bearing_deg, cfg.beam_half_angle_deg, r0, r1)
            pygame.draw.polygon(self.overlay, fill, poly)
            pygame.draw.polygon(self.overlay, (*fill[:3], 90), poly, 1)
            reading = eng.ultrasonic.last_emitted.get(spec.sensor_id)
            label_pos = self.view.polar(spec.bearing_deg, WEARER_RADIUS_CM + cfg.max_range_cm + 18)
            if (
                eng.mode is PerceptionMode.SENSORS
                and reading is not None
                and reading.status is RangeStatus.VALID
                and reading.distance_cm is not None
                and not degraded
            ):
                rr = self._ring_px(reading.distance_cm)
                arc = self._wedge(spec.bearing_deg, cfg.beam_half_angle_deg, rr - 2, rr + 2)
                pygame.draw.polygon(self.screen, US_READING, arc)
            self.text(self.screen, spec.sensor_id, label_pos, _mix(US_READING, BG, 0.2),
                      self.small, "center")  # fmt: skip

    def _draw_obstacles(self) -> None:
        v = self.view
        for o in self.engine.world.obstacles:
            c = v.to_screen(o.x, o.y)
            r = max(3, round(o.radius_cm * v.scale))
            pygame.draw.circle(self.overlay, (*OBSTACLE, 40), c, r)
            if o.echo_probability < 1.0:
                _dashed_circle(self.screen, OBSTACLE, c, r)
            else:
                pygame.draw.circle(self.screen, OBSTACLE, c, r, 2)
            if o.id == self.selected:
                pygame.draw.circle(self.screen, SELECT, c, r + 5, 2)
            if o.speed > 0.5:
                end = v.to_screen(o.x + o.vx, o.y + o.vy)  # 1 s look-ahead
                _arrow(self.screen, OBSTACLE, c, end)
            label = o.id + (f" {o.label}" if o.label else "")
            self.text(self.screen, label, (c[0], c[1] - r - 2), OBSTACLE, self.small, "midbottom")

    def _draw_perceived(self) -> None:
        """Where the *pipeline* believes hazards are (vs. ground-truth circles)."""
        for a in self.engine.result.report.assessments:
            if a.effective_distance_cm is None:
                continue
            color = SEVERITY_COLORS[a.severity] if a.is_actionable else FAINT
            p = self.view.polar(a.bearing_deg, WEARER_RADIUS_CM + a.effective_distance_cm)
            s = 6
            pts = [(p[0], p[1] - s), (p[0] + s, p[1]), (p[0], p[1] + s), (p[0] - s, p[1])]
            pygame.draw.polygon(self.screen, color, pts, 0 if a.is_actionable else 1)

    def _draw_wearer(self, motor_on: tuple[bool, ...]) -> None:
        v = self.view
        cx, cy = round(v.cx), round(v.cy)
        torso_r = max(30, round(WEARER_RADIUS_CM * v.scale))
        body = pygame.Rect(0, 0, round(torso_r * 2.2), round(torso_r * 1.5))
        body.center = (cx, cy)
        pygame.draw.ellipse(self.screen, (58, 66, 84), body)
        pygame.draw.ellipse(self.screen, (120, 130, 150), body, 2)
        pygame.draw.circle(self.screen, (92, 102, 124), (cx, cy), round(torso_r * 0.45))
        nose = [(cx, cy - torso_r - 12), (cx - 7, cy - torso_r + 1), (cx + 7, cy - torso_r + 1)]
        pygame.draw.polygon(self.screen, ACCENT, nose)
        self.text(self.screen, "FRONT", (cx, cy - torso_r - 14), ACCENT, self.small, "midbottom")
        frame = self.engine.result.frame
        for m in MOTORS:
            pid = frame.pattern_ids[m.index]
            rx, ry = body.width / 2 + 2, body.height / 2 + 2
            ang = math.radians(m.bearing_deg)
            p = (round(cx + rx * math.sin(ang)), round(cy - ry * math.cos(ang)))
            self._motor_dot(p, pid, motor_on[m.index], 7)

    def _motor_dot(self, p: tuple[int, int], pid: PatternId, on: bool, r: int) -> None:
        color = PATTERN_COLORS[pid]
        if pid is PatternId.OFF:
            pygame.draw.circle(self.screen, color, p, r)
            pygame.draw.circle(self.screen, FAINT, p, r, 1)
            return
        if on:
            g = round(r * 1.9)
            glow = pygame.Surface((g * 2, g * 2), pygame.SRCALPHA)
            pygame.draw.circle(glow, (*color, 80), (g, g), g)
            self.screen.blit(glow, (p[0] - g, p[1] - g))
            pygame.draw.circle(self.screen, color, p, r)
            pygame.draw.circle(self.screen, (255, 255, 255), p, r, 1)
        else:
            pygame.draw.circle(self.screen, _mix(color, BG, 0.65), p, r)
            pygame.draw.circle(self.screen, color, p, r, 1)
        if pid in (PatternId.CAUTION_APPROACH, PatternId.WARNING_APPROACH,
                   PatternId.CRITICAL_APPROACH):  # fmt: skip
            pygame.draw.circle(self.screen, color, p, r + 4, 1)

    def _draw_env_legend(self) -> None:
        x, y = 12, WINDOW_H - 92
        items = [
            ("circle", OBSTACLE, "ground-truth obstacle (dashed = weak echo)"),
            ("diamond", SEVERITY_COLORS[Severity.WARNING], "pipeline's perceived hazard (range)"),
            ("arc", US_READING, "ultrasonic measurement inside beam"),
            ("ray", CAM_COLOR, "camera detection: bearing only, no range"),
        ]
        for kind, color, label in items:
            cy = y + 8
            if kind == "circle":
                pygame.draw.circle(self.screen, color, (x + 8, cy), 6, 2)
            elif kind == "diamond":
                pygame.draw.polygon(self.screen, color, [(x + 8, cy - 6), (x + 14, cy),
                                                         (x + 8, cy + 6), (x + 2, cy)])  # fmt: skip
            elif kind == "arc":
                pygame.draw.rect(self.screen, color, (x + 2, cy - 2, 12, 4))
            else:
                pygame.draw.line(self.screen, color, (x, cy), (x + 16, cy), 1)
            self.text(self.screen, label, (x + 24, y), DIM, self.small)
            y += 19
        mode = self.engine.mode
        label = "GROUND TRUTH mode" if mode is PerceptionMode.GROUND_TRUTH else "SENSOR mode"
        sub = (
            "oracle: perfect observations, sensors bypassed"
            if mode is PerceptionMode.GROUND_TRUTH
            else "simulated ultrasonic + cameras -> shared pipeline"
        )
        color = (255, 120, 200) if mode is PerceptionMode.GROUND_TRUTH else US_READING
        self.text(self.screen, label, (12, 12), color, self.bold)
        self.text(self.screen, sub, (12, 32), DIM, self.small)
        if self.engine.paused:
            self.text(
                self.screen, "PAUSED", (ENV_W - 12, 12), (255, 210, 90), self.bold, "topright"
            )

    # ------------------------------------------------------------ side panel

    def _draw_panel(self, motor_on: tuple[bool, ...]) -> None:
        eng = self.engine
        pygame.draw.rect(self.screen, PANEL_BG, (PANEL_X, 0, PANEL_W, WINDOW_H))
        pygame.draw.line(self.screen, GRID, (PANEL_X, 0), (PANEL_X, WINDOW_H))
        x = PANEL_X + 16
        w = PANEL_W - 32
        y = 12.0
        self.text(self.screen, "HapticSight Simulator", (x, y), TEXT, self.title)
        self.text(self.screen, "no hardware connected", (x + w, y + 6), DIM, self.small, "topright")
        y += 30
        sc = eng.scenario
        state = "PAUSED" if eng.paused else "RUNNING"
        status = f"t={eng.time:5.1f}s  {state}  {SPEEDS[self.speed_index]:g}x  seed {eng.seed}"
        self.text(self.screen, f"{sc.number}. {sc.name}", (x, y), ACCENT, self.bold)
        y += 20
        self.text(self.screen, status, (x, y), DIM, self.small)
        y += 18
        y = self.wrapped(sc.description, x, y, w, TEXT, self.small)
        for line in sc.expected:
            y = self.wrapped(line, x, y, w, DIM, self.small, prefix="• ")
        y += 8
        y = self._section(x, y, w, "Haptic output (8 motors)")
        y = self._draw_vest(x, y, w, motor_on)
        y = self._section(x, y, w, "Sensors")
        y = self._draw_sensors(x, y)
        y = self._section(x, y, w, "Hazards (most relevant first)")
        y = self._draw_hazards(x, y, w)
        self._draw_footer(x)

    def _section(self, x: float, y: float, w: float, title: str) -> float:
        self.text(self.screen, title.upper(), (x, y), (150, 160, 182), self.small)
        pygame.draw.line(self.screen, GRID, (x, y + 16), (x + w, y + 16))
        return y + 22

    def _draw_vest(self, x: float, y: float, w: float, motor_on: tuple[bool, ...]) -> float:
        frame = self.engine.result.frame
        cx, cy, ring = x + 62, y + 66, 46
        pygame.draw.ellipse(self.screen, (50, 57, 74), (cx - 34, cy - 24, 68, 48))
        self.text(self.screen, "▲", (cx, cy - 4), ACCENT, self.small, "center")
        for m in MOTORS:
            ang = math.radians(m.bearing_deg)
            p = (round(cx + ring * math.sin(ang)), round(cy - ring * math.cos(ang)))
            self._motor_dot(p, frame.pattern_ids[m.index], motor_on[m.index], 9)
            lp = (cx + (ring + 22) * math.sin(ang), cy - (ring + 17) * math.cos(ang))
            self.text(self.screen, m.name, lp, DIM, self.small, "center")
        tx = x + 146
        ty = y
        now = self.engine.time
        for ch in frame.channels:
            name = PATTERN_NAMES[ch.pattern_id] + ("*" if ch.held else "")
            color = PATTERN_COLORS[ch.pattern_id] if ch.active else FAINT
            m = MOTORS[ch.motor]
            self.text(self.screen, m.name, (tx, ty), DIM, self.small)
            self.text(self.screen, name, (tx + 26, ty), color, self.small)
            self._pulse_trace(ch.motor, tx + 136, ty + 2, x + w - (tx + 136), 10, color, now)
            if ch.active:
                self.text(self.screen, ch.pattern.describe(), (tx + 26, ty + 13), FAINT,
                          self.small)  # fmt: skip
                ty += 13
            ty += 15
        self.text(self.screen, "* held by anti-flicker hysteresis   trace: last 2 s",
                  (x, max(y + 136, ty + 2)), FAINT, self.small)  # fmt: skip
        return max(y + 156, ty + 22)

    def _pulse_trace(
        self, motor: int, x: float, y: float, w: float, h: int, color: Color, now: float
    ) -> None:
        """Last 2 s of the motor's on/off waveform, computed from pattern timing."""
        sched = self.engine.scheduler
        pattern, started = sched.pattern_state(motor)
        pygame.draw.line(self.screen, GRID, (x, y + h), (x + w, y + h))
        if pattern is None or pattern.is_silent or sched.expired(now):
            return
        window = 2.0
        n = int(w)
        for i in range(n):
            t = now - window + window * i / n
            if t >= started and pattern.is_on(int((t - started) * 1000)):
                pygame.draw.line(self.screen, color, (x + i, y), (x + i, y + h))

    def _draw_sensors(self, x: float, y: float) -> float:
        eng = self.engine
        health = {h.sensor_id: h for h in eng.result.sensor_health}
        for col, spec in enumerate(ULTRASONIC_SENSORS):
            mode = eng.ultrasonic.modes[spec.sensor_id]
            h = health.get(spec.sensor_id)
            if eng.mode is PerceptionMode.GROUND_TRUTH:
                value, color = "bypassed", FAINT
            elif mode is SensorMode.DROPOUT:
                age = f" ({h.age_s:.1f}s old)" if h and h.age_s is not None else ""
                value, color = f"DROPOUT{age}", (200, 130, 130)
            elif mode is SensorMode.FAULT:
                value, color = "FAULT", (240, 90, 90)
            elif h is None or h.status is SensorStatus.NEVER_SEEN:
                value, color = "waiting", DIM
            elif h.status is SensorStatus.OK:
                value, color = format_distance(h.last_distance_cm), US_READING
            elif h.status is SensorStatus.NO_ECHO:
                value, color = "no echo", DIM
            else:
                value, color = h.status.value.upper(), (240, 90, 90)
            px = x + (col % 2) * 232
            self.text(self.screen, f"{spec.sensor_id} {spec.label:<5}", (px, y), DIM, self.small)
            self.text(self.screen, value, (px + 80, y), color, self.small)
            if col % 2 == 1:
                y += 16
        for i, cam in enumerate(eng.cameras):
            px = x + i * 232
            if cam.enabled:
                value, color = f"{len(cam.last_detections)} detections", CAM_COLOR
            else:
                value, color = "OFF", (240, 90, 90)
            self.text(self.screen, cam.spec.camera_id, (px, y), DIM, self.small)
            self.text(self.screen, value, (px + 80, y), color, self.small)
        return y + 24

    def _draw_hazards(self, x: float, y: float, w: float) -> float:
        report = self.engine.result.report
        if not report.assessments:
            self.text(self.screen, "no observations", (x, y), FAINT, self.small)
            return y + 18
        limit = WINDOW_H - 60
        for a in report.assessments:
            if y > limit - 30:
                self.text(self.screen, "…", (x, y), FAINT, self.small)
                break
            y = self._hazard_row(a, x, y, w)
        return y

    def _hazard_row(self, a: RiskAssessment, x: float, y: float, w: float) -> float:
        color = SEVERITY_COLORS[a.severity] if a.is_actionable else FAINT
        tag = a.severity.name if a.is_actionable else "ignored"
        src = {SourceType.ULTRASONIC: "US", SourceType.CAMERA: "CAM",
               SourceType.FUSED: "FUS", SourceType.SIMULATION: "GT"}[a.source]  # fmt: skip
        ttc = f" TTC {a.time_to_contact_s:.1f}s" if a.time_to_contact_s is not None else ""
        line = (
            f"{tag:<8} {src:<3} {format_bearing(a.bearing_deg):>4} "
            f"{format_distance(a.effective_distance_cm):>7}  {a.motion.value}{ttc}"
        )
        self.text(self.screen, line, (x, y), color, self.small)
        y += 14
        detail = f"{a.id}  conf {a.observation.confidence:.2f}"
        if a.observation.metadata.get("shared_range"):
            detail += "  shared echo"
        if not a.is_actionable:
            detail += "  - " + a.reasons[-1]
        self.text(self.screen, _ellipsize(self.small, detail, w), (x + 12, y), FAINT, self.small)
        return y + 16

    def _draw_footer(self, x: float) -> None:
        y = WINDOW_H - 40
        pygame.draw.line(self.screen, GRID, (PANEL_X, y - 6), (WINDOW_W, y - 6))
        sel = self.engine.world.get(self.selected) if self.selected else None
        if sel:
            info = (
                f"selected {sel.id}: {format_bearing(sel.bearing_deg)} "
                f"{sel.clearance_cm:.0f} cm  v=({sel.vx:+.0f},{sel.vy:+.0f}) cm/s  "
                f"closing {sel.closing_speed_cm_s:+.0f}"
            )
        else:
            info = "click the environment to add an obstacle"
        self.text(self.screen, info, (x, y), TEXT, self.small)
        self.text(
            self.screen,
            "H all controls   Space pause   R reset   1-0 scenarios   G mode",
            (x, y + 16),
            DIM,
            self.small,
        )

    def _draw_help(self) -> None:
        shade = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
        shade.fill((8, 10, 14, 215))
        self.screen.blit(shade, (0, 0))
        w, h = 560, 40 + 19 * len(HELP_LINES) + 60
        box = pygame.Rect(0, 0, w, h)
        box.center = (WINDOW_W // 2, WINDOW_H // 2)
        pygame.draw.rect(self.screen, PANEL_BG, box, border_radius=10)
        pygame.draw.rect(self.screen, GRID, box, 1, border_radius=10)
        y = box.top + 16
        self.text(self.screen, "Controls", (box.left + 20, y), TEXT, self.title)
        y += 34
        for key, desc in HELP_LINES:
            color = ACCENT if not desc else TEXT
            self.text(self.screen, key, (box.left + 20, y), color, self.font)
            self.text(self.screen, desc, (box.left + 250, y), DIM, self.font)
            y += 19
        names = ", ".join(f"{s.number} {s.key}" for s in SCENARIOS)
        self.wrapped(f"Scenarios: {names}", box.left + 20, y + 6, w - 40, FAINT, self.small)


# ------------------------------------------------------------ drawing utils


def _font(size: int, bold: bool = False) -> pygame.font.Font:
    names = "menlo,sfmono,dejavusansmono,consolas,couriernew,monospace"
    try:
        return pygame.font.SysFont(names, size, bold=bold)
    except Exception:  # pragma: no cover - font discovery is platform dependent
        return pygame.font.Font(None, size + 4)


def _mix(a: Color, b: Color, t: float) -> Color:
    return (
        round(a[0] + (b[0] - a[0]) * t),
        round(a[1] + (b[1] - a[1]) * t),
        round(a[2] + (b[2] - a[2]) * t),
    )


def _ellipsize(font: pygame.font.Font, s: str, width: float) -> str:
    if font.size(s)[0] <= width:
        return s
    while s and font.size(s + "…")[0] > width:
        s = s[:-1]
    return s + "…"


def _dashed_line(
    surface: pygame.Surface,
    color: Color,
    p0: tuple[float, float],
    p1: tuple[float, float],
    dash: int,
    gap: int,
) -> None:
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return
    ux, uy = dx / length, dy / length
    pos = 0.0
    while pos < length:
        end = min(pos + dash, length)
        pygame.draw.line(
            surface,
            color,
            (p0[0] + ux * pos, p0[1] + uy * pos),
            (p0[0] + ux * end, p0[1] + uy * end),
        )
        pos += dash + gap


def _dashed_circle(surface: pygame.Surface, color: Color, c: tuple[int, int], r: int) -> None:
    segments = max(8, r // 2)
    for i in range(0, segments, 2):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        pygame.draw.line(
            surface,
            color,
            (c[0] + r * math.cos(a0), c[1] + r * math.sin(a0)),
            (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1)),
            2,
        )


def _arrow(surface: pygame.Surface, color: Color, p0: tuple[int, int], p1: tuple[int, int]) -> None:
    pygame.draw.line(surface, color, p0, p1, 2)
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    for da in (2.6, -2.6):
        pygame.draw.line(
            surface,
            color,
            p1,
            (p1[0] + 9 * math.cos(ang + da), p1[1] + 9 * math.sin(ang + da)),
            2,
        )
