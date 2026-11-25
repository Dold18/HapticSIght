"""Live dual-camera diagnostic application (Pygame).

Shows both camera feeds with tracked detections and bearings, camera health and
performance, the shared pipeline's risk assessments, and the eight virtual
haptic motors. Ultrasonic range comes from a hand-controlled **simulated**
source until the Arduino exists, and is labelled as such everywhere.

Rendering never blocks capture or inference: capture and inference run on
their own threads (:class:`VisionSystem`); this loop only reads snapshots.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import replace
from pathlib import Path
from typing import IO, Any

import pygame

from hapticsight.config import HapticSightConfig
from hapticsight.core.clock import MonotonicClock
from hapticsight.core.format import format_bearing, format_distance
from hapticsight.core.geometry import polar_to_cartesian
from hapticsight.core.layout import MOTORS, ULTRASONIC_SENSORS
from hapticsight.core.models import SourceType
from hapticsight.haptics.patterns import PatternId
from hapticsight.haptics.scheduler import PulseScheduler
from hapticsight.hardware.arduino import LinkState
from hapticsight.hardware.inputs import ManualInput, UltrasonicInput
from hapticsight.hardware.runtime import HardwareRuntime
from hapticsight.hardware.telemetry import TelemetryLogger
from hapticsight.pipeline import HapticSightPipeline, PipelineResult
from hapticsight.protocol.messages import FirmwareMode
from hapticsight.simulation.app import (
    FAINT,
    GRID,
    PATTERN_COLORS,
    PATTERN_NAMES,
    SEVERITY_COLORS,
    _dashed_line,
    _font,
    _mix,
)
from hapticsight.simulation.manual_ultrasonic import ManualMode, ManualUltrasonic
from hapticsight.vision.camera import CameraState
from hapticsight.vision.config import calibration_to_toml
from hapticsight.vision.processor import FrameResult, VisionSnapshot, VisionSystem
from hapticsight.vision.recording import VideoRecorder
from hapticsight.vision.tracker import TrackState

Color = tuple[int, int, int]
W, H = 1360, 820
CAM_W, CAM_H = 640, 360
BG: Color = (16, 19, 26)
PANEL: Color = (24, 28, 37)
TEXT: Color = (222, 226, 235)
DIM: Color = (135, 143, 160)
GOOD: Color = (110, 220, 140)
BAD: Color = (240, 90, 90)
WARN: Color = (240, 190, 70)
SIM: Color = (235, 110, 220)  # everything simulated is magenta
REAL: Color = (110, 220, 140)  # real hardware measurements are green
CAM_COLOR: Color = (120, 160, 255)

STATE_COLORS = {
    CameraState.RUNNING: GOOD,
    CameraState.STARTING: WARN,
    CameraState.STALLED: WARN,
    CameraState.DISCONNECTED: BAD,
    CameraState.UNAVAILABLE: BAD,
    CameraState.DISABLED: DIM,
    CameraState.FINISHED: DIM,
    CameraState.STOPPED: DIM,
}

HELP = (
    "Tab: select camera    F / B: front / rear camera on-off    P: pause inference",
    "1-4: select sensor S0-S3    Up/Down: simulated range ±10 cm (Shift ±50)",
    "E: echo / no echo    D: dropout    A: approach 50 cm/s    S: stop motion",
    "L: stop / resume host commands to the Arduino (link-loss demo)",
    "C: calibration overlay    , .: yaw ∓0.5°    - =: HFOV ∓1°    W: write calibration",
    "K: save screenshot    H: help    Esc / Q: quit",
)


class LiveVisionApp:
    def __init__(
        self,
        vision: VisionSystem,
        config: HapticSightConfig | None = None,
        ultrasonic: UltrasonicInput | ManualUltrasonic | None = None,
        detector_label: str = "",
        record_dir: Path | None = None,
        calibration_path: Path = Path("config/cameras.local.toml"),
        fps: int = 30,
        loop_hz: float = 15.0,
        logger: TelemetryLogger | None = None,
    ) -> None:
        pygame.init()
        pygame.display.set_caption("HapticSight Live Vision")
        self.screen = pygame.display.set_mode((W, H))
        self.vision = vision
        self.config = config or HapticSightConfig()
        self.pipeline = HapticSightPipeline(self.config)
        self.scheduler = PulseScheduler()
        if ultrasonic is None or isinstance(ultrasonic, ManualUltrasonic):
            ultrasonic = ManualInput(ultrasonic)
        self.ultrasonic: UltrasonicInput = ultrasonic
        self.clock = MonotonicClock()
        # The same step function as the headless hardware runtime (no separate logic).
        self.runtime = HardwareRuntime(
            ultrasonic, self.pipeline, vision, self.clock, logger, loop_hz=loop_hz
        )
        self.loop_hz = loop_hz
        self._next_step = 0.0
        self.fps = fps
        self.detector_label = detector_label or vision.detector.name
        self.calibration_path = calibration_path
        self.font = _font(13)
        self.small = _font(11)
        self.bold = _font(14, bold=True)
        self.title = _font(17, bold=True)
        self.selected_sensor = "S0"
        self.selected_camera = vision.captures[0].camera_id if vision.captures else ""
        self.calibrating = False
        self.show_help = False
        self.running = True
        self.message = ""
        self.message_until = 0.0
        self.result: PipelineResult | None = None
        self.frames_rendered = 0
        self._tick = pygame.time.Clock()
        self._surface_cache: dict[str, tuple[int, pygame.Surface]] = {}
        self._recorders: dict[str, VideoRecorder] = {}
        self._last_rec_seq: dict[str, int] = {}
        self._log: IO[str] | None = None
        self._next_log = 0.0
        self.record_dir: Path | None = None
        if record_dir is not None:
            stamp = time.strftime("%Y%m%d-%H%M%S")
            self.record_dir = record_dir / f"session-{stamp}"
            self.record_dir.mkdir(parents=True, exist_ok=True)
            self._log = open(self.record_dir / "session.jsonl", "w")  # noqa: SIM115

    # ------------------------------------------------------------ loop

    def run(self, max_frames: int | None = None) -> None:
        self.vision.start()
        try:
            while self.running:
                self._tick.tick(self.fps)
                for event in pygame.event.get():
                    self.handle_event(event)
                self.step()
                self.draw()
                pygame.display.flip()
                self.frames_rendered += 1
                if max_frames is not None and self.frames_rendered >= max_frames:
                    break
        finally:
            self.close()

    def close(self) -> None:
        self.runtime.close()  # stops vision, switches Arduino motors off, closes the log
        for rec in self._recorders.values():
            rec.close()
        if self._log is not None:
            self._log.close()
            self._log = None
        pygame.quit()

    def step(self) -> VisionSnapshot:
        now = self.clock.now()
        if self.result is None or now >= self._next_step:  # host loop at loop_hz
            self._next_step = now + 1.0 / self.loop_hz
            self.result = self.runtime.step(now).result
            self.scheduler.apply(self.result.frame, now)
        snap = self.vision.snapshot()
        self.snapshot = snap
        self._record(snap, now)
        return snap

    # ------------------------------------------------------------ recording

    def _record(self, snap: VisionSnapshot, now: float) -> None:
        if self.record_dir is None:
            return
        for cap in self.vision.captures:
            frame = cap.slot.latest()
            if frame is None or self._last_rec_seq.get(cap.camera_id) == frame.seq:
                continue
            self._last_rec_seq[cap.camera_id] = frame.seq
            rec = self._recorders.get(cap.camera_id)
            if rec is None:
                fps = snap.statuses[cap.camera_id].fps or 30.0
                rec = VideoRecorder(
                    self.record_dir / f"{cap.camera_id}.mp4", fps, (frame.width, frame.height)
                )
                self._recorders[cap.camera_id] = rec
            rec.add(frame)
        if self._log is not None and self.result is not None and now >= self._next_log:
            self._next_log = now + 0.1
            r = self.result
            entry = {
                "t": round(now, 4),
                "ultrasonic_source": self.ultrasonic.kind.value,
                "ultrasonic_simulated": self.ultrasonic.kind.simulated,
                "ultrasonic": {
                    spec.sensor_id: self.ultrasonic.display(spec.sensor_id).text
                    for spec in ULTRASONIC_SENSORS
                },
                "haptics": r.frame.pattern_string(),
                "assessments": [
                    {
                        "id": a.id,
                        "source": a.source.value,
                        "bearing_deg": round(a.bearing_deg, 1),
                        "range_cm": a.effective_distance_cm,
                        "severity": a.severity.name,
                    }
                    for a in r.report.top(8)
                ],
                "cameras": {
                    k: {
                        "state": st.state.value,
                        "fps": round(st.fps, 1),
                        "inference_ms": round(snap.perf[k].inference_ms, 1),
                        "latency_ms": round(snap.perf[k].latency_ms, 1),
                    }
                    for k, st in snap.statuses.items()
                },
            }
            self._log.write(json.dumps(entry) + "\n")

    # ------------------------------------------------------------ input

    def flash(self, msg: str) -> None:
        self.message, self.message_until = msg, time.monotonic() + 4.0

    def handle_event(self, event: pygame.event.Event) -> None:
        if event.type == pygame.QUIT:
            self.running = False
        elif event.type == pygame.KEYDOWN:
            self._on_key(event.key, bool(event.mod & pygame.KMOD_SHIFT))

    def _capture(self, camera_id: str) -> Any:
        return next((c for c in self.vision.captures if c.camera_id == camera_id), None)

    def _on_key(self, key: int, shift: bool) -> None:
        manual = self.ultrasonic.manual  # only simulated sources accept hand-set ranges
        sensor_keys = {pygame.K_1: "S0", pygame.K_2: "S1", pygame.K_3: "S2", pygame.K_4: "S3"}
        range_keys = (pygame.K_UP, pygame.K_DOWN, pygame.K_e, pygame.K_d, pygame.K_a, pygame.K_s)
        if key in range_keys:
            if manual is None:
                self.flash("real sensors: ranges cannot be set by hand")
                return
            self._manual_key(manual, key, shift)
            self.ultrasonic.manual_changed()
            return
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif key == pygame.K_h:
            self.show_help = not self.show_help
        elif key in sensor_keys:
            self.selected_sensor = sensor_keys[key]
        elif key == pygame.K_l:
            bridge = getattr(self.ultrasonic, "bridge", None)
            if bridge is None:
                self.flash("no Arduino link in this mode")
            else:
                bridge.commands_enabled = not bridge.commands_enabled
                state = "resumed" if bridge.commands_enabled else "STOPPED (expect FALLBACK)"
                self.flash(f"host commands {state}")
        elif key == pygame.K_TAB and self.vision.captures:
            ids = [c.camera_id for c in self.vision.captures]
            self.selected_camera = ids[(ids.index(self.selected_camera) + 1) % len(ids)]
        elif key in (pygame.K_f, pygame.K_b):
            cid = "CAM_FRONT" if key == pygame.K_f else "CAM_REAR"
            cap = self._capture(cid)
            if cap is not None:
                cap.set_enabled(not cap.config.enabled)
                self.flash(f"{cid} {'enabled' if cap.config.enabled else 'disabled'}")
        elif key == pygame.K_p:
            self.vision.paused = not self.vision.paused
        elif key == pygame.K_c:
            self.calibrating = not self.calibrating
        elif key in (pygame.K_COMMA, pygame.K_PERIOD, pygame.K_MINUS, pygame.K_EQUALS):
            self._adjust_calibration(key)
        elif key == pygame.K_w:
            self._write_calibration()
        elif key == pygame.K_k:
            path = Path("recordings") / f"live-{time.strftime('%Y%m%d-%H%M%S')}.png"
            path.parent.mkdir(exist_ok=True)
            pygame.image.save(self.screen, str(path))
            self.flash(f"saved {path} (local only)")

    def _manual_key(self, manual: ManualUltrasonic, key: int, shift: bool) -> None:
        us = manual.sensors[self.selected_sensor]
        if key in (pygame.K_UP, pygame.K_DOWN):
            step = (50 if shift else 10) * (1 if key == pygame.K_UP else -1)
            us.mode = ManualMode.ECHO
            us.speed_cm_s = 0.0
            us.distance_cm = min(us.max_distance_cm, max(us.min_distance_cm, us.distance_cm + step))
        elif key == pygame.K_e:
            us.mode = ManualMode.NO_ECHO if us.mode is ManualMode.ECHO else ManualMode.ECHO
        elif key == pygame.K_d:
            us.mode = ManualMode.ECHO if us.mode is ManualMode.DROPOUT else ManualMode.DROPOUT
        elif key == pygame.K_a:
            us.mode = ManualMode.ECHO
            us.speed_cm_s = 50.0
        elif key == pygame.K_s:
            us.speed_cm_s = 0.0

    def _source_color(self) -> Color:
        if self.ultrasonic.title().startswith("ULTRASONIC: DISCONNECTED"):
            return BAD
        return SIM if self.ultrasonic.kind.simulated else REAL

    def _adjust_calibration(self, key: int) -> None:
        proc = self.vision.processors.get(self.selected_camera)
        if proc is None:
            return
        geo = proc.geometry
        cal = geo.calibration
        if key in (pygame.K_COMMA, pygame.K_PERIOD):
            delta = -0.5 if key == pygame.K_COMMA else 0.5
            cal = replace(cal, yaw_offset_deg=cal.yaw_offset_deg + delta)
        else:
            delta = -1.0 if key == pygame.K_MINUS else 1.0
            cal = replace(
                cal, hfov_deg=min(170.0, max(10.0, cal.hfov_deg + delta)), fx=None, cx=None,
                reference_width=None,
            )  # fmt: skip
        cal = replace(
            cal,
            calibrated=True,
            hfov_uncertainty_deg=min(cal.hfov_uncertainty_deg, 3.0),
            yaw_uncertainty_deg=min(cal.yaw_uncertainty_deg, 2.0),
            note="manual overlay alignment",
        )
        proc.set_geometry(replace(geo, calibration=cal))
        self.calibrating = True
        self.flash(
            f"{self.selected_camera}: HFOV {cal.hfov_deg:.1f}°  yaw {cal.yaw_offset_deg:+.1f}°"
        )

    def _write_calibration(self) -> None:
        proc = self.vision.processors.get(self.selected_camera)
        if proc is None:
            return
        block = calibration_to_toml(self.selected_camera, proc.geometry.calibration)
        path = self.calibration_path
        path.parent.mkdir(parents=True, exist_ok=True)
        existing = path.read_text() if path.exists() else ""
        header = f"[cameras.{self.selected_camera}.calibration]"
        if header in existing:  # replace the old block for this camera
            start = existing.index(header)
            nxt = existing.find("\n[", start + len(header))
            existing = existing[:start] + (existing[nxt + 1 :] if nxt != -1 else "")
        path.write_text(existing.rstrip() + ("\n\n" if existing.strip() else "") + block)
        self.flash(f"wrote {self.selected_camera} calibration to {path}")

    # ------------------------------------------------------------ drawing helpers

    def text(
        self, s: str, pos: tuple[float, float], color: Color = TEXT,
        font: pygame.font.Font | None = None, anchor: str = "topleft",
    ) -> pygame.Rect:  # fmt: skip
        img = (font or self.font).render(s, True, color)
        rect = img.get_rect(**{anchor: (round(pos[0]), round(pos[1]))})
        self.screen.blit(img, rect)
        return rect

    def draw(self) -> None:
        self.screen.fill(BG)
        snap = getattr(self, "snapshot", None) or self.vision.snapshot()
        for i, cap in enumerate(self.vision.captures[:2]):
            self._draw_camera(cap.camera_id, 20 + i * (CAM_W + 40), 30, snap)
        motor_on = self.scheduler.outputs(self.clock.now())
        self._draw_map(20, 414, 380, motor_on)
        self._draw_middle(420, 414)
        self._draw_right(800, 414, snap)
        if self.show_help:
            self._draw_help()
        if self.message and time.monotonic() < self.message_until:
            self.text(self.message, (W // 2, H - 14), WARN, self.font, "center")

    def _camera_surface(self, cid: str, frame: Any) -> pygame.Surface:
        import cv2

        cached = self._surface_cache.get(cid)
        if cached is not None and cached[0] == frame.seq:
            return cached[1]
        h, w = frame.image.shape[:2]
        scale = min(CAM_W / w, CAM_H / h)
        size = (max(1, int(w * scale)), max(1, int(h * scale)))
        rgb = cv2.cvtColor(cv2.resize(frame.image, size), cv2.COLOR_BGR2RGB)
        surf = pygame.image.frombuffer(rgb.tobytes(), size, "RGB").copy()
        self._surface_cache[cid] = (frame.seq, surf)
        return surf

    def _draw_camera(self, cid: str, x: int, y: int, snap: VisionSnapshot) -> None:
        st = snap.statuses[cid]
        perf = snap.perf.get(cid)
        proc = self.vision.processors[cid]
        sel = "▶ " if cid == self.selected_camera else ""
        mount = format_bearing(proc.geometry.mount_bearing_deg)
        device = proc.camera.device
        source = camera_source_label(device)
        name = self.text(f"{sel}{cid} ({mount}) {source}", (x, y - 28), TEXT, self.bold)
        self.text(st.state.value.upper(), (name.right + 10, y - 28), STATE_COLORS[st.state],
                  self.bold)  # fmt: skip
        perf_txt = (
            f"cap {st.fps:4.1f} fps | infer {perf.inference_fps:4.1f} fps "
            f"{perf.inference_ms:4.1f} ms | cap->obs {perf.latency_ms:4.0f} ms"
            if perf and perf.processed_frames
            else "no inference yet"
        )
        self.text(perf_txt, (x + CAM_W, y - 13), DIM, self.small, "topright")
        rect = pygame.Rect(x, y, CAM_W, CAM_H)
        pygame.draw.rect(self.screen, PANEL, rect)
        frame = snap.frames.get(cid)
        result = snap.results.get(cid)
        if frame is None or result is None or st.state in (
            CameraState.DISABLED, CameraState.UNAVAILABLE, CameraState.STOPPED,
        ):  # fmt: skip
            msg = st.detail or st.state.value
            self.text(st.state.value.upper(), rect.center, STATE_COLORS[st.state], self.title,
                      "center")  # fmt: skip
            for i, line in enumerate(_wrap(msg, 80)):
                self.text(line, (rect.centerx, rect.centery + 24 + 16 * i), DIM, self.small,
                          "center")  # fmt: skip
            pygame.draw.rect(self.screen, GRID, rect, 1)
            return
        surf = self._camera_surface(cid, frame)
        ox = x + (CAM_W - surf.get_width()) // 2
        oy = y + (CAM_H - surf.get_height()) // 2
        self.screen.blit(surf, (ox, oy))
        scale = surf.get_width() / frame.width
        if self.calibrating and cid == self.selected_camera:
            self._draw_bearing_grid(proc, frame.width, ox, oy, scale, surf.get_height())
        self._draw_tracks(result, ox, oy, scale)
        age = st.last_frame_age_s
        if st.state is CameraState.STALLED or (age is not None and age > 0.5):
            self.text(f"STALE FRAME ({age:.1f}s)", (x + 8, y + 8), BAD, self.bold)
        cal = proc.geometry.calibration
        cal_txt = (
            f"HFOV {proc.geometry.effective_hfov_deg(frame.width):.1f}° "
            f"yaw {cal.yaw_offset_deg:+.1f}° "
            + ("calibrated" if cal.calibrated else "UNCALIBRATED (nominal FOV)")
        )
        self.text(cal_txt, (x + 6, y + CAM_H - 16), WARN if not cal.calibrated else DIM, self.small)
        self.text(f"{frame.width}x{frame.height}", (x + CAM_W - 6, y + CAM_H - 16), DIM,
                  self.small, "topright")  # fmt: skip
        pygame.draw.rect(self.screen, GRID, rect, 1)

    def _draw_bearing_grid(
        self, proc: Any, width: int, ox: int, oy: int, scale: float, height: int
    ) -> None:
        geo = proc.geometry
        for b in range(-90, 91, 10):
            u = geo.column_for_body_bearing(geo.mount_bearing_deg + b, width)
            if u is None or not 0 <= u <= width:
                continue
            px = ox + u * scale
            color = WARN if b == 0 else (200, 200, 200)
            pygame.draw.line(self.screen, color, (px, oy), (px, oy + height), 1)
            self.text(
                format_bearing(geo.mount_bearing_deg + b), (px + 2, oy + 2), color, self.small
            )

    def _draw_tracks(self, result: FrameResult, ox: int, oy: int, scale: float) -> None:
        for tr in result.tracks:
            b = tr.detection.box
            r = pygame.Rect(ox + b.x1 * scale, oy + b.y1 * scale, b.width * scale, b.height * scale)
            confirmed = tr.state is TrackState.CONFIRMED
            color = (90, 230, 120) if confirmed else (170, 170, 170)
            pygame.draw.rect(self.screen, color, r, 2 if tr.updated else 1)
            tid = tr.track_id.split(":")[-1]
            label = (
                f"{tid} {tr.label} {tr.confidence:.2f}  {format_bearing(tr.detection.bearing_deg)}"
                f"±{tr.detection.bearing_uncertainty_deg:.0f}°"
            )
            img = self.small.render(label, True, (10, 10, 10))
            bg = pygame.Rect(r.x, max(oy, r.y - 15), img.get_width() + 6, 15)
            pygame.draw.rect(self.screen, color, bg)
            self.screen.blit(img, (bg.x + 3, bg.y + 1))

    # ------------------------------------------------------------ map

    def _draw_map(self, x: int, y: int, size: int, motor_on: tuple[bool, ...]) -> None:
        rect = pygame.Rect(x, y, size, size)
        pygame.draw.rect(self.screen, PANEL, rect)
        pygame.draw.rect(self.screen, GRID, rect, 1)
        cx, cy = rect.center
        scale = (size / 2 - 20) / 420.0

        def pt(bearing: float, cm: float) -> tuple[float, float]:
            px, py = polar_to_cartesian(bearing, cm)
            return cx + px * scale, cy - py * scale

        for cm, label in ((50, "50"), (100, "100"), (200, "200 cm")):
            r = int((cm + 20) * scale)
            pygame.draw.circle(self.screen, GRID, (cx, cy), r, 1)
            self.text(label, (cx + r * 0.71 + 2, cy + r * 0.71), FAINT, self.small)
        # camera FOV wedges (from current calibration)
        for cid, proc in self.vision.processors.items():
            geo = proc.geometry
            st = self.snapshot.statuses[cid] if hasattr(self, "snapshot") else None
            width = st.width if st and st.width else 1280
            half = geo.effective_hfov_deg(width) / 2
            center = geo.mount_bearing_deg + geo.calibration.yaw_offset_deg
            color = CAM_COLOR if proc.camera.enabled else (110, 60, 70)
            for edge in (-half, half):
                _dashed_line(self.screen, _mix(color, BG, 0.4), pt(center + edge, 20),
                             pt(center + edge, 400), 5, 5)  # fmt: skip
        # ultrasonic beams + readings (magenta = simulated, green = real hardware)
        beam = self.config.ultrasonic.beam_half_angle_deg
        us_color = self._source_color()
        for spec in ULTRASONIC_SENSORS:
            d = self.ultrasonic.display(spec.sensor_id)
            for edge in (-beam, beam):
                pygame.draw.line(self.screen, _mix(us_color, BG, 0.7),
                                 pt(spec.bearing_deg + edge, 20),
                                 pt(spec.bearing_deg + edge, 400))  # fmt: skip
            if d.distance_cm is not None:
                pts = [pt(spec.bearing_deg + a, d.distance_cm + 20) for a in range(-15, 16, 3)]
                pygame.draw.lines(self.screen, us_color, False, pts, 3)
        # observations
        if self.result is not None:
            for a in self.result.report.assessments:
                obs = a.observation
                color = SEVERITY_COLORS[a.severity] if a.is_actionable else FAINT
                if obs.source is SourceType.CAMERA:
                    p0, p1 = pt(obs.bearing_deg, 25), pt(obs.bearing_deg, 400)
                    pygame.draw.line(self.screen, CAM_COLOR if not a.is_stale else FAINT, p0, p1, 1)
                    label = f"{obs.id.split(':')[-1]} {obs.metadata.get('label', '')}"
                    self.text(label, p1, CAM_COLOR, self.small, "center")
                elif a.effective_distance_cm is not None:
                    p = pt(obs.bearing_deg, a.effective_distance_cm + 20)
                    s6 = 6
                    pygame.draw.polygon(self.screen, color, [(p[0], p[1] - s6), (p[0] + s6, p[1]),
                                        (p[0], p[1] + s6), (p[0] - s6, p[1])])  # fmt: skip
        # wearer + motors
        pygame.draw.ellipse(self.screen, (58, 66, 84), (cx - 22, cy - 15, 44, 30))
        pygame.draw.polygon(self.screen, CAM_COLOR, [(cx, cy - 27), (cx - 6, cy - 16),
                                                     (cx + 6, cy - 16)])  # fmt: skip
        frame = self.result.frame if self.result else None
        for m in MOTORS:
            pid = frame.pattern_ids[m.index] if frame else PatternId.OFF
            ang = math.radians(m.bearing_deg)
            p = (round(cx + 30 * math.sin(ang)), round(cy - 22 * math.cos(ang)))
            color = PATTERN_COLORS[pid]
            on = motor_on[m.index]
            pygame.draw.circle(self.screen, color if on or pid is PatternId.OFF else
                               _mix(color, BG, 0.6), p, 6)  # fmt: skip
            if pid is not PatternId.OFF:
                pygame.draw.circle(self.screen, color, p, 9 if on else 7, 1)
        self.text("top-down (wearer faces up)", (x + 6, y + 4), DIM, self.small)

    # ------------------------------------------------------------ middle

    def _draw_middle(self, x: int, y: int) -> None:
        color = self._source_color()
        head, _, detail = self.ultrasonic.title().partition(" (")
        self.text(head, (x, y), color, self.bold)
        if detail:
            self.text(detail.rstrip(")")[:60], (x, y + 17), color, self.small)
            y += 14
        y += 20
        st = self.ultrasonic.bridge_status()
        if st is not None:
            mode = st.firmware_mode
            mode_txt, mode_color = (
                {
                    FirmwareMode.HOST: ("HOST: Arduino follows host commands", REAL),
                    FirmwareMode.FALLBACK: ("FALLBACK: Arduino drives motors locally", WARN),
                    FirmwareMode.BOOT: ("BOOT: motors off, waiting for host", DIM),
                }.get(mode, ("unknown (no status)", BAD))
                if mode
                else ("unknown (no status)", BAD)
            )
            self.text(f"FIRMWARE {mode_txt}", (x, y), mode_color, self.small)
            y += 14
            cmd = "sending" if st.commands_enabled else "STOPPED (L)"
            ack = st.last_hap_seq_acked if st.last_hap_seq_acked is not None else "-"
            self.text(f"US {st.us_rate_hz:3.1f} Hz  cmds {cmd}  acked #{ack}  "
                      f"err {st.decode_errors}/{st.firmware_rx_errors or 0}",
                      (x, y), DIM, self.small)  # fmt: skip
            y += 16
            if st.state is not LinkState.CONNECTED:
                self.text(st.detail[:58], (x, y), BAD, self.small)
                y += 14
        for spec in ULTRASONIC_SENSORS:
            d = self.ultrasonic.display(spec.sensor_id)
            sel = "▶" if spec.sensor_id == self.selected_sensor else " "
            dcolor = color if d.status == "echo" else (BAD if d.status == "error" else DIM)
            self.text(f"{sel} {spec.sensor_id} {spec.label:<5}", (x, y), TEXT, self.font)
            self.text(d.text, (x + 120, y), dcolor, self.font)
            y += 18
        health = {h.sensor_id: h for h in (self.result.sensor_health if self.result else ())}
        degraded = [k for k, h in health.items() if h.status.is_degraded]
        if degraded:
            self.text(f"degraded: {', '.join(degraded)}", (x, y), BAD, self.small)
        y += 22
        target = (
            "commands sent to the Arduino"
            if self.ultrasonic.bridge_status() is not None
            else "virtual motors only"
        )
        self.text(f"HAPTIC OUTPUT ({target})", (x, y), (150, 160, 182), self.small)
        y += 16
        frame = self.result.frame if self.result else None
        if frame is None:
            return
        for ch in frame.channels:
            m = MOTORS[ch.motor]
            name = PATTERN_NAMES.get(ch.pattern_id, ch.pattern_id.name)
            if ch.advisory:
                name = "ADVISORY (exp.)"
            color = PATTERN_COLORS.get(ch.pattern_id, DIM) if ch.active else FAINT
            self.text(f"{m.name} {m.label:<11}", (x, y), DIM, self.small)
            self.text(name + ("*" if ch.held else ""), (x + 120, y), color, self.small)
            y += 15
        adv = "on (experimental)" if self.config.haptics.advisory_cue_enabled else "off"
        self.text(f"camera-only advisory cue: {adv}", (x, y + 4), DIM, self.small)

    # ------------------------------------------------------------ right

    def _draw_right(self, x: int, y: int, snap: VisionSnapshot) -> None:
        self.text("RISK ASSESSMENTS (shared pipeline)", (x, y), (150, 160, 182), self.small)
        y += 18
        report = self.result.report if self.result else None
        if report is None or not report.assessments:
            self.text("no observations", (x, y), FAINT, self.small)
            y += 16
        else:
            for a in report.top(9):
                color = SEVERITY_COLORS[a.severity] if a.is_actionable else FAINT
                src = {SourceType.ULTRASONIC: "US", SourceType.CAMERA: "CAM",
                       SourceType.FUSED: "FUS", SourceType.SIMULATION: "GT"}[a.source]  # fmt: skip
                rng = format_distance(a.effective_distance_cm)
                rng_src = (
                    "unavailable"
                    if a.effective_distance_cm is None
                    else "SIMULATED"
                    if self.ultrasonic.kind.simulated
                    else "REAL"
                )
                tag = a.severity.name if a.is_actionable else "info"
                label = a.observation.metadata.get("label", "")
                line = (
                    f"{tag:<8} {src:<3} {format_bearing(a.bearing_deg):>4} "
                    f"±{a.observation.bearing_uncertainty_deg:3.0f}  {rng:>8} ({rng_src})"
                )
                self.text(line, (x, y), color, self.small)
                detail = f"   {a.id} {label} conf {a.observation.confidence:.2f}"
                if a.is_stale:
                    detail += " STALE"
                self.text(detail[:80], (x, y + 13), FAINT, self.small)
                y += 29
        y = max(y + 6, 700)
        self.text(f"detector: {self.detector_label}", (x, y), DIM, self.small)
        y += 15
        if snap.worker_error:
            self.text(f"worker error: {snap.worker_error[:70]}", (x, y), BAD, self.small)
            y += 15
        rec = f"recording to {self.record_dir}" if self.record_dir else "not recording"
        paused = "   INFERENCE PAUSED" if self.vision.paused else ""
        self.text(rec + paused, (x, y), WARN if self.record_dir else FAINT, self.small)
        y += 15
        self.text("H: help   Tab: camera   1-4: sensor   C: calibrate", (x, y), FAINT, self.small)

    def _draw_help(self) -> None:
        shade = pygame.Surface((W, H), pygame.SRCALPHA)
        shade.fill((8, 10, 14, 220))
        self.screen.blit(shade, (0, 0))
        y = 260
        self.text("Controls", (W // 2, y), TEXT, self.title, "center")
        for line in HELP:
            y += 26
            self.text(line, (W // 2, y), TEXT, self.font, "center")
        y += 40
        self.text("Magenta = simulated ultrasonic data. Green = real Arduino measurements.",
                  (W // 2, y), SIM, self.font, "center")  # fmt: skip
        self.text("Camera range is never estimated.", (W // 2, y + 20), DIM, self.font, "center")


def camera_source_label(device: object) -> str:
    """REAL CAMERA / SYNTHETIC / VIDEO FILE, derived from the configured device."""
    if isinstance(device, int) or (isinstance(device, str) and device.isdigit()):
        return "REAL CAMERA"
    if device == "synthetic":
        return "SYNTHETIC"
    return "VIDEO FILE"


def _wrap(s: str, n: int) -> list[str]:
    words, lines, cur = s.split(), [], ""
    for w in words:
        if len(cur) + len(w) + 1 > n:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    if cur:
        lines.append(cur)
    return lines
