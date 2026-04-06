"""
dummy.py — RAKSHAQ test harness (no hardware required).

Replaces STM32 motor control with IJKL keyboard pan/tilt.
All other subsystems (YOLO, depth UI, stream, sensor HUD) behave normally.
Hardware sensors fall back to mock/0mm automatically.

Controls
--------
  I        — tilt up         (Y+)
  K        — tilt down       (Y-)
  J        — pan left        (X-)
  L        — pan right       (X+)
  SPACE    — stop / clear step
  r        — reset pan to centre
  d        — toggle depth trigger
  1/2/3/0  — mode: STANDBY / MONITOR / ENGAGE / ABORT
  q / Esc  — quit

Run:
    uv run python dummy.py          (Windows — shows cv2 window)
    uv run python dummy.py --stream (Pi headless — stream only)
"""

from __future__ import annotations
import cv2, time, argparse, sys, config

# ── cv2 headless stub ─────────────────────────────────────────────────────────
for _fn in ("imshow", "waitKey", "destroyAllWindows", "namedWindow", "resizeWindow"):
    if not hasattr(cv2, _fn):
        setattr(cv2, _fn, lambda *a, **k: None if "wait" not in str(a) else -1)

# ── now safe to import ultralytics-backed modules ─────────────────────────────
from src.turret.detection.camera   import get_camera
from src.turret.detection.detector import YOLODetector
from src.turret.detection.depth    import DepthEstimator
from src.turret.hardware.sensors   import TOFSensor, UltrasonicSensor, LaserController
from src.turret.utils.visualizer   import Visualizer
from src.turret.utils.streamer     import FrameStreamer
from src.turret.ui.depth_control   import DepthControlUI
from src.turret.action             import ActionMode, ActionController, TargetSelector

# ── IJKL step size per key-press ─────────────────────────────────────────────
STEP = 100   # steps per key-press  (~11°)

FONT = cv2.FONT_HERSHEY_SIMPLEX


class MockTurret:
    """Prints IJKL motor commands instead of sending serial."""
    def __init__(self):
        self._pan_steps  = 0
        self._tilt_steps = 0

    def pan(self, steps: int):
        self._pan_steps += steps
        print(f"[MOCK] X{steps:+d}  (total pan {self._pan_steps:+d})")

    def tilt(self, steps: int):
        self._tilt_steps += steps
        print(f"[MOCK] Y{steps:+d}  (total tilt {self._tilt_steps:+d})")

    def reset_pan(self):
        print(f"[MOCK] X{-self._pan_steps:+d}  (pan → home)")
        self._pan_steps = 0

    def get_position_deg(self):
        from src.turret.hardware.stm32 import STEPS_PER_DEGREE
        return (self._pan_steps / STEPS_PER_DEGREE,
                self._tilt_steps / STEPS_PER_DEGREE)


class DummySystem:
    def __init__(self, headless: bool = False):
        self._headless = headless
        self.frame_count = 0
        self.running = False

        # Camera
        print("[DUMMY] Opening camera...")
        self.camera = get_camera(
            source=config.CAMERA_SOURCE,
            width=config.FRAME_WIDTH,
            height=config.FRAME_HEIGHT,
            threaded=config.CAMERA_THREADED,
            rotate=config.CAMERA_ROTATE,
        )

        # Detector
        print("[DUMMY] Loading YOLO model...")
        self.detector = YOLODetector(
            model_path=config.MODEL_PATH,
            confidence=config.CONFIDENCE,
            iou_thresh=config.IOU_THRESH,
            imgsz=config.IMGSZ,
            track_classes=config.TRACK_CLASSES,
        )

        # Mock turret
        self.turret = MockTurret()

        # Sensors (all degrade gracefully to 0mm if not on Pi)
        self.tof   = TOFSensor(port=config.TOF_PORT, baud=config.TOF_BAUD)
        self.sonar = UltrasonicSensor(
            echo_pin=config.SONAR_ECHO_PIN,
            trigger_pin=config.SONAR_TRIGGER_PIN,
            max_distance_m=config.SONAR_MAX_DISTANCE_M,
        )
        self.laser = LaserController(pin=config.LASER_GPIO_PIN)
        self.tof.start();  self.sonar.start();  self.laser.start()

        # Depth
        self.depth_est = DepthEstimator(
            backend=config.DEPTH_BACKEND,
            tof_sensor=self.tof   if config.DEPTH_BACKEND == "tof"        else None,
            sonar_sensor=self.sonar if config.DEPTH_BACKEND == "ultrasonic" else None,
            max_mm=config.TOF_MAX_MM,
        )
        self._depth_enabled = config.DEPTH_ENABLED
        self._last_depth = 0.0

        # Depth UI
        self.depth_ui: DepthControlUI | None = None
        if config.SHOW_DEPTH_UI and not headless:
            ui_mode = "mm" if self.depth_est.uses_mm else "pct"
            ui_max  = config.TOF_MAX_MM if ui_mode == "mm" else 100
            self.depth_ui = DepthControlUI(
                initial_threshold=config.DEPTH_THRESHOLD,
                initial_enabled=self._depth_enabled,
                mode=ui_mode, max_value=ui_max,
            )
            self.depth_ui.open()

        # Action
        self.ac = ActionController(ActionMode.MONITOR)
        self.ts = TargetSelector(strategy=config.TARGETING_STRATEGY,
                                 min_confidence=config.CONFIDENCE)
        self.ts.update_frame_size(config.FRAME_WIDTH, config.FRAME_HEIGHT)

        # Visualizer + streamer
        self.viz      = Visualizer(show_fps=config.SHOW_FPS)
        self.streamer = FrameStreamer(port=config.STREAM_PORT,
                                     quality=config.STREAM_QUALITY)
        if config.ENABLE_STREAM:
            self.streamer.start()

        print(f"\n[DUMMY] Ready — {'headless, view: http://<ip>:5000/' if headless else 'press IJKL to move'}")
        print("  I=up  K=down  J=left  L=right  r=reset  d=depth  q=quit")

    # ──────────────────────────────────────────────────────────────────────────
    def process_frame(self, frame):
        if self.frame_count % max(config.DETECTION_SKIP_FRAMES, 1) == 0:
            self._last_dets = self.detector.detect(frame)
        dets   = getattr(self, "_last_dets", [])
        target = self.ts.select_target(dets)

        depth = 0.0
        if self._depth_enabled:
            depth = self.depth_est.estimate(target, config.FRAME_HEIGHT)
            self._last_depth = depth
        if self.depth_ui:
            self.depth_ui.update(depth)

        # Auto-track with mock turret
        if target and self.ac.should_track():
            from src.turret.hardware.stm32 import STEPS_PER_DEGREE
            cx, cy = target.center
            err_x  = cx - config.FRAME_WIDTH  // 2
            err_y  = cy - config.FRAME_HEIGHT // 2
            p = config.TRACKING_P_GAIN
            sx = int((err_x  / config.FRAME_WIDTH)  * config.CAMERA_HFOV_DEG * STEPS_PER_DEGREE * p)
            sy = int((-err_y / config.FRAME_HEIGHT) * config.CAMERA_VFOV_DEG * STEPS_PER_DEGREE * p)
            if abs(sx) > 4: self.turret.pan(sx)
            if abs(sy) > 4: self.turret.tilt(sy)

        annotated = self.viz.draw_detections(frame, dets)
        annotated = self._overlay(annotated, target, depth)
        self.streamer.update(annotated)
        return annotated

    def _overlay(self, frame, target, depth):
        h, w = frame.shape[:2]

        # Mode
        mode = self.ac.current_mode
        col  = {ActionMode.STANDBY:(128,128,128), ActionMode.MONITOR:(0,255,255),
                ActionMode.ENGAGE:(0,0,255),       ActionMode.ABORT:(255,0,0)}.get(mode, (255,255,255))
        cv2.putText(frame, f"MODE: {mode.value.upper()}", (10,30), FONT, 0.8, col, 2)

        # IJKL hint (bottom)
        cv2.putText(frame, "I=Up  K=Down  J=Left  L=Right",
                    (10, h-10), FONT, 0.45, (200,200,200), 1, cv2.LINE_AA)

        # Target reticle
        if target:
            cx, cy = target.center
            cv2.drawMarker(frame, (cx, cy), (0,255,255), cv2.MARKER_CROSS, 30, 2)
            cv2.circle(frame, (cx, cy), 40, (0,255,255), 2)

        # Sensor HUD
        tof_mm   = self.tof.distance_mm
        sonar_mm = self.sonar.distance_mm
        cv2.rectangle(frame, (w-200, 5), (w-5, 85), (0,0,0), -1)
        cv2.putText(frame, f"LASER: {tof_mm}mm",   (w-190,30), FONT, 0.65, (0,255,255), 2)
        cv2.putText(frame, f"SONAR: {sonar_mm}mm", (w-190,58), FONT, 0.65, (255,255,0), 2)
        laser_txt = "WEAPON: FIRING" if self.laser.is_active else "WEAPON: SAFE"
        laser_col = (0,0,255) if self.laser.is_active else (0,255,0)
        cv2.putText(frame, laser_txt, (w-190,83), FONT, 0.55, laser_col, 2)

        # Position
        pan, tilt = self.turret.get_position_deg()
        cv2.putText(frame, f"Pan: {pan:.0f}°  Tilt: {tilt:.0f}°",
                    (10, h-30), FONT, 0.6, (255,255,255), 2)
        return frame

    # ──────────────────────────────────────────────────────────────────────────
    def handle_key(self, key: int) -> bool:
        if key in (ord("q"), 27):          return False
        elif key == ord("i"):              self.turret.tilt(STEP)
        elif key == ord("k"):              self.turret.tilt(-STEP)
        elif key == ord("j"):              self.turret.pan(-STEP)
        elif key == ord("l"):              self.turret.pan(STEP)
        elif key == ord("r"):              self.turret.reset_pan()
        elif key == ord("d"):
            self._depth_enabled = not self._depth_enabled
            if self.depth_ui:
                self.depth_ui.set_enabled(self._depth_enabled)
        elif key == ord("1"):              self.ac.set_mode(ActionMode.STANDBY)
        elif key == ord("2"):              self.ac.set_mode(ActionMode.MONITOR)
        elif key == ord("3"):              self.ac.set_mode(ActionMode.ENGAGE)
        elif key == ord("0"):              self.ac.set_mode(ActionMode.ABORT)
        return True

    def run(self):
        self.running = True
        try:
            while self.running:
                frame = self.camera.read()
                if frame is None: break

                annotated = self.process_frame(frame)
                self.frame_count += 1

                if not self._headless:
                    cv2.imshow("RAKSHAQ DUMMY", annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if not self.handle_key(key):
                        break
        except KeyboardInterrupt:
            print("\n[DUMMY] Interrupted")
        finally:
            if self.depth_ui: self.depth_ui.close()
            self.tof.stop(); self.sonar.stop(); self.laser.stop()
            self.camera.release()
            try: cv2.destroyAllWindows()
            except Exception: pass
            print(f"[DUMMY] Done — {self.frame_count} frames")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--stream", action="store_true",
                    help="Headless mode (no cv2 window, stream only)")
    args = ap.parse_args()
    DummySystem(headless=args.stream).run()
