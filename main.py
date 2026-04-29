"""
RAKSHAQ — Autonomous AI Turret System
Main entry point — wires detection, hardware control, and action states.
"""

import cv2
import time

# Headless OpenCV builds (Pi) omit cv2.imshow; ultralytics patches it at
# import time — stub missing functions before any ultralytics import.
for _fn in ("imshow", "waitKey", "destroyAllWindows", "namedWindow"):
    if not hasattr(cv2, _fn):
        setattr(cv2, _fn, lambda *a, **k: None)

import config
from src.turret.detection.camera   import get_camera
from src.turret.detection.detector import YOLODetector
from src.turret.detection.depth    import DepthEstimator
from src.turret.hardware.sensors   import TOFSensor, UltrasonicSensor, LaserController
from src.turret.utils.visualizer   import Visualizer
from src.turret.utils.streamer     import FrameStreamer
from src.turret.ui.depth_control   import DepthControlUI
from src.turret.action             import ActionController, ActionMode, TargetSelector
from src.turret.hardware.stm32     import STM32Controller
from src.turret.tracking           import Tracker, ParallaxCompensator, AxisMapper


class RakshaqSystem:
    """Main RAKSHAQ turret system."""

    def __init__(self):
        print("=" * 60)
        print("RAKSHAQ — Autonomous AI Turret System")
        print("=" * 60)

        # ── Phase 1: Detection ────────────────────────────────────────────────
        print("\n[INIT] Phase 1: Detection")
        print("-" * 60)

        print("[CAMERA] Setting up camera...")
        self.camera = get_camera(
            source=config.CAMERA_SOURCE,
            width=config.FRAME_WIDTH,
            height=config.FRAME_HEIGHT,
            threaded=config.CAMERA_THREADED,
            rotate=config.CAMERA_ROTATE,
        )

        print("[DETECTOR] Loading YOLO model...")
        self.detector = YOLODetector(
            model_path=config.MODEL_PATH,
            confidence=config.CONFIDENCE,
            iou_thresh=config.IOU_THRESH,
            imgsz=config.IMGSZ,
            track_classes=config.TRACK_CLASSES,
            tracking_enabled=getattr(config, "TRACKING_ENABLED", True),
            tracker_type=getattr(config, "TRACKER_TYPE", "bytetrack"),
        )

        self.visualizer = Visualizer(show_fps=config.SHOW_FPS)

        # ── Phase 2: Hardware ─────────────────────────────────────────────────
        print("\n[INIT] Phase 2: Hardware")
        print("-" * 60)

        stm32 = STM32Controller(port=config.STM32_PORT, baud=config.STM32_BAUD)
        axis_mapper = AxisMapper(
            hfov_deg=config.CAMERA_HFOV_DEG,
            vfov_deg=config.CAMERA_VFOV_DEG,
            camera_rotate=config.CAMERA_ROTATE,
            pan_invert=config.PAN_INVERT,
            tilt_invert=config.TILT_INVERT,
        )
        parallax = ParallaxCompensator(
            barrel_right_cm=config.BARREL_RIGHT_CM,
            barrel_up_cm=config.BARREL_UP_CM,
            barrel_forward_cm=config.BARREL_FORWARD_CM,
        )

        self.tracker = Tracker(
            stm32=stm32,
            axis_mapper=axis_mapper,
            parallax=parallax,
            kp=config.TRACK_KP,
            ki=config.TRACK_KI,
            kd=config.TRACK_KD,
            i_max=config.TRACK_I_MAX,
            deadzone_px=config.TRACK_DEADZONE_PX,
            max_steps_per_frame=config.MOTOR_MAX_STEPS_PER_FRAME,
            chase_timeout_s=config.CHASE_TIMEOUT_S,
            search_timeout_s=config.SEARCH_TIMEOUT_S,
            predict_window=config.CHASE_PREDICT_FRAMES,
        )

        self.tof   = TOFSensor(port=config.TOF_PORT, baud=config.TOF_BAUD)
        self.sonar = UltrasonicSensor(
            echo_pin=config.SONAR_ECHO_PIN,
            trigger_pin=config.SONAR_TRIGGER_PIN,
            max_distance_m=config.SONAR_MAX_DISTANCE_M,
        )
        self.tof.start()
        self.sonar.start()

        self.laser = LaserController(pin=config.LASER_GPIO_PIN)
        self.laser.start()

        # ── Phase 2b: Depth ───────────────────────────────────────────────────
        print("\n[INIT] Phase 2b: Depth")
        print("-" * 60)

        # Inject real sensor objects when using tof/ultrasonic backends
        self.depth_estimator = DepthEstimator(
            backend      = config.DEPTH_BACKEND,
            tof_sensor   = self.tof   if config.DEPTH_BACKEND == "tof"        else None,
            sonar_sensor = self.sonar if config.DEPTH_BACKEND == "ultrasonic" else None,
            max_mm       = config.TOF_MAX_MM,
        )

        self._depth_enabled  = config.DEPTH_ENABLED
        self._engage_counter = 0
        self._retreat_counter = 0
        self._last_depth     = 0.0

        # Depth UI — adapt display mode and range to backend
        self.depth_ui: DepthControlUI | None = None
        if config.SHOW_DEPTH_UI:
            ui_mode = "mm" if self.depth_estimator.uses_mm else "pct"
            ui_max  = config.TOF_MAX_MM if ui_mode == "mm" else 100
            self.depth_ui = DepthControlUI(
                initial_threshold = config.DEPTH_THRESHOLD,
                initial_enabled   = self._depth_enabled,
                mode              = ui_mode,
                max_value         = ui_max,
            )
            self.depth_ui.open()

        # ── Phase 3: Action ───────────────────────────────────────────────────
        print("\n[INIT] Phase 3: Action Layer")
        print("-" * 60)

        mode_map = {
            "standby": ActionMode.STANDBY, "monitor": ActionMode.MONITOR,
            "engage":  ActionMode.ENGAGE,  "abort":   ActionMode.ABORT,
        }
        initial_mode = mode_map.get(config.DEFAULT_MODE, ActionMode.MONITOR)
        self.action_controller = ActionController(initial_mode)

        self.target_selector = TargetSelector(
            min_confidence=config.CONFIDENCE,
            switch_hysteresis_px=config.TARGET_SWITCH_HYSTERESIS,
        )
        self.target_selector.update_frame_size(config.FRAME_WIDTH, config.FRAME_HEIGHT)

        # ── Phase 4: Streaming ────────────────────────────────────────────────
        self.streamer = FrameStreamer(port=config.STREAM_PORT, quality=config.STREAM_QUALITY)
        if config.ENABLE_STREAM:
            self.streamer.start()

        # General state
        self.running        = False
        self.current_target = None
        self.frame_count    = 0
        self._fps           = 0.0
        self._fps_ts        = time.monotonic()
        self._fps_counter   = 0
        self._engagements_log: list[str] = []   # kept for dashboard

        print("\n[INIT] ✅ System ready!")
        print(f"[MODE]  {self.action_controller.current_mode.value.upper()}")
        print(f"[DEPTH] backend={config.DEPTH_BACKEND}  threshold={config.DEPTH_THRESHOLD}"
              f"{'mm' if self.depth_estimator.uses_mm else '%'}")
        if config.ENABLE_STREAM:
            print(f"[STREAM] http://<pi-ip>:{config.STREAM_PORT}/")
        self._print_controls()

    # ──────────────────────────────────────────────────────────────────────────
    def _print_controls(self):
        print()
        print("=" * 60)
        print("KEYBOARD CONTROLS")
        print("=" * 60)
        print("  q / Esc  — Quit")
        print("  r        — Reset turret to home")
        print("  s        — Manual scan")
        print("  d        — Toggle depth trigger ON/OFF")
        print()
        print("  1  — STANDBY   2  — MONITOR   3  — ENGAGE   0  — ABORT")
        print("=" * 60)

    # ──────────────────────────────────────────────────────────────────────────
    def process_frame(self, frame):
        """Run full detection + action pipeline on one frame."""

        # Detection — only run YOLO every DETECTION_SKIP_FRAMES frames
        if self.frame_count % max(config.DETECTION_SKIP_FRAMES, 1) == 0:
            self._last_detections = self.detector.detect(frame)
        detections = getattr(self, "_last_detections", [])

        # Target selection
        target = self.target_selector.select_target(detections)
        self.current_target = target

        # Depth
        depth = 0.0
        if self._depth_enabled:
            try:
                depth = self.depth_estimator.estimate(target, config.FRAME_HEIGHT)
                if target or self.depth_estimator.uses_mm:
                    # mm backends: always read (sensor is independent of detections)
                    # pct backend: only meaningful when there's a target
                    self._update_depth_trigger(depth)
            except Exception as exc:
                print(f"[DEPTH] Error (disabling): {exc}")
                self._depth_enabled = False
                if self.depth_ui:
                    self.depth_ui.set_enabled(False)
        else:
            self._engage_counter  = 0
            self._retreat_counter = 0

        self._last_depth = depth
        if self.depth_ui:
            self.depth_ui.update(depth)

        # Tracking / engagement
        tracking_state = self.tracker.update(
            target=target,
            detections=detections,
            frame_w=config.FRAME_WIDTH,
            frame_h=config.FRAME_HEIGHT,
            tof_mm=self.tof.distance_mm,
            allow_movement=self.action_controller.config.allow_movement,
        )

        if tracking_state.name == "TRACKING" and target:
            # We are on-target, consider engaging
            # The old TurretController checked `is_centered(target.center)`.
            # Tracker handles the PID positioning, so we assume if we are the TRACKING state,
            # we are successfully aiming at the target.
            if self.action_controller.should_engage(target_centered=True):
                 self.action_controller.record_engagement(
                    f"{target.class_name} (conf: {target.confidence:.0%})"
                )

        # Visualise
        annotated = self.visualizer.draw_detections(frame, detections, selected_target=target)
        annotated = self._add_overlays(annotated, target, depth)

        # ── FPS counter ───────────────────────────────────────────────────────
        self._fps_counter += 1
        now = time.monotonic()
        if now - self._fps_ts >= 1.0:
            self._fps      = self._fps_counter / (now - self._fps_ts)
            self._fps_counter = 0
            self._fps_ts   = now

        # ── Stream frame ──────────────────────────────────────────────────────
        self.streamer.update(annotated)

        # ── Push telemetry to dashboard ───────────────────────────────────────
        pan_deg, tilt_deg = self.tracker.get_position()
        self.streamer.set_status({
            "mode":          self.action_controller.current_mode.value.upper(),
            "track_state":   tracking_state.name,
            "tof_mm":        self.tof.distance_mm,
            "sonar_mm":      self.sonar.distance_mm,
            "pan_deg":       round(pan_deg, 1),
            "tilt_deg":      round(tilt_deg, 1),
            "fps":           round(self._fps, 1),
            "laser_active":  self.laser.is_active,
            "depth_enabled": self._depth_enabled,
            "depth_mm":      round(self._last_depth) if self.depth_estimator.uses_mm else 0,
            "depth_thresh":  self.depth_ui.threshold if self.depth_ui else config.DEPTH_THRESHOLD,
            "stm32_ok":      self.tracker._stm32.connected,
            "target_label":  target.class_name if target else None,
            "target_conf":   round(target.confidence * 100) if target else 0,
            "track_id":      target.track_id  if target else None,
            "engagements":   self._engagements_log[-20:],
        })

        return annotated

    # ──────────────────────────────────────────────────────────────────────────
    def _update_depth_trigger(self, depth) -> None:
        """Hysteresis depth → MONITOR ↔ ENGAGE transitions."""
        threshold    = self.depth_ui.threshold if self.depth_ui else config.DEPTH_THRESHOLD
        current_mode = self.action_controller.current_mode
        uses_mm      = self.depth_estimator.uses_mm

        # Condition for "target within range"
        if uses_mm:
            in_range = (0 < depth <= threshold)       # closer = smaller mm value
        else:
            in_range = (depth >= threshold)            # closer = larger % value

        if in_range:
            self._engage_counter  += 1
            self._retreat_counter  = 0
            if (self._engage_counter >= config.DEPTH_ENGAGE_FRAMES
                    and current_mode == ActionMode.MONITOR):
                unit = "mm" if uses_mm else "%"
                print(f"\n[DEPTH] 🚨 Target in range ({depth:.0f}{unit} "
                      f"{'<=' if uses_mm else '>='} {threshold}{unit}) → ENGAGE")
                self.change_mode(ActionMode.ENGAGE)
                self._engage_counter = 0
            # Fire laser only when TOF is the active backend
            if config.DEPTH_BACKEND == "tof":
                self.laser.on()
        else:
            self._retreat_counter += 1
            self._engage_counter   = 0
            # TOF backend: safe the laser as soon as sensor reading retreats
            if config.DEPTH_BACKEND == "tof":
                self.laser.off()
            if (self._retreat_counter >= config.DEPTH_RETREAT_FRAMES
                    and current_mode == ActionMode.ENGAGE):
                print(f"\n[DEPTH] Target retreated → MONITOR")
                self.change_mode(ActionMode.MONITOR)
                self._retreat_counter = 0

    # ──────────────────────────────────────────────────────────────────────────
    def _add_overlays(self, frame, target, depth):
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Mode indicator
        if config.SHOW_MODE_INDICATOR:
            mode = self.action_controller.current_mode
            mode_colors = {
                ActionMode.STANDBY: config.COLOR_MODE_STANDBY,
                ActionMode.MONITOR: config.COLOR_MODE_MONITOR,
                ActionMode.ENGAGE:  config.COLOR_MODE_ENGAGE,
                ActionMode.ABORT:   config.COLOR_MODE_ABORT,
            }
            color = mode_colors.get(mode, config.COLOR_TEXT)
            cv2.putText(frame, f"MODE: {mode.value.upper()}",
                        (10, 30), font, 0.8, color, 2)

        # Target reticle + depth bar
        if target and config.SHOW_TARGET_RETICLE:
            cx, cy = target.center
            cv2.drawMarker(frame, (cx, cy), config.COLOR_RETICLE,
                           cv2.MARKER_CROSS, 30, 2)
            cv2.circle(frame, (cx, cy), 40, config.COLOR_RETICLE, 2)

            if self._depth_enabled:
                threshold = self.depth_ui.threshold if self.depth_ui else config.DEPTH_THRESHOLD
                max_val   = config.TOF_MAX_MM if self.depth_estimator.uses_mm else 100
                self._draw_depth_bar(frame, target, depth, threshold, max_val)

        # Sensor HUD (top-right) — TOF + sonar
        if config.SHOW_SENSOR_HUD:
            tof_mm   = self.tof.distance_mm
            sonar_mm = self.sonar.distance_mm
            cv2.rectangle(frame, (w - 200, 5), (w - 5, 85), (0, 0, 0), -1)
            cv2.putText(frame, f"LASER: {tof_mm}mm",
                        (w - 190, 30), font, 0.65, (0, 255, 255), 2)
            cv2.putText(frame, f"SONAR: {sonar_mm}mm",
                        (w - 190, 58), font, 0.65, (255, 255, 0), 2)
            if self.laser.is_active:
                cv2.putText(frame, "WEAPON: FIRING",
                            (w - 190, 83), font, 0.55, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "WEAPON: SAFE",
                            (w - 190, 83), font, 0.55, (0, 255, 0), 2)

        # Pan / tilt position (bottom left)
        if config.SHOW_POSITION_INFO:
            pan, tilt = self.tracker.get_position()
            cv2.putText(frame, f"Pan: {pan:.0f}°  Tilt: {tilt:.0f}°",
                        (10, h - 20), font, 0.6, config.COLOR_TEXT, 2)

        # Depth toggle status (bottom right)
        dtxt = "DEPTH: ON" if self._depth_enabled else "DEPTH: OFF"
        dcol = (0, 200, 80) if self._depth_enabled else (60, 60, 60)
        (tw, _), _ = cv2.getTextSize(dtxt, font, 0.5, 1)
        cv2.putText(frame, dtxt, (w - tw - 10, h - 20),
                    font, 0.5, dcol, 1, cv2.LINE_AA)

        return frame

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _draw_depth_bar(frame, target, depth, threshold, max_val: float = 100) -> None:
        """Draw a compact distance/proximity bar beneath the target bbox."""
        bx0, bx1 = target.x1, target.x2
        by0 = target.y2 + 4
        by1 = by0 + 6
        bw  = max(bx1 - bx0, 1)

        cv2.rectangle(frame, (bx0, by0), (bx1, by1), (50, 50, 50), -1)

        if depth > 0 and max_val > 0:
            fill_ratio = min(depth / max_val, 1.0)
            fill_w     = int(bw * fill_ratio)
            # mm mode: engage when depth <= threshold (red when close)
            # pct mode: engage when depth >= threshold (red when bbox fills frame)
            triggered = (depth <= threshold) if max_val > 100 else (depth >= threshold)
            col = (0, 80, 255) if triggered else (0, 200, 100)
            if fill_w > 0:
                cv2.rectangle(frame, (bx0, by0), (bx0 + fill_w, by1), col, -1)

        tick_x = bx0 + int(bw * min(threshold / max(max_val, 1), 1.0))
        cv2.line(frame, (tick_x, by0 - 2), (tick_x, by1 + 2), (0, 220, 255), 1)

        unit = "mm" if max_val > 100 else "%"
        cv2.putText(frame, f"{depth:.0f}{unit}",
                    (bx0, by0 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (200, 200, 200), 1, cv2.LINE_AA)

    # ──────────────────────────────────────────────────────────────────────────
    def change_mode(self, mode: ActionMode):
        if self.action_controller.set_mode(mode):
            print(f"\n{'='*60}\nMODE: {mode.value.upper()}\n{'='*60}\n")
            if mode == ActionMode.ABORT:
                self.tracker.emergency_stop()
            elif mode == ActionMode.STANDBY:
                self.tracker.reset()

    def handle_keyboard(self, key: int) -> bool:
        if key in (ord("q"), 27):
            return False
        elif key == ord("r"):
            self.tracker.reset()
        elif key == ord("s"):
            self.tracker.scan_area()
        elif key == ord("1"):
            self.change_mode(ActionMode.STANDBY)
        elif key == ord("2"):
            self.change_mode(ActionMode.MONITOR)
        elif key == ord("3"):
            self.change_mode(ActionMode.ENGAGE)
        elif key == ord("0"):
            self.change_mode(ActionMode.ABORT)
        elif key == ord("d"):
            self._depth_enabled = not self._depth_enabled
            print(f"\n[DEPTH] {'ON' if self._depth_enabled else 'OFF'}")
            if self.depth_ui:
                self.depth_ui.set_enabled(self._depth_enabled)
            if not self._depth_enabled:
                self._engage_counter  = 0
                self._retreat_counter = 0
                if self.action_controller.current_mode == ActionMode.ENGAGE:
                    self.change_mode(ActionMode.MONITOR)
        return True

    # ──────────────────────────────────────────────────────────────────────────
    def run(self):
        print("[SYSTEM] Starting main loop...\n")
        self.running = True
        self._headless = self._detect_headless()

        if self._headless:
            print("[SYSTEM] Headless mode — no display detected.")
            print("[SYSTEM] View stream at http://<pi-ip>:5000/")
            print("[SYSTEM] Press Ctrl+C to quit.\n")
            self._start_stdin_keyboard()

        try:
            while self.running:
                frame = self.camera.read()
                if frame is None:
                    print("[ERROR] No frame from camera")
                    break

                annotated = self.process_frame(frame)

                if not self._headless:
                    cv2.imshow("RAKSHAQ", annotated)
                    key = cv2.waitKey(1) & 0xFF
                    if not self.handle_keyboard(key):
                        break

                self.frame_count += 1

        except KeyboardInterrupt:
            print("\n[SYSTEM] Interrupted by user")
        except Exception as exc:
            print(f"\n[ERROR] System error: {exc}")
            import traceback; traceback.print_exc()
            self.tracker.emergency_stop()
        finally:
            self.cleanup()

    def cleanup(self):
        print("\n" + "=" * 60)
        print("SYSTEM SHUTDOWN")
        print("=" * 60)
        self.running = False
        if self.depth_ui:
            self.depth_ui.close()
        print("[CLEANUP] Stopping sensors...")
        self.tof.stop()
        self.sonar.stop()
        self.laser.stop()   # always safe the laser first
        print("[CLEANUP] Stopping camera...")
        self.camera.release()
        print("[CLEANUP] Resetting turret...")
        self.tracker.cleanup()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        print(f"\n[STATS] Frames: {self.frame_count}")
        print(f"[STATS] Final mode: {self.action_controller.current_mode.value.upper()}")
        print("\n✅ Shutdown complete\n" + "=" * 60)


def main():
    RakshaqSystem().run()


# ──────────────────────────────────────────────────────────────────────────────

def _static_detect_headless() -> bool:
    """Try opening an OpenCV window; return True if display is unavailable."""
    try:
        cv2.namedWindow("__hd_test__", cv2.WINDOW_NORMAL)
        cv2.destroyWindow("__hd_test__")
        return False
    except Exception:
        return True


RakshaqSystem._detect_headless = staticmethod(_static_detect_headless)


def _start_stdin_keyboard(self):
    """Background thread reading single-char commands from stdin when headless."""
    import threading
    import sys

    def _reader():
        print("[KEYBOARD] Headless controls: r=reset  s=scan  d=depth  q=quit")
        try:
            for line in sys.stdin:
                key_char = line.strip().lower()[:1]
                if not key_char:
                    continue
                if key_char == "q":
                    self.running = False
                    break
                self.handle_keyboard(ord(key_char))
        except Exception:
            pass

    t = threading.Thread(target=_reader, daemon=True, name="StdinKeyboard")
    t.start()


RakshaqSystem._start_stdin_keyboard = _start_stdin_keyboard


if __name__ == "__main__":
    main()