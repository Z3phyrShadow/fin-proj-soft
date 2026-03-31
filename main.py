"""
RAKSHAQ - Autonomous AI Turret System
Main entry point with integrated Phase 1 (Detection) and Phase 2 (Action Layer)
"""

import cv2
import time

import config
from src.turret.detection.camera   import get_camera
from src.turret.detection.detector import YOLODetector
from src.turret.detection.depth    import DepthEstimator
from src.turret.utils.visualizer   import Visualizer
from src.turret.ui.depth_control   import DepthControlUI

# Action layer imports (Phase 2)
from src.turret.action import ActionController, ActionMode, TurretController, TargetSelector


class RakshaqSystem:
    """Main RAKSHAQ turret system"""

    def __init__(self):
        """Initialize all system components"""
        print("=" * 60)
        print("RAKSHAQ - Autonomous AI Turret System")
        print("=" * 60)
        print()

        # Phase 1: Detection
        print("[INIT] Phase 1: Detection System")
        print("-" * 60)

        print("[CAMERA] Setting up camera...")
        self.camera = get_camera(
            source=config.CAMERA_SOURCE,
            width=config.FRAME_WIDTH,
            height=config.FRAME_HEIGHT
        )

        print("[DETECTOR] Loading YOLO model...")
        self.detector = YOLODetector(
            model_path=config.MODEL_PATH,
            confidence=config.CONFIDENCE,
            iou_thresh=config.IOU_THRESH,
            track_classes=config.TRACK_CLASSES
        )

        self.visualizer = Visualizer(show_fps=config.SHOW_FPS)

        # Phase 2: Action Layer
        print()
        print("[INIT] Phase 2: Action Layer")
        print("-" * 60)

        mode_map = {
            "standby": ActionMode.STANDBY,
            "monitor": ActionMode.MONITOR,
            "engage":  ActionMode.ENGAGE,
            "abort":   ActionMode.ABORT
        }
        initial_mode = mode_map.get(config.DEFAULT_MODE, ActionMode.MONITOR)
        self.action_controller = ActionController(initial_mode)

        self.turret = TurretController(
            pan_pin=config.PAN_SERVO_PIN,
            tilt_pin=config.TILT_SERVO_PIN,
            servo_frequency=config.SERVO_FREQUENCY,
            smoothing_factor=config.SMOOTHING_FACTOR
        )

        self.target_selector = TargetSelector(
            strategy=config.TARGETING_STRATEGY,
            min_confidence=config.CONFIDENCE
        )
        self.target_selector.update_frame_size(config.FRAME_WIDTH, config.FRAME_HEIGHT)

        # Depth system
        print()
        print("[INIT] Phase 2b: Depth System")
        print("-" * 60)

        self.depth_estimator = DepthEstimator(
            backend=config.DEPTH_BACKEND,
            ir_max_cm=config.DEPTH_IR_MAX_CM
        )

        self.depth_ui: DepthControlUI | None = None
        if config.SHOW_DEPTH_UI:
            self.depth_ui = DepthControlUI(initial_threshold=config.DEPTH_THRESHOLD)
            self.depth_ui.open()

        # Hysteresis counters
        self._engage_counter  = 0
        self._retreat_counter = 0
        self._last_depth      = 0.0

        # General state
        self.running       = False
        self.current_target = None
        self.last_scan_time = 0
        self.frame_count    = 0

        print()
        print("[INIT] ✅ System ready!")
        print(f"[MODE] Current mode: {self.action_controller.current_mode.value.upper()}")
        print(f"[DEPTH] Backend: {self.depth_estimator.backend}  |  "
              f"Threshold: {config.DEPTH_THRESHOLD}%")
        self._print_controls()

    # ──────────────────────────────────────────────────────────────────────────
    def _print_controls(self):
        """Print keyboard controls"""
        print()
        print("=" * 60)
        print("KEYBOARD CONTROLS")
        print("=" * 60)
        print("  q / Esc  - Quit system")
        print("  r        - Reset turret to center")
        print("  s        - Manual scan")
        print()
        print("  1        - STANDBY mode  (no movement)")
        print("  2        - MONITOR mode  (track + alert)")
        print("  3        - ENGAGE mode   (track + engage)")
        print("  0        - ABORT mode    (emergency stop)")
        print("=" * 60)
        print()

    # ──────────────────────────────────────────────────────────────────────────
    def process_frame(self, frame):
        """
        Process frame through detection and action pipeline.

        Returns
        -------
        numpy.ndarray
            Annotated frame for display.
        """
        # PHASE 1 — Detection
        detections = self.detector.detect(frame)

        # PHASE 2 — Target selection
        target = self.target_selector.select_target(detections)
        self.current_target = target

        # DEPTH — Estimate and apply auto-engage logic
        depth = 0.0
        if target:
            depth = self.depth_estimator.estimate(target, config.FRAME_HEIGHT)
            self._update_depth_trigger(depth)
        else:
            # Reset counters when no target in view
            self._engage_counter  = 0
            self._retreat_counter = 0
        self._last_depth = depth

        # Update depth UI
        if self.depth_ui:
            self.depth_ui.update(depth)

        # PHASE 2 — Tracking / engagement
        if target:
            self._process_target(target, frame)
        else:
            self._handle_no_target()

        # PHASE 1 — Visualise
        annotated = self.visualizer.draw_detections(frame, detections)

        # PHASE 2 — Action overlays
        annotated = self._add_action_overlays(annotated, target, depth)

        return annotated

    # ──────────────────────────────────────────────────────────────────────────
    def _update_depth_trigger(self, depth: float) -> None:
        """
        Hysteresis-controlled mode switching based on target depth.

        MONITOR → ENGAGE when depth ≥ threshold for DEPTH_ENGAGE_FRAMES frames.
        ENGAGE → MONITOR when depth < threshold for DEPTH_RETREAT_FRAMES frames.
        """
        # Read live threshold from UI slider (or fall back to config)
        threshold = self.depth_ui.threshold if self.depth_ui else config.DEPTH_THRESHOLD

        current_mode = self.action_controller.current_mode

        if depth >= threshold:
            self._engage_counter  += 1
            self._retreat_counter  = 0

            if (self._engage_counter >= config.DEPTH_ENGAGE_FRAMES
                    and current_mode == ActionMode.MONITOR):
                print(f"\n[DEPTH] 🚨 Target crossed threshold! "
                      f"({depth:.1f}% ≥ {threshold}%) → ENGAGE")
                self.change_mode(ActionMode.ENGAGE)
                self._engage_counter = 0
        else:
            self._retreat_counter += 1
            self._engage_counter   = 0

            if (self._retreat_counter >= config.DEPTH_RETREAT_FRAMES
                    and current_mode == ActionMode.ENGAGE):
                print(f"\n[DEPTH] Target retreated below threshold "
                      f"({depth:.1f}% < {threshold}%) → MONITOR")
                self.change_mode(ActionMode.MONITOR)
                self._retreat_counter = 0

    # ──────────────────────────────────────────────────────────────────────────
    def _process_target(self, target, frame):
        if self.action_controller.should_track():
            cx, cy = target.center
            self.turret.track_target(cx, cy, config.FRAME_WIDTH, config.FRAME_HEIGHT)

            is_centered = self.turret.is_centered(
                cx, cy,
                config.FRAME_WIDTH, config.FRAME_HEIGHT,
                config.CENTER_TOLERANCE_X, config.CENTER_TOLERANCE_Y
            )

            if is_centered and self.action_controller.should_engage(is_centered):
                self._engage_target(target)

    def _engage_target(self, target):
        target_info = f"{target.class_name} (conf: {target.confidence:.2%})"
        self.action_controller.record_engagement(target_info)

    def _handle_no_target(self):
        if config.AUTO_SCAN_ON_NO_TARGET:
            current_time = time.time()
            if current_time - self.last_scan_time > config.SCAN_INTERVAL:
                if self.action_controller.config.allow_movement:
                    self.turret.scan_area()
                    self.last_scan_time = current_time

    # ──────────────────────────────────────────────────────────────────────────
    def _add_action_overlays(self, frame, target, depth: float):
        """Add action layer visual overlays including depth bar on target."""
        h, w = frame.shape[:2]

        # Mode indicator — top left
        if config.SHOW_MODE_INDICATOR:
            mode = self.action_controller.current_mode
            mode_colors = {
                ActionMode.STANDBY: config.COLOR_MODE_STANDBY,
                ActionMode.MONITOR: config.COLOR_MODE_MONITOR,
                ActionMode.ENGAGE:  config.COLOR_MODE_ENGAGE,
                ActionMode.ABORT:   config.COLOR_MODE_ABORT,
            }
            mode_text  = f"MODE: {mode.value.upper()}"
            mode_color = mode_colors.get(mode, config.COLOR_TEXT)
            cv2.putText(frame, mode_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)

        # Target reticle + depth bar below target bbox
        if target and config.SHOW_TARGET_RETICLE:
            cx, cy = target.center
            cv2.drawMarker(frame, (cx, cy), config.COLOR_RETICLE,
                           cv2.MARKER_CROSS, 30, 2)
            cv2.circle(frame, (cx, cy), 40, config.COLOR_RETICLE, 2)

            # Depth bar below the target bbox
            threshold = self.depth_ui.threshold if self.depth_ui else config.DEPTH_THRESHOLD
            self._draw_depth_bar(frame, target, depth, threshold)

        # Pan/tilt position — bottom left
        if config.SHOW_POSITION_INFO:
            pan, tilt = self.turret.get_position()
            pos_text = f"Pan: {pan:.0f}° | Tilt: {tilt:.0f}°"
            cv2.putText(frame, pos_text, (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, config.COLOR_TEXT, 2)

        return frame

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _draw_depth_bar(frame, target, depth: float, threshold: int) -> None:
        """Draw a small depth progress bar beneath the target bounding box."""
        bar_h    = 6
        bar_y0   = target.y2 + 4
        bar_y1   = bar_y0 + bar_h
        bar_x0   = target.x1
        bar_x1   = target.x2
        bar_w    = max(bar_x1 - bar_x0, 1)

        # Background
        cv2.rectangle(frame, (bar_x0, bar_y0), (bar_x1, bar_y1), (50, 50, 50), -1)

        # Fill
        fill_w     = int(bar_w * depth / 100.0)
        triggered  = depth >= threshold
        fill_color = (0, 80, 255) if triggered else (0, 200, 100)
        if fill_w > 0:
            cv2.rectangle(frame, (bar_x0, bar_y0),
                          (bar_x0 + fill_w, bar_y1), fill_color, -1)

        # Threshold tick
        tick_x = bar_x0 + int(bar_w * threshold / 100.0)
        cv2.line(frame, (tick_x, bar_y0 - 2), (tick_x, bar_y1 + 2), (0, 220, 255), 1)

        # Depth text
        cv2.putText(frame, f"{depth:.0f}%",
                    (bar_x0, bar_y0 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, fill_color, 1, cv2.LINE_AA)

    # ──────────────────────────────────────────────────────────────────────────
    def change_mode(self, mode: ActionMode):
        """Change action mode with appropriate turret actions."""
        if self.action_controller.set_mode(mode):
            print(f"\n{'='*60}")
            print(f"MODE CHANGED: {mode.value.upper()}")
            print(f"{'='*60}\n")

            if mode == ActionMode.ABORT:
                self.turret.emergency_stop()
            elif mode == ActionMode.STANDBY:
                self.turret.reset()

    def handle_keyboard(self, key: int) -> bool:
        if key in (ord('q'), 27):
            return False
        elif key == ord('r'):
            print("\n[TURRET] Resetting to center position...")
            self.turret.reset()
        elif key == ord('s'):
            print("\n[TURRET] Manual scan initiated...")
            self.turret.scan_area()
        elif key == ord('1'):
            self.change_mode(ActionMode.STANDBY)
        elif key == ord('2'):
            self.change_mode(ActionMode.MONITOR)
        elif key == ord('3'):
            self.change_mode(ActionMode.ENGAGE)
        elif key == ord('0'):
            self.change_mode(ActionMode.ABORT)
        return True

    # ──────────────────────────────────────────────────────────────────────────
    def run(self):
        """Main execution loop"""
        print("[SYSTEM] Starting main loop...\n")
        self.running = True

        try:
            while self.running:
                frame = self.camera.read()
                if frame is None:
                    print("[ERROR] Failed to capture frame")
                    break

                annotated = self.process_frame(frame)
                cv2.imshow('RAKSHAQ Turret System', annotated)

                key = cv2.waitKey(1) & 0xFF
                if not self.handle_keyboard(key):
                    break

                self.frame_count += 1

        except KeyboardInterrupt:
            print("\n[SYSTEM] Interrupted by user")

        except Exception as e:
            print(f"\n[ERROR] System error: {e}")
            import traceback
            traceback.print_exc()
            self.turret.emergency_stop()

        finally:
            self.cleanup()

    # ──────────────────────────────────────────────────────────────────────────
    def cleanup(self):
        """Clean up resources"""
        print("\n" + "=" * 60)
        print("SYSTEM SHUTDOWN")
        print("=" * 60)

        self.running = False

        if self.depth_ui:
            self.depth_ui.close()

        print("[CLEANUP] Stopping camera...")
        self.camera.release()

        print("[CLEANUP] Resetting turret...")
        self.turret.cleanup()

        cv2.destroyAllWindows()

        print(f"\n[STATS] Frames processed: {self.frame_count}")
        print(f"[STATS] Final mode: {self.action_controller.current_mode.value.upper()}")
        print("\n✅ Shutdown complete")
        print("=" * 60 + "\n")


def main():
    system = RakshaqSystem()
    system.run()


if __name__ == "__main__":
    main()