"""
RAKSHAQ - Autonomous AI Turret System
Main entry point with integrated Phase 1 (Detection) and Phase 2 (Action Layer)
"""

import cv2
import time

import config
from src.turret.detection.camera import get_camera
from src.turret.detection.detector import YOLODetector
from src.turret.utils.visualizer import Visualizer

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
        
        # Phase 1: Detection (Your teammate's work)
        print("[INIT] Phase 1: Detection System")
        print("-" * 60)
        
        # Camera
        print("[CAMERA] Setting up camera...")
        self.camera = get_camera(
            source=config.CAMERA_SOURCE,
            width=config.FRAME_WIDTH,
            height=config.FRAME_HEIGHT
        )
        
        # Detector
        print("[DETECTOR] Loading YOLO model...")
        self.detector = YOLODetector(
            model_path=config.MODEL_PATH,
            confidence=config.CONFIDENCE,
            iou_thresh=config.IOU_THRESH,
            track_classes=config.TRACK_CLASSES
        )
        
        # Visualizer
        self.visualizer = Visualizer(show_fps=config.SHOW_FPS)
        
        # Phase 2: Action Layer (Your work)
        print()
        print("[INIT] Phase 2: Action Layer")
        print("-" * 60)
        
        # Action controller
        mode_map = {
            "standby": ActionMode.STANDBY,
            "monitor": ActionMode.MONITOR,
            "engage": ActionMode.ENGAGE,
            "abort": ActionMode.ABORT
        }
        initial_mode = mode_map.get(config.DEFAULT_MODE, ActionMode.MONITOR)
        self.action_controller = ActionController(initial_mode)
        
        # Turret controller
        self.turret = TurretController(
            pan_pin=config.PAN_SERVO_PIN,
            tilt_pin=config.TILT_SERVO_PIN,
            servo_frequency=config.SERVO_FREQUENCY,
            smoothing_factor=config.SMOOTHING_FACTOR
        )
        
        # Target selector
        self.target_selector = TargetSelector(
            strategy=config.TARGETING_STRATEGY,
            min_confidence=config.CONFIDENCE
        )
        self.target_selector.update_frame_size(config.FRAME_WIDTH, config.FRAME_HEIGHT)
        
        # State
        self.running = False
        self.current_target = None
        self.last_scan_time = 0
        self.frame_count = 0
        
        print()
        print("[INIT] ✅ System ready!")
        print(f"[MODE] Current mode: {self.action_controller.current_mode.value.upper()}")
        self._print_controls()
    
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
    
    def process_frame(self, frame):
        """
        Process frame through detection and action pipeline
        
        Args:
            frame: Input frame from camera
            
        Returns:
            Annotated frame for display
        """
        # PHASE 1
        detections = self.detector.detect(frame)
        
        # PHASE 2
        target = self.target_selector.select_target(detections)
        self.current_target = target
        
        # PHASE 2: Process based on action mode 
        if target:
            self._process_target(target, frame)
        else:
            self._handle_no_target()
        
        # PHASE 1: Visualize detections
        annotated = self.visualizer.draw_detections(frame, detections)
        
        # PHASE 2: Add action layer overlays 
        annotated = self._add_action_overlays(annotated, target)
        
        return annotated
    
    def _process_target(self, target, frame):
        """
        Process detected target based on action mode
        
        Args:
            target: Selected Detection object
            frame: Current frame
        """
        # Check if should track
        if self.action_controller.should_track():
            cx, cy = target.center
            self.turret.track_target(cx, cy, config.FRAME_WIDTH, config.FRAME_HEIGHT)
            
            # Check if centered and should engage
            is_centered = self.turret.is_centered(
                cx, cy,
                config.FRAME_WIDTH, config.FRAME_HEIGHT,
                config.CENTER_TOLERANCE_X, config.CENTER_TOLERANCE_Y
            )
            
            if is_centered and self.action_controller.should_engage(is_centered):
                self._engage_target(target)
    
    def _engage_target(self, target):
        """
        Engage target (simulated for safety)
        
        Args:
            target: Target Detection object
        """
        target_info = f"{target.class_name} (conf: {target.confidence:.2%})"
        self.action_controller.record_engagement(target_info)
        
        # In real deployment, this would trigger actual engagement
        # For now, just visual/audio feedback
    
    def _handle_no_target(self):
        """Handle case when no target is detected"""
        if config.AUTO_SCAN_ON_NO_TARGET:
            current_time = time.time()
            if current_time - self.last_scan_time > config.SCAN_INTERVAL:
                if self.action_controller.config.allow_movement:
                    self.turret.scan_area()
                    self.last_scan_time = current_time
    
    def _add_action_overlays(self, frame, target):
        """
        Add action layer visual overlays
        
        Args:
            frame: Annotated frame
            target: Selected target (or None)
            
        Returns:
            Frame with action overlays
        """
        # Mode indicator
        if config.SHOW_MODE_INDICATOR:
            mode = self.action_controller.current_mode
            mode_colors = {
                ActionMode.STANDBY: config.COLOR_MODE_STANDBY,
                ActionMode.MONITOR: config.COLOR_MODE_MONITOR,
                ActionMode.ENGAGE: config.COLOR_MODE_ENGAGE,
                ActionMode.ABORT: config.COLOR_MODE_ABORT
            }
            
            mode_text = f"MODE: {mode.value.upper()}"
            mode_color = mode_colors.get(mode, config.COLOR_TEXT)
            
            cv2.putText(frame, mode_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
        
        # Target reticle
        if target and config.SHOW_TARGET_RETICLE:
            cx, cy = target.center
            # Draw crosshair
            cv2.drawMarker(frame, (cx, cy), config.COLOR_RETICLE,
                          cv2.MARKER_CROSS, 30, 2)
            # Draw circle
            cv2.circle(frame, (cx, cy), 40, config.COLOR_RETICLE, 2)
        
        # Position info
        if config.SHOW_POSITION_INFO:
            pan, tilt = self.turret.get_position()
            pos_text = f"Pan: {pan:.0f}° | Tilt: {tilt:.0f}°"
            cv2.putText(frame, pos_text, (10, config.FRAME_HEIGHT - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, config.COLOR_TEXT, 2)
        
        return frame
    
    def change_mode(self, mode: ActionMode):
        """Change action mode with appropriate actions"""
        if self.action_controller.set_mode(mode):
            print(f"\n{'='*60}")
            print(f"MODE CHANGED: {mode.value.upper()}")
            print(f"{'='*60}\n")
            
            # Mode-specific actions
            if mode == ActionMode.ABORT:
                self.turret.emergency_stop()
            elif mode == ActionMode.STANDBY:
                self.turret.reset()
    
    def handle_keyboard(self, key: int) -> bool:
        """
        Handle keyboard input
        
        Args:
            key: Key code from cv2.waitKey()
            
        Returns:
            True to continue, False to quit
        """
        if key == ord('q') or key == 27:  # q or Esc
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
    
    def run(self):
        """Main execution loop"""
        print("[SYSTEM] Starting main loop...\n")
        self.running = True
        
        try:
            while self.running:
                # Capture frame
                frame = self.camera.read()
                if frame is None:
                    print("[ERROR] Failed to capture frame")
                    break
                
                # Process through pipeline
                annotated = self.process_frame(frame)
                
                # Display
                cv2.imshow('RAKSHAQ Turret System', annotated)
                
                # Handle keyboard
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
    
    def cleanup(self):
        """Clean up resources"""
        print("\n" + "=" * 60)
        print("SYSTEM SHUTDOWN")
        print("=" * 60)
        
        self.running = False
        
        # Stop camera
        print("[CLEANUP] Stopping camera...")
        self.camera.release()
        
        # Reset turret
        print("[CLEANUP] Resetting turret...")
        self.turret.cleanup()
        
        # Close windows
        cv2.destroyAllWindows()
        
        # Print stats
        print(f"\n[STATS] Frames processed: {self.frame_count}")
        print(f"[STATS] Final mode: {self.action_controller.current_mode.value.upper()}")
        
        print("\n✅ Shutdown complete")
        print("=" * 60 + "\n")


def main():
    """Entry point"""
    system = RakshaqSystem()
    system.run()


if __name__ == "__main__":
    main()