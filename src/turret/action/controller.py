"""
controller.py — TurretController backed by STM32 stepper motors.

Motor math (1/16 microstepping):
    400 steps = 45°  →  8.888 steps/degree

Tracking flow:
    pixel error → degrees → steps → X/Y serial command to STM32

Pan:  positive X = right,   negative X = left
Tilt: positive Y = up,      negative Y = down

Camera rotation handling:
    When the camera is physically mounted sideways and the image is
    software-rotated, the pixel axes in the displayed frame no longer
    line up with the physical pan/tilt axes.  This controller swaps
    and sign-corrects the error axes based on camera_rotate so the
    motors always move towards the target, not away from it.
"""

from __future__ import annotations
import time
import config
from src.turret.hardware.stm32 import STM32Controller, STEPS_PER_DEGREE


class TurretController:
    """
    High-level turret controller: converts vision pixel errors into
    STM32 step commands for pan/tilt stepper motors.

    Parameters
    ----------
    stm32_port : str
        Serial port for the STM32 Nucleo.
    stm32_baud : int
        Baud rate.
    hfov_deg : float
        Horizontal field-of-view of the camera (degrees).
    vfov_deg : float
        Vertical field-of-view of the camera (degrees).
    pan_invert : bool
        Flip pan direction if wiring is reversed.
    tilt_invert : bool
        Flip tilt direction if wiring is reversed.
    """

    def __init__(
        self,
        stm32_port: str   = "/dev/ttyACM0",
        stm32_baud: int   = 115200,
        hfov_deg:  float  = 66.0,
        vfov_deg:  float  = 52.0,
        pan_invert:  bool = False,
        tilt_invert: bool = False,
        camera_rotate: str = "none",
    ):
        self._stm32       = STM32Controller(port=stm32_port, baud=stm32_baud)
        self._hfov        = hfov_deg
        self._vfov        = vfov_deg
        self._pan_inv     = -1 if pan_invert  else 1
        self._tilt_inv    = -1 if tilt_invert else 1
        self._camera_rotate = camera_rotate.lower()
        print(f"[TURRET] Initialized (STM32 on {stm32_port}, rotate={self._camera_rotate})")

    # ──────────────────────────────────────────────────────────────────────────
    def track_target(
        self,
        target_cx: int,
        target_cy: int,
        frame_width: int,
        frame_height: int,
    ) -> None:
        """
        Compute pixel error from frame centre and send motor commands.

        Error → degrees → steps → X/Y command.

        When the camera is rotated, the pixel axes in the displayed frame
        are swapped/flipped relative to the physical sensor axes.  We undo
        the rotation on the error vector so that the pan/tilt motors always
        move in the correct physical direction.

        Rotation axis mapping (post-rotation screen → physical sensor):
          none : screen-X → sensor-X,  screen-Y → sensor-Y
          90cw : screen-X → sensor-Y,  screen-Y → -sensor-X
          90ccw: screen-X → -sensor-Y, screen-Y → sensor-X
          180  : screen-X → -sensor-X, screen-Y → -sensor-Y
        """
        error_x = target_cx - frame_width  // 2
        error_y = target_cy - frame_height // 2

        # Un-rotate the error vector so it maps to physical pan/tilt axes.
        # After un-rotation, pan_error aligns with the physical pan motor and
        # tilt_error aligns with the physical tilt motor on the raw sensor.
        #
        # cv2 rotation transforms  (W×H original → rotated image):
        #   90CW:  (x,y) → (H-1-y, x)      →  inverse: sensor_X = screen_Y, sensor_Y = -screen_X
        #   90CCW: (x,y) → (y, W-1-x)       →  inverse: sensor_X = -screen_Y, sensor_Y = screen_X
        #   180:   (x,y) → (W-1-x, H-1-y)   →  inverse: sensor_X = -screen_X, sensor_Y = -screen_Y
        rot = self._camera_rotate
        if rot == "90cw":
            pan_error  =  error_y     # screen-Y → physical pan (sensor-X)
            tilt_error = -error_x     # -screen-X → physical tilt (sensor-Y)
            # FOV: the config's HFOV/VFOV refer to the *rotated* image axes,
            # so HFOV = rotated-X FOV, VFOV = rotated-Y FOV.
            # pan_error came from screen-Y, so use VFOV; tilt from screen-X, so use HFOV.
            deg_pan  = (pan_error  / frame_height) * self._vfov
            deg_tilt = (tilt_error / frame_width)  * self._hfov
        elif rot == "90ccw":
            pan_error  = -error_y
            tilt_error =  error_x
            deg_pan  = (pan_error  / frame_height) * self._vfov
            deg_tilt = (tilt_error / frame_width)  * self._hfov
        elif rot == "180":
            pan_error  = -error_x
            tilt_error = -error_y
            deg_pan  = (pan_error  / frame_width)  * self._hfov
            deg_tilt = (tilt_error / frame_height) * self._vfov
        else:  # "none" or unrecognised
            deg_pan  = (error_x / frame_width)  * self._hfov
            deg_tilt = (error_y / frame_height) * self._vfov

        # Proportional gain — correct only a fraction of the error per frame
        # to prevent oscillation (tune TRACKING_P_GAIN in config.py)
        p = config.TRACKING_P_GAIN

        steps_pan  = int(deg_pan   * STEPS_PER_DEGREE * p) * self._pan_inv
        steps_tilt = int(-deg_tilt * STEPS_PER_DEGREE * p) * self._tilt_inv

        # Max steps per frame to avoid overshooting
        max_step = config.MOTOR_MAX_STEPS_PER_FRAME
        steps_pan  = max(-max_step, min(max_step, steps_pan))
        steps_tilt = max(-max_step, min(max_step, steps_tilt))

        self._stm32.pan(steps_pan)
        self._stm32.tilt(steps_tilt)

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def is_centered(
        target_cx: int,
        target_cy: int,
        frame_width: int,
        frame_height: int,
        tolerance_x: int = 50,
        tolerance_y: int = 50,
    ) -> bool:
        """Return True if target is within tolerance pixels of frame centre."""
        return (
            abs(target_cx - frame_width  // 2) < tolerance_x
            and abs(target_cy - frame_height // 2) < tolerance_y
        )

    def get_position(self) -> tuple[float, float]:
        """Return (pan_deg, tilt_deg) relative to home position."""
        return self._stm32.get_position_deg()

    # ──────────────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Return pan to home; tilt stays at current elevation."""
        print("[TURRET] Pan → home (tilt held)")
        self._stm32.reset(reset_tilt=False)

    def scan_area(self) -> None:
        """Simple scan sweep — pan ±45° from current position."""
        print("[TURRET] Scanning area...")
        scan_steps = int(45 * STEPS_PER_DEGREE)   # 45 degrees
        self._stm32.pan(scan_steps)
        time.sleep(0.8)
        self._stm32.pan(-scan_steps * 2)
        time.sleep(0.8)
        self._stm32.pan(scan_steps)               # return to centre estimate

    def emergency_stop(self) -> None:
        """Emergency stop — return home."""
        print("[TURRET] ⚠️ EMERGENCY STOP")
        self.reset()

    def cleanup(self) -> None:
        """Release resources."""
        print("[TURRET] Cleaning up...")
        self._stm32.cleanup()