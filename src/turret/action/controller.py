"""
controller.py — TurretController backed by STM32 stepper motors.

Motor math (1/16 microstepping):
    400 steps = 45°  →  8.888 steps/degree

Tracking flow:
    pixel error → degrees → steps → X/Y serial command to STM32

Pan:  positive X = right,   negative X = left
Tilt: positive Y = up,      negative Y = down
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
    ):
        self._stm32       = STM32Controller(port=stm32_port, baud=stm32_baud)
        self._hfov        = hfov_deg
        self._vfov        = vfov_deg
        self._pan_inv     = -1 if pan_invert  else 1
        self._tilt_inv    = -1 if tilt_invert else 1
        print(f"[TURRET] Initialized (STM32 on {stm32_port})")

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
        """
        error_x = target_cx - frame_width  // 2
        error_y = target_cy - frame_height // 2

        # Degrees to move
        deg_x = (error_x / frame_width)  * self._hfov
        deg_y = (error_y / frame_height) * self._vfov

        # Steps (tilt Y is negated: error_y > 0 means target is below → tilt up)
        steps_x = int(deg_x  * STEPS_PER_DEGREE) * self._pan_inv
        steps_y = int(-deg_y * STEPS_PER_DEGREE) * self._tilt_inv

        # Max steps per frame to avoid overshooting
        max_step = config.MOTOR_MAX_STEPS_PER_FRAME
        steps_x = max(-max_step, min(max_step, steps_x))
        steps_y = max(-max_step, min(max_step, steps_y))

        self._stm32.pan(steps_x)
        self._stm32.tilt(steps_y)

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