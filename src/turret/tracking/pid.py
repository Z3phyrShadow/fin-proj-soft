"""
pid.py — Time-aware PID controller for smooth turret tracking.

Each axis (pan, tilt) gets its own PID instance.  The controller
outputs a correction in *degrees* which the caller converts to
motor steps.

Features:
  • Time-based integral and derivative (robust to variable FPS)
  • Anti-windup clamp on the integral term
  • Output magnitude clamp
  • Manual reset for state-machine transitions
"""

from __future__ import annotations
import time


class PID:
    """
    Incremental PID controller.

    Parameters
    ----------
    kp : float
        Proportional gain.
    ki : float
        Integral gain (eliminates steady-state error).
    kd : float
        Derivative gain (dampens oscillation / overshoot).
    i_max : float
        Anti-windup: maximum absolute accumulated integral (degrees).
    output_min, output_max : float
        Clamp range for the final output (degrees).
    """

    __slots__ = (
        "kp", "ki", "kd",
        "_i_max", "_out_min", "_out_max",
        "_integral", "_prev_error", "_prev_time",
    )

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        i_max: float   = 50.0,
        output_min: float = -float("inf"),
        output_max: float =  float("inf"),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._i_max   = abs(i_max)
        self._out_min = output_min
        self._out_max = output_max

        self._integral:   float = 0.0
        self._prev_error: float = 0.0
        self._prev_time:  float | None = None

    # ──────────────────────────────────────────────────────────────────────────
    def update(self, error: float) -> float:
        """
        Compute PID output for the current error.

        Parameters
        ----------
        error : float
            Signed error in degrees (positive = target is to the right /
            below the aim point, depending on axis convention).

        Returns
        -------
        float
            Correction in degrees, clamped to [output_min, output_max].
        """
        now = time.monotonic()
        
        # Handle first frame after reset: prevent D-term spike
        if self._prev_time is None:
            dt = 0.033
            self._prev_error = error
        else:
            dt = max(now - self._prev_time, 1e-6)
            
        self._prev_time = now

        # ── Proportional ─────────────────────────────────────────────────────
        p_term = self.kp * error

        # ── Integral (with anti-windup) ──────────────────────────────────────
        self._integral += error * dt
        self._integral  = max(-self._i_max, min(self._i_max, self._integral))
        i_term = self.ki * self._integral

        # ── Derivative ───────────────────────────────────────────────────────
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # ── Output ───────────────────────────────────────────────────────────
        output = p_term + i_term + d_term
        return max(self._out_min, min(self._out_max, output))

    # ──────────────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Zero all internal state (call on target switch / mode change)."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None

    def freeze_integral(self) -> None:
        """Stop accumulating integral (useful in dead-zone)."""
        self._integral = 0.0
