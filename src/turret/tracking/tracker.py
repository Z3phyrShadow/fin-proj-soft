"""
tracker.py — PID-based turret tracking state machine.

Replaces the old TurretController with a clean state machine:

    IDLE → TRACKING → CHASING → SEARCHING → IDLE
              ↑          │          │
              └──────────┴──────────┘  (target re-acquired)

The tracker is called once per frame with the selected target (or None)
and produces motor commands via the STM32 driver.

Features:
    • Dual-axis PID control for smooth, oscillation-free tracking
    • Camera-rotation-aware axis mapping (90CW, 90CCW, 180)
    • Parallax compensation for the camera-to-barrel offset
    • Dead-zone near centre to prevent micro-jitter
    • Chase mode: follows last-known trajectory when target is lost
    • Search mode: systematic pan sweep after chase timeout
    • Rate-limited motor output to prevent violent snapping
"""

from __future__ import annotations

import time
from collections import deque
from enum import Enum

from src.turret.hardware.stm32 import STM32Controller, STEPS_PER_DEGREE
from src.turret.tracking.pid import PID
from src.turret.tracking.axis_mapper import AxisMapper
from src.turret.tracking.parallax import ParallaxCompensator


class TrackState(Enum):
    """Tracking state machine states."""
    IDLE       = "idle"        # No target, not searching
    TRACKING   = "tracking"    # Actively tracking a visible target
    CHASING    = "chasing"     # Target lost, following predicted path
    SEARCHING  = "searching"   # Chase timed out, sweeping area


class Tracker:
    """
    Main tracking controller.

    Parameters
    ----------
    stm32 : STM32Controller
        Low-level motor driver (pan/tilt step commands).
    axis_mapper : AxisMapper
        Pixel-error → degree converter (rotation-aware).
    parallax : ParallaxCompensator
        Barrel-offset correction.
    kp, ki, kd : float
        PID gains for both pan and tilt axes.
    i_max : float
        Anti-windup integral limit (degrees).
    deadzone_px : int
        Pixel error below which motors are not commanded.
    max_steps_per_frame : int
        Output rate limit (motor steps per frame, per axis).
    chase_timeout_s : float
        Seconds to chase predicted trajectory after target loss.
    search_timeout_s : float
        Seconds to sweep before giving up and returning to IDLE.
    predict_window : int
        Number of recent target positions to average for velocity.
    """

    def __init__(
        self,
        stm32: STM32Controller,
        axis_mapper: AxisMapper,
        parallax: ParallaxCompensator,
        *,
        kp: float                = 0.35,
        ki: float                = 0.02,
        kd: float                = 0.15,
        i_max: float             = 50.0,
        deadzone_px: int         = 15,
        max_steps_per_frame: int = 200,
        chase_timeout_s: float   = 2.0,
        search_timeout_s: float  = 10.0,
        predict_window: int      = 5,
    ):
        self._stm32    = stm32
        self._mapper   = axis_mapper
        self._parallax = parallax

        # PID limits: max output in degrees per frame
        max_deg = max_steps_per_frame / STEPS_PER_DEGREE
        self._pid_pan  = PID(kp, ki, kd, i_max=i_max,
                             output_min=-max_deg, output_max=max_deg)
        self._pid_tilt = PID(kp, ki, kd, i_max=i_max,
                             output_min=-max_deg, output_max=max_deg)

        self._deadzone_px  = deadzone_px
        self._max_steps    = max_steps_per_frame
        self._chase_timeout  = chase_timeout_s
        self._search_timeout = search_timeout_s

        # ── State ────────────────────────────────────────────────────────────
        self._state: TrackState = TrackState.IDLE
        self._lost_time: float  = 0.0       # time.monotonic() when target lost
        self._search_dir: int   = 1         # +1 = sweep right, -1 = sweep left
        self._search_steps: int = 0         # cumulative search steps

        # ── Target history (for velocity prediction) ─────────────────────────
        self._history: deque[tuple[float, int, int]] = deque(
            maxlen=predict_window
        )  # (timestamp, cx, cy)

        self._current_track_id: int | None = None

        print(f"[TRACKER] Initialized: PID({kp:.2f}, {ki:.3f}, {kd:.2f})  "
              f"deadzone={deadzone_px}px  chase={chase_timeout_s}s  "
              f"search={search_timeout_s}s")

    # ══════════════════════════════════════════════════════════════════════════
    #  Public API
    # ══════════════════════════════════════════════════════════════════════════

    @property
    def state(self) -> TrackState:
        return self._state

    @property
    def current_track_id(self) -> int | None:
        return self._current_track_id

    # ──────────────────────────────────────────────────────────────────────────
    def update(
        self,
        target,
        detections: list,
        frame_w: int,
        frame_h: int,
        tof_mm: float = 0.0,
        allow_movement: bool = True,
    ) -> TrackState:
        """
        Run one tracking cycle.  Call every frame.

        Parameters
        ----------
        target : Detection | None
            Selected target from TargetSelector (None = no target).
        detections : list[Detection]
            All current detections (used for context, not directly).
        frame_w, frame_h : int
            Post-rotation frame dimensions.
        tof_mm : float
            Current ToF distance reading (mm).  0 = no data.
        allow_movement : bool
            False in STANDBY / ABORT modes — skips motor commands.

        Returns
        -------
        TrackState
            Current state after this cycle.
        """
        if not allow_movement:
            self._pid_pan.reset()
            self._pid_tilt.reset()
            return self._state

        if target is not None:
            return self._do_track(target, frame_w, frame_h, tof_mm)
        else:
            return self._do_lost(frame_w, frame_h)

    # ──────────────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Return to IDLE, zero PID state, send motors home."""
        self._state = TrackState.IDLE
        self._pid_pan.reset()
        self._pid_tilt.reset()
        self._history.clear()
        self._current_track_id = None
        self._search_steps = 0
        print("[TRACKER] Reset → IDLE")

    def emergency_stop(self) -> None:
        """Immediate stop — zero PID, return motors home."""
        self.reset()
        self._stm32.reset(reset_tilt=False)

    def home(self) -> None:
        """Send pan motor home (tilt held)."""
        self._stm32.reset(reset_tilt=False)

    def get_position(self) -> tuple[float, float]:
        """Return (pan_deg, tilt_deg) relative to home."""
        return self._stm32.get_position_deg()

    def cleanup(self) -> None:
        """Release hardware resources."""
        self._stm32.cleanup()

    # ══════════════════════════════════════════════════════════════════════════
    #  State machine internals
    # ══════════════════════════════════════════════════════════════════════════

    def _do_track(self, target, frame_w: int, frame_h: int, tof_mm: float) -> TrackState:
        """Target is visible — PID track it."""

        # Transition from any state → TRACKING
        if self._state != TrackState.TRACKING:
            if self._state == TrackState.IDLE:
                print(f"[TRACKER] Target acquired → TRACKING")
            elif self._state in (TrackState.CHASING, TrackState.SEARCHING):
                print(f"[TRACKER] Target re-acquired → TRACKING")
            self._state = TrackState.TRACKING
            self._pid_pan.reset()
            self._pid_tilt.reset()
            self._search_steps = 0

        # Update track ID
        if target.track_id is not None:
            self._current_track_id = target.track_id

        # Record position history for velocity prediction
        cx, cy = target.center
        self._history.append((time.monotonic(), cx, cy))

        # ── Pixel error from frame centre ────────────────────────────────────
        error_x = cx - frame_w // 2
        error_y = cy - frame_h // 2

        # Dead zone — don't move if target is close enough to centre
        if abs(error_x) < self._deadzone_px and abs(error_y) < self._deadzone_px:
            self._pid_pan.freeze_integral()
            self._pid_tilt.freeze_integral()
            return self._state

        # ── Convert to physical degrees ──────────────────────────────────────
        pan_deg, tilt_deg = self._mapper.to_degrees(
            error_x, error_y, frame_w, frame_h
        )

        # ── Parallax correction ──────────────────────────────────────────────
        pan_deg, tilt_deg = self._parallax.correct(pan_deg, tilt_deg, tof_mm)

        # ── PID ──────────────────────────────────────────────────────────────
        pan_out  = self._pid_pan.update(pan_deg)
        tilt_out = self._pid_tilt.update(tilt_deg)

        # ── Convert to steps and command ─────────────────────────────────────
        pan_steps  = int(pan_out  * STEPS_PER_DEGREE)
        tilt_steps = int(tilt_out * STEPS_PER_DEGREE)

        # Rate limit
        pan_steps  = max(-self._max_steps, min(self._max_steps, pan_steps))
        tilt_steps = max(-self._max_steps, min(self._max_steps, tilt_steps))

        self._stm32.pan(pan_steps)
        self._stm32.tilt(tilt_steps)

        return self._state

    # ──────────────────────────────────────────────────────────────────────────
    def _do_lost(self, frame_w: int, frame_h: int) -> TrackState:
        """Target is NOT visible — decide between chase, search, or idle."""

        now = time.monotonic()

        # ── Was TRACKING → enter CHASING ─────────────────────────────────────
        if self._state == TrackState.TRACKING:
            self._state = TrackState.CHASING
            self._lost_time = now
            self._pid_pan.reset()
            self._pid_tilt.reset()
            print("[TRACKER] Target lost → CHASING")

        # ── CHASING: follow predicted trajectory ─────────────────────────────
        if self._state == TrackState.CHASING:
            elapsed = now - self._lost_time
            if elapsed < self._chase_timeout:
                self._chase_predict()
                return self._state
            else:
                # Chase timed out → start searching
                self._state = TrackState.SEARCHING
                self._search_dir = self._guess_search_direction()
                self._search_steps = 0
                self._search_sweep_limit = int(90 * STEPS_PER_DEGREE)
                print("[TRACKER] Chase timeout → SEARCHING")

        # ── SEARCHING: systematic pan sweep ──────────────────────────────────
        if self._state == TrackState.SEARCHING:
            elapsed = now - self._lost_time
            if elapsed < self._lost_time + self._search_timeout:
                self._search_sweep()
                return self._state
            else:
                # Search timed out → give up
                print("[TRACKER] Search timeout → IDLE")
                self.reset()
                return self._state

        # ── IDLE: do nothing ─────────────────────────────────────────────────
        return self._state

    # ──────────────────────────────────────────────────────────────────────────
    def _chase_predict(self) -> None:
        """
        Move motors in the direction the target was last traveling.

        Uses a simple linear velocity estimate from the position history.
        """
        if len(self._history) < 2:
            return

        # Velocity = (last_pos - first_pos) / time_span
        t0, x0, y0 = self._history[0]
        t1, x1, y1 = self._history[-1]
        dt_hist = t1 - t0
        if dt_hist < 0.01:
            return

        vx = (x1 - x0) / dt_hist   # pixels per second
        vy = (y1 - y0) / dt_hist

        now = time.monotonic()
        # Ensure we only apply one frame's worth of delta since the last update
        dt_frame = now - getattr(self, '_last_chase_time', min(now - 0.033, t1))
        self._last_chase_time = now

        # Convert velocity (pixels/sec) to degrees/sec
        pan_deg_per_sec, tilt_deg_per_sec = self._mapper.to_degrees(
            vx, vy, 
            480, 360,  # approximate frame size
        )

        # Chase steps for this frame = degrees/sec * frame_duration * steps/deg
        chase_gain = 0.6  # slightly softened
        pan_steps  = int(pan_deg_per_sec  * dt_frame * STEPS_PER_DEGREE * chase_gain)
        tilt_steps = int(tilt_deg_per_sec * dt_frame * STEPS_PER_DEGREE * chase_gain)

        pan_steps  = max(-self._max_steps // 2, min(self._max_steps // 2, pan_steps))
        tilt_steps = max(-self._max_steps // 2, min(self._max_steps // 2, tilt_steps))

        if abs(pan_steps) > 0:
            self._stm32.pan(pan_steps)
        if abs(tilt_steps) > 0:
            self._stm32.tilt(tilt_steps)

    # ──────────────────────────────────────────────────────────────────────────
    def _guess_search_direction(self) -> int:
        """
        Guess which direction to sweep based on the target's last
        known velocity.  +1 = right, -1 = left.
        """
        if len(self._history) < 2:
            return 1   # default right

        _, x0, _ = self._history[0]
        _, x1, _ = self._history[-1]
        return 1 if (x1 - x0) >= 0 else -1

    def _search_sweep(self) -> None:
        """
        Slow pan sweep in the predicted direction.

        Reverses direction when the pan limit is approached, and sweeps
        back across the arc.
        """
        sweep_speed = 15   # Much slower step rate (was 40). ~1.7°/frame

        steps = sweep_speed * self._search_dir
        self._search_steps += abs(steps)

        if self._search_steps > getattr(self, '_search_sweep_limit', int(90 * STEPS_PER_DEGREE)):
            # Reverse direction
            self._search_dir *= -1
            self._search_steps = 0
            # Double the sweep distance for subsequent legs so it fully crosses the center tracking point
            self._search_sweep_limit = int(180 * STEPS_PER_DEGREE)

        self._stm32.pan(steps)

    # ──────────────────────────────────────────────────────────────────────────
    def scan_area(self) -> None:
        """
        Manual scan sweep — pan ±45° from current position.
        Blocking call (takes ~1.6 seconds).
        """
        print("[TRACKER] Scanning area...")
        scan_steps = int(45 * STEPS_PER_DEGREE)
        self._stm32.pan(scan_steps)
        import time as _t
        _t.sleep(0.8)
        self._stm32.pan(-scan_steps * 2)
        _t.sleep(0.8)
        self._stm32.pan(scan_steps)
