"""
stm32.py — STM32/Nucleo stepper motor controller via serial.

Motor math (1/16 microstepping):
    400 steps = 45°   →   8.888 steps per degree

Commands sent as ASCII lines:
    X<steps>\n   — pan  (positive = right, negative = left)
    Y<steps>\n   — tilt (positive = up,    negative = down)

Falls back to mock (console print) if the serial port is unavailable.
"""

from __future__ import annotations
import time


STEPS_PER_DEGREE = 400 / 45   # ≈ 8.888


class STM32Controller:
    """
    Pan/tilt controller that talks to an STM32/Nucleo over serial.

    Parameters
    ----------
    port : str
        Serial device path, e.g. "/dev/ttyACM0".
    baud : int
        Baud rate (default 115200).
    pan_limit : int
        Max cumulative pan steps from home (software limit).
    tilt_limit : int
        Max cumulative tilt steps from home (software limit).
    deadband : int
        Minimum step magnitude to send — prevents micro-jitter.
    """

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud: int = 115200,
        pan_limit: int = 3200,   # ±360°
        tilt_limit: int = 800,   # ±90°
        deadband: int = 4,
    ):
        self._port      = port
        self._baud      = baud
        self._pan_limit = pan_limit
        self._tilt_limit = tilt_limit
        self._deadband  = deadband
        self._ser       = None
        self._connected = False

        # Software position tracking (steps from home)
        self._pan_steps  = 0
        self._tilt_steps = 0

        self._connect()

    # ──────────────────────────────────────────────────────────────────────────
    def _connect(self) -> None:
        try:
            import serial
            self._ser       = serial.Serial(self._port, self._baud, timeout=0.1)
            self._connected = True
            print(f"[STM32] Connected on {self._port} @ {self._baud} baud")
        except Exception as exc:
            self._connected = False
            print(f"[STM32] Not found ({exc}) — mock mode active")

    # ──────────────────────────────────────────────────────────────────────────
    def send(self, cmd: str) -> None:
        """Send a raw command string (newline appended automatically)."""
        if self._connected and self._ser:
            try:
                self._ser.write((cmd + "\n").encode())
            except Exception as exc:
                print(f"[STM32] Write error: {exc}")
                self._connected = False
        else:
            print(f"[STM32 MOCK] {cmd}")

    # ──────────────────────────────────────────────────────────────────────────
    def pan(self, steps: int) -> None:
        """
        Relative pan move.

        Parameters
        ----------
        steps : int
            Positive = right, negative = left.
        """
        if abs(steps) < self._deadband:
            return
        new_pos = self._pan_steps + steps
        if abs(new_pos) > self._pan_limit:
            return   # software limit reached
        self.send(f"X{steps}")
        self._pan_steps = new_pos

    def tilt(self, steps: int) -> None:
        """
        Relative tilt move.

        Parameters
        ----------
        steps : int
            Positive = up (Y+), negative = down (Y-).
        """
        if abs(steps) < self._deadband:
            return
        new_pos = self._tilt_steps + steps
        if abs(new_pos) > self._tilt_limit:
            return
        self.send(f"Y{steps}")
        self._tilt_steps = new_pos

    # ──────────────────────────────────────────────────────────────────────────
    def reset(self) -> None:
        """Return to home (0, 0) by sending the inverse of current position."""
        print("[STM32] Returning to home position")
        if self._pan_steps != 0:
            self.send(f"X{-self._pan_steps}")
            self._pan_steps = 0
        if self._tilt_steps != 0:
            self.send(f"Y{-self._tilt_steps}")
            self._tilt_steps = 0

    def stop(self) -> None:
        """Send zero-step commands (no-op for most firmware, useful as heartbeat)."""
        self.send("X0")
        self.send("Y0")

    # ──────────────────────────────────────────────────────────────────────────
    def get_position_deg(self) -> tuple[float, float]:
        """Return (pan_deg, tilt_deg) relative to home."""
        pan_deg  = self._pan_steps  / STEPS_PER_DEGREE
        tilt_deg = self._tilt_steps / STEPS_PER_DEGREE
        return (pan_deg, tilt_deg)

    # ──────────────────────────────────────────────────────────────────────────
    def cleanup(self) -> None:
        self.reset()
        time.sleep(0.15)
        if self._ser:
            self._ser.close()
        self._connected = False
        print("[STM32] Cleaned up")

    @property
    def connected(self) -> bool:
        return self._connected
