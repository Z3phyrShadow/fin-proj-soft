"""
radar.py — Servo-driven sonar radar.

A single servo sweeps a cone from -HALF_ANGLE to +HALF_ANGLE degrees and back
continuously, while reading the HC-SR04 sonar at each position. The resulting
(angle, distance_mm) pairs are stored for the dashboard radar display.

Hardware:
    Servo signal wire → GPIO 12 (Pi physical pin 32) — hardware PWM capable.
    Sonar stays wired as-is; we pass the existing UltrasonicSensor reference in.

Servo value mapping (gpiozero convention):
    value = -1  →  -90°  (full left)
    value =  0  →    0°  (center)
    value = +1  →  +90°  (full right)
    ±45° therefore maps to value ±0.5

Falls back silently to mock mode if gpiozero / GPIO is unavailable.
"""

from __future__ import annotations
import threading
import time


class RadarServo:
    """
    Continuously sweeps a servo through ±half_angle degrees while
    sampling the attached sonar sensor.

    Parameters
    ----------
    gpio_pin : int
        BCM GPIO pin connected to the servo signal wire (default 12).
    half_angle : float
        Half-width of the sweep cone in degrees (default 45 → ±45°).
    step_deg : float
        Angular step per iteration in degrees (smaller = smoother but slower).
    step_delay : float
        Seconds to wait at each step (controls sweep speed and sonar settle time).
    sonar : UltrasonicSensor | None
        If provided, .distance_mm is sampled at each step.
    """

    def __init__(
        self,
        gpio_pin:   int   = 12,
        half_angle: float = 45.0,
        step_deg:   float = 2.0,
        step_delay: float = 0.03,
        sonar              = None,
    ):
        self._pin        = gpio_pin
        self._half       = half_angle
        self._step_deg   = step_deg
        self._step_delay = step_delay
        self._sonar      = sonar

        self._angle:      float = 0.0
        self._direction:  int   = 1          # +1 sweeping right, -1 left
        self._running:    bool  = False
        self._thread:     threading.Thread | None = None
        self._servo       = None
        self._available:  bool = False

        # Scan map: angle_bucket → distance_mm (bucket = rounded to nearest step)
        self._scan_map:   dict[int, int] = {}
        self._lock = threading.Lock()

    # ──────────────────────────────────────────────────────────────────────────
    def start(self) -> None:
        """Open servo connection and start the sweep thread."""
        try:
            from gpiozero import Servo  # type: ignore

            # Prefer pigpio (hardware PWM → jitter-free servo) but fall back
            # to the default software PWM factory gracefully.
            try:
                from gpiozero.pins.pigpio import PiGPIOFactory  # type: ignore
                self._servo = Servo(self._pin, pin_factory=PiGPIOFactory())
                print(f"[RADAR] Using hardware PWM (pigpio) on GPIO {self._pin}")
            except Exception:
                self._servo = Servo(self._pin)
                print(f"[RADAR] Using software PWM on GPIO {self._pin}")

            self._available = True

        except Exception as exc:
            print(f"[RADAR] Not available ({exc}) — mock mode (angle still updates)")
            self._available = False

        self._running = True
        self._thread = threading.Thread(
            target=self._sweep_loop, daemon=True, name="RadarThread"
        )
        self._thread.start()
        print(f"[RADAR] Sweep started: ±{self._half}°  step={self._step_deg}°  "
              f"delay={self._step_delay}s")

    # ──────────────────────────────────────────────────────────────────────────
    def stop(self) -> None:
        """Stop the sweep and return servo to centre."""
        self._running = False
        if self._servo:
            try:
                self._servo.mid()          # return to 0°
                time.sleep(0.3)
                self._servo.detach()
            except Exception:
                pass

    # ──────────────────────────────────────────────────────────────────────────
    def _angle_to_servo_value(self, deg: float) -> float:
        """Convert ±90° range to gpiozero servo value (-1 … +1)."""
        return max(-1.0, min(1.0, deg / 90.0))

    # ──────────────────────────────────────────────────────────────────────────
    def _sweep_loop(self) -> None:
        """Background sweep loop. Steps servo and records sonar distance."""
        angle     = 0.0
        direction = 1

        while self._running:
            # ── Command servo ─────────────────────────────────────────────────
            if self._servo:
                try:
                    self._servo.value = self._angle_to_servo_value(angle)
                except Exception:
                    pass

            # ── Wait for servo to settle and sonar to update ──────────────────
            time.sleep(self._step_delay)

            # ── Sample sonar ──────────────────────────────────────────────────
            dist_mm = 0
            if self._sonar is not None:
                try:
                    dist_mm = self._sonar.distance_mm
                except Exception:
                    pass

            # ── Store reading ─────────────────────────────────────────────────
            bucket = int(round(angle))
            with self._lock:
                self._angle = angle
                self._scan_map[bucket] = dist_mm

            # ── Advance angle ─────────────────────────────────────────────────
            angle += direction * self._step_deg
            if angle >= self._half:
                angle     = self._half
                direction = -1
            elif angle <= -self._half:
                angle     = -self._half
                direction = 1

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def angle(self) -> float:
        """Current servo angle in degrees (−half_angle … +half_angle)."""
        with self._lock:
            return self._angle

    @property
    def scan_points(self) -> list[dict]:
        """
        Snapshot of the latest scan as a list of dicts:
            [{"angle": int, "dist": int}, ...]
        Sorted by angle for easy rendering.
        """
        with self._lock:
            return [
                {"angle": a, "dist": d}
                for a, d in sorted(self._scan_map.items())
            ]

    @property
    def available(self) -> bool:
        return self._available
