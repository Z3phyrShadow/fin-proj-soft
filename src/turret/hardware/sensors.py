"""
sensors.py — Distance sensors: TOF laser + HC-SR04 ultrasonic.

Both sensors run in daemon threads and expose a .distance_mm property.
Both degrade gracefully when hardware is unavailable.

TOF protocol (binary, /dev/ttyAMA0 @ 921600):
    16-byte frames: [0x57][0x00][...14 bytes...][checksum]
    Distance: bytes 8–10, little-endian, in mm
    Checksum: sum(bytes 0–15) & 0xFF == byte 15

Ultrasonic (gpiozero DistanceSensor):
    Returns metres → converted to mm
    Requires: sudo apt install python3-gpiozero
"""

from __future__ import annotations
import threading


class TOFSensor:
    """
    Background-thread TOF laser distance sensor reader.

    Parameters
    ----------
    port : str
        Serial device, typically "/dev/ttyAMA0".
    baud : int
        Baud rate (921600 for typical TOF modules).
    """

    def __init__(self, port: str = "/dev/ttyAMA0", baud: int = 921600):
        self._port       = port
        self._baud       = baud
        self._distance   = 0        # mm
        self._lock       = threading.Lock()
        self._running    = False
        self._thread     = None
        self._ser        = None
        self._available  = False

    # ──────────────────────────────────────────────────────────────────────────
    def start(self) -> None:
        """Open serial port and start background reader thread."""
        try:
            import serial
            self._ser = serial.Serial(self._port, self._baud, timeout=1)
            self._available = True
        except Exception as exc:
            print(f"[TOF] Not available ({exc}) — distance will read 0 mm")
            self._available = False
            return

        self._running = True
        self._thread  = threading.Thread(
            target=self._read_loop, daemon=True, name="TOFThread"
        )
        self._thread.start()
        print(f"[TOF] Started on {self._port} @ {self._baud}")

    def stop(self) -> None:
        self._running = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    # ──────────────────────────────────────────────────────────────────────────
    def _read_loop(self) -> None:
        ser = self._ser
        while self._running:
            try:
                # Sync to frame header
                if ser.read(1) != b"\x57":
                    continue
                if ser.read(1) != b"\x00":
                    continue

                payload = ser.read(14)
                if len(payload) != 14:
                    continue

                full_frame = [0x57, 0x00] + list(payload)

                # Verify checksum
                if (sum(full_frame[:15]) & 0xFF) != full_frame[15]:
                    continue

                # Distance in mm — bytes 8, 9, 10 (little-endian 24-bit)
                dist = full_frame[8] | (full_frame[9] << 8) | (full_frame[10] << 16)

                with self._lock:
                    self._distance = dist

            except Exception:
                pass

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def distance_mm(self) -> int:
        with self._lock:
            return self._distance

    @property
    def available(self) -> bool:
        return self._available


# ──────────────────────────────────────────────────────────────────────────────

class UltrasonicSensor:
    """
    HC-SR04 ultrasonic sensor via gpiozero.DistanceSensor.

    Parameters
    ----------
    echo_pin : int
        GPIO BCM pin connected to the Echo pin.
    trigger_pin : int
        GPIO BCM pin connected to the Trigger pin.
    """

    def __init__(self, echo_pin: int = 24, trigger_pin: int = 23,
                 max_distance_m: float = 4.0):
        self._echo_pin      = echo_pin
        self._trigger_pin   = trigger_pin
        self._max_dist      = max_distance_m
        self._sensor        = None
        self._available     = False

    def start(self) -> None:
        try:
            from gpiozero import DistanceSensor  # type: ignore
            self._sensor    = DistanceSensor(
                echo=self._echo_pin,
                trigger=self._trigger_pin,
                queue_len=3,
                max_distance=self._max_dist,
            )
            self._available = True
            print(f"[SONAR] Started — echo={self._echo_pin}, trigger={self._trigger_pin}, "
                  f"max={self._max_dist*1000:.0f}mm")
        except Exception as exc:
            print(f"[SONAR] Not available ({exc}) — distance will read 0 mm")
            self._available = False

    def stop(self) -> None:
        if self._sensor:
            try:
                self._sensor.close()
            except Exception:
                pass

    @property
    def distance_mm(self) -> int:
        if self._sensor and self._available:
            try:
                return int(self._sensor.distance * 1000)
            except Exception:
                return 0
        return 0

    @property
    def available(self) -> bool:
        return self._available


# ──────────────────────────────────────────────────────────────────────────────

class LaserController:
    """
    Laser / weapon output via MOSFET on a GPIO pin.

    Uses gpiozero.OutputDevice. Falls back to console mock on non-Pi.

    Parameters
    ----------
    pin : int
        GPIO BCM pin driving the MOSFET gate (default 27).
    """

    def __init__(self, pin: int = 27):
        self._pin       = pin
        self._device    = None
        self._active    = False
        self._available = False

    def start(self) -> None:
        try:
            from gpiozero import OutputDevice  # type: ignore
            self._device    = OutputDevice(self._pin, active_high=True, initial_value=False)
            self._available = True
            print(f"[LASER] Initialized on GPIO {self._pin}")
        except Exception as exc:
            print(f"[LASER] Not available ({exc}) — mock mode")

    def on(self) -> None:
        if self._active:
            return
        self._active = True
        if self._device:
            self._device.on()
        else:
            print("[LASER MOCK] FIRE")

    def off(self) -> None:
        if not self._active:
            return
        self._active = False
        if self._device:
            self._device.off()
        else:
            print("[LASER MOCK] SAFE")

    def stop(self) -> None:
        """Always ensure laser is off on shutdown."""
        self.off()
        if self._device:
            try:
                self._device.close()
            except Exception:
                pass

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def available(self) -> bool:
        return self._available
