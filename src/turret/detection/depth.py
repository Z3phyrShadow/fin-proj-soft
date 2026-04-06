"""
depth.py — Target distance estimation.

Backends:
  "bbox"       — bounding-box height proxy (0–100 %, Windows testing)
  "tof"        — TOF laser sensor via TOFSensor (mm, production)
  "ultrasonic" — HC-SR04 sonar via UltrasonicSensor (mm, production)

The TOF and ultrasonic backends return distance in mm (closer = smaller value).
The bbox backend returns a closeness percentage (closer = larger value).

Use .uses_mm to tell the calling code which unit system is active.
"""

from __future__ import annotations


class DepthEstimator:
    """
    Estimates target distance.

    Parameters
    ----------
    backend : str
        "bbox" | "tof" | "ultrasonic"
    tof_sensor : TOFSensor | None
        Required when backend == "tof".
    sonar_sensor : UltrasonicSensor | None
        Required when backend == "ultrasonic".
    max_mm : float
        Maximum expected sensor range in mm (used for bbox normalisation only).
    """

    def __init__(
        self,
        backend:      str   = "bbox",
        tof_sensor          = None,
        sonar_sensor        = None,
        max_mm:       float = 5000.0,
    ):
        self._backend = backend.lower()
        self._tof     = tof_sensor
        self._sonar   = sonar_sensor
        self._max_mm  = max_mm

        valid = ("bbox", "tof", "ultrasonic")
        if self._backend not in valid:
            raise ValueError(f"Unknown depth backend '{backend}'. Use: {valid}")

        print(f"[DEPTH] Backend: {self._backend}  |  units: {'mm' if self.uses_mm else '%'}")

    # ──────────────────────────────────────────────────────────────────────────
    def estimate(self, detection=None, frame_height: int = 480):
        """
        Return current depth/distance reading.

        Returns
        -------
        float | int
            mm  when uses_mm is True  (lower = closer, 0 = no data)
            %   when uses_mm is False (higher = closer, 0–100)
        """
        if self._backend == "tof":
            return self._tof.distance_mm if self._tof else 0

        if self._backend == "ultrasonic":
            return self._sonar.distance_mm if self._sonar else 0

        # bbox fallback
        if detection is None:
            return 0.0
        return self._bbox_pct(detection, frame_height)

    # ── Helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _bbox_pct(detection, frame_height: int) -> float:
        if frame_height <= 0:
            return 0.0
        return min(detection.height / frame_height * 100.0, 100.0)

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def uses_mm(self) -> bool:
        """True for tof/ultrasonic (mm), False for bbox (%)."""
        return self._backend in ("tof", "ultrasonic")

    @property
    def backend(self) -> str:
        return self._backend
