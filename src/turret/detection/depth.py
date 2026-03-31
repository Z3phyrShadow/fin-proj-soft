"""
depth.py — Target depth estimation.

Currently supports two backends:
  "bbox"  — bounding-box height as a proximity proxy (0–100, higher = closer)
  "ir"    — placeholder for an IR laser range sensor (returns raw cm value,
             normalised to the same 0–100 scale using a configurable max range)

Swap backends in config.py:  DEPTH_BACKEND = "ir"
"""

from __future__ import annotations


class DepthEstimator:
    """
    Estimates target depth (proximity) for a single Detection.

    Parameters
    ----------
    backend : str
        "bbox" | "ir"
    ir_max_cm : float
        IR-sensor max range in cm used for normalisation (ir backend only).
    """

    def __init__(self, backend: str = "bbox", ir_max_cm: float = 500.0):
        self._backend  = backend.lower()
        self._ir_max   = ir_max_cm
        self._ir_value = 0.0  # updated by set_ir_reading()

        if self._backend not in ("bbox", "ir"):
            raise ValueError(f"Unknown depth backend: '{backend}'. Use 'bbox' or 'ir'.")

        print(f"[DEPTH] Backend: {self._backend}")

    # ──────────────────────────────────────────────────────────────────────────
    def estimate(self, detection, frame_height: int) -> float:
        """
        Return proximity as a percentage 0–100 (100 = closest / fills frame).

        Parameters
        ----------
        detection : Detection
            A Detection object (from detector.py).
        frame_height : int
            Height of the current frame in pixels.

        Returns
        -------
        float
            0–100 proximity score.
        """
        if self._backend == "bbox":
            return self._bbox_depth(detection, frame_height)
        elif self._backend == "ir":
            return self._ir_depth()
        return 0.0

    # ── Backends ──────────────────────────────────────────────────────────────

    @staticmethod
    def _bbox_depth(detection, frame_height: int) -> float:
        """Proximity ∝ bbox height / frame height."""
        if frame_height <= 0:
            return 0.0
        raw = detection.height / frame_height
        return min(max(raw * 100.0, 0.0), 100.0)

    def _ir_depth(self) -> float:
        """
        Normalise raw IR cm reading to 0–100.
        100 = target at 0 cm (touching sensor), 0 = at or beyond ir_max_cm.
        """
        normalised = 1.0 - min(self._ir_value / self._ir_max, 1.0)
        return normalised * 100.0

    # ──────────────────────────────────────────────────────────────────────────
    def set_ir_reading(self, cm: float) -> None:
        """
        Feed a raw IR sensor reading (cm) into the estimator.
        Call this each frame from your sensor driver thread.
        """
        self._ir_value = max(0.0, cm)

    @property
    def backend(self) -> str:
        return self._backend
