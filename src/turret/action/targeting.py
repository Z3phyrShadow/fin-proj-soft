"""
targeting.py — Target selection with closest-to-centre strategy.

Selects the person closest to frame centre, with hysteresis to prevent
flip-flopping between two equidistant targets.
"""

from __future__ import annotations

import math
from typing import List, Optional


class TargetSelector:
    """
    Selects the highest priority target from YOLO detections.

    Strategy: always pick the person closest to frame centre.
    Hysteresis: only switch to a different target if it is closer
    to centre by at least *switch_hysteresis_px* pixels.

    Parameters
    ----------
    min_confidence : float
        Minimum confidence to consider a detection.
    switch_hysteresis_px : int
        A new target must be this many pixels closer to centre
        before we switch away from the current one.
    """

    def __init__(
        self,
        min_confidence: float     = 0.40,
        switch_hysteresis_px: int = 40,
        **_kwargs,  # accept and ignore extra kwargs for compat
    ):
        self.min_confidence      = min_confidence
        self.switch_hysteresis   = switch_hysteresis_px
        self.frame_width:  int   = 480
        self.frame_height: int   = 360

        self._current_track_id: int | None = None

        print(f"[TARGETING] Strategy: closest-to-centre  "
              f"hysteresis={switch_hysteresis_px}px  "
              f"min_conf={min_confidence}")

    # ──────────────────────────────────────────────────────────────────────────
    def update_frame_size(self, width: int, height: int) -> None:
        self.frame_width  = width
        self.frame_height = height

    # ──────────────────────────────────────────────────────────────────────────
    def select_target(self, detections: list) -> Optional:
        """
        Pick the best target from *detections*.

        Returns the Detection closest to frame centre, with hysteresis
        to avoid flip-flopping.  Returns None if no valid detections.
        """
        if not detections:
            self._current_track_id = None
            return None

        # Filter by confidence
        valid = [d for d in detections if d.confidence >= self.min_confidence]
        if not valid:
            self._current_track_id = None
            return None

        cx_frame = self.frame_width  / 2.0
        cy_frame = self.frame_height / 2.0

        def _dist(det):
            cx, cy = det.center
            return math.hypot(cx - cx_frame, cy - cy_frame)

        # Find the absolute closest target
        closest = min(valid, key=_dist)

        # Hysteresis: if we're already tracking someone, keep them
        # unless the new closest is significantly better.
        if self._current_track_id is not None:
            current = self._find_by_track_id(valid, self._current_track_id)
            if current is not None:
                dist_current = _dist(current)
                dist_closest = _dist(closest)
                # Only switch if the new one is closer by the hysteresis margin
                if dist_current - dist_closest < self.switch_hysteresis:
                    return current
                # else: switch to the new closest below

        # Update tracked ID
        self._current_track_id = closest.track_id
        return closest

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _find_by_track_id(detections: list, track_id: int):
        """Find a detection with the given ByteTrack ID, or None."""
        for d in detections:
            if d.track_id == track_id:
                return d
        return None

    @property
    def current_track_id(self) -> int | None:
        return self._current_track_id