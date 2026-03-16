"""
visualizer.py — Draws detection overlays and HUD onto frames.

Provides a Visualizer class that can be called with a frame + detections
and returns the annotated frame.  Keeps a rolling FPS counter internally.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Sequence

import cv2
import numpy as np

from src.turret.detection.detector import Detection


# ──────────────────────────────────────────────────────────────────────────────
# Small helpers
# ──────────────────────────────────────────────────────────────────────────────

def _text_size(text: str, font_scale: float, thickness: int) -> tuple[int, int]:
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
    return w, h


def _put_text_with_bg(
    frame: np.ndarray,
    text: str,
    origin: tuple[int, int],
    font_scale: float,
    text_color: tuple[int, int, int],
    bg_color: tuple[int, int, int],
    thickness: int = 1,
    padding: int = 4,
) -> None:
    """Draw text with a solid background rectangle."""
    x, y      = origin
    tw, th    = _text_size(text, font_scale, thickness)
    cv2.rectangle(
        frame,
        (x - padding, y - th - padding),
        (x + tw + padding, y + padding),
        bg_color, -1,
    )
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                font_scale, text_color, thickness, cv2.LINE_AA)


# ──────────────────────────────────────────────────────────────────────────────

class Visualizer:
    """
    Annotates frames with bounding boxes, labels, confidence scores,
    detection count, and a rolling FPS counter.

    Parameters
    ----------
    class_colors : dict[str, tuple[int,int,int]]
        BGR colour per class name.
    default_color : tuple[int,int,int]
        Fallback BGR colour for unknown classes.
    show_fps : bool
        Whether to render FPS in the top-left corner.
    show_labels : bool
        Whether to render class labels above boxes.
    show_conf : bool
        Whether to append confidence score to the label.
    fps_history : int
        Number of recent frame times to average for FPS.
    """

    def __init__(
        self,
        class_colors:  dict[str, tuple[int, int, int]] | None = None,
        default_color: tuple[int, int, int]                   = (0, 165, 255),
        show_fps:      bool                                    = True,
        show_labels:   bool                                    = True,
        show_conf:     bool                                    = True,
        fps_history:   int                                     = 30,
    ):
        self._colors       = class_colors or {}
        self._default_color = default_color
        self.show_fps      = show_fps
        self.show_labels   = show_labels
        self.show_conf     = show_conf

        self._timestamps: deque[float] = deque(maxlen=fps_history)
        self._last_fps: float = 0.0

    # ──────────────────────────────────────────────────────────────────────────
    def _tick(self) -> float:
        """Record a frame timestamp and return current FPS."""
        now = time.perf_counter()
        self._timestamps.append(now)
        if len(self._timestamps) >= 2:
            elapsed      = self._timestamps[-1] - self._timestamps[0]
            self._last_fps = (len(self._timestamps) - 1) / elapsed if elapsed > 0 else 0.0
        return self._last_fps

    def _color_for(self, class_name: str) -> tuple[int, int, int]:
        return self._colors.get(class_name, self._default_color)

    # ──────────────────────────────────────────────────────────────────────────
    def draw(
        self,
        frame:      np.ndarray,
        detections: Sequence[Detection],
    ) -> np.ndarray:
        """
        Annotate *frame* in-place and return it.

        Parameters
        ----------
        frame : np.ndarray
            BGR frame.
        detections : list[Detection]
            Detections returned by Detector.detect().
        """
        fps = self._tick()
        out = frame  # annotate in-place (caller can .copy() first if needed)

        # ── Bounding boxes + labels ──────────────────────────────────────────
        for det in detections:
            color     = self._color_for(det.class_name)
            box_thick = 2

            # Box
            cv2.rectangle(out, (det.x1, det.y1), (det.x2, det.y2), color, box_thick)

            # Crosshair at centre
            cx, cy = det.center
            arm = 8
            cv2.line(out, (cx - arm, cy), (cx + arm, cy), color, 1, cv2.LINE_AA)
            cv2.line(out, (cx, cy - arm), (cx, cy + arm), color, 1, cv2.LINE_AA)

            # Label
            if self.show_labels:
                label = det.class_name
                if self.show_conf:
                    label += f" {det.confidence:.0%}"
                _put_text_with_bg(
                    out, label,
                    origin=(det.x1, det.y1 - 6),
                    font_scale=0.55, thickness=1,
                    text_color=(255, 255, 255),
                    bg_color=color,
                )

        # ── HUD ─────────────────────────────────────────────────────────────
        h, w = out.shape[:2]

        # FPS — top-left
        if self.show_fps:
            _put_text_with_bg(
                out, f"FPS {fps:5.1f}",
                origin=(10, 26),
                font_scale=0.6, thickness=1,
                text_color=(255, 255, 255),
                bg_color=(30, 30, 30),
            )

        # Detection count — top-right
        count_text = f"Targets: {len(detections)}"
        tw, _ = _text_size(count_text, 0.6, 1)
        _put_text_with_bg(
            out, count_text,
            origin=(w - tw - 16, 26),
            font_scale=0.6, thickness=1,
            text_color=(255, 255, 255),
            bg_color=(30, 30, 30),
        )

        return out
