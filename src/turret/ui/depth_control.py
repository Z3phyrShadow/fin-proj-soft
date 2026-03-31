"""
depth_control.py — Floating OpenCV control window for depth threshold.

Creates a small panel with:
  • A trackbar to set the depth trigger threshold (0–100)
  • A live depth bar showing the current target's estimated proximity
  • A text readout of threshold vs current depth

Usage:
    ui = DepthControlUI(initial_threshold=40)
    ui.open()

    # each frame:
    ui.update(current_depth=62.4)      # draws the bar
    threshold = ui.threshold           # read current slider value

    ui.close()
"""

from __future__ import annotations
import cv2
import numpy as np


# Window dimensions
_W, _H = 320, 140
_TRACKBAR = "Depth Threshold"
_WIN      = "RAKSHAQ — Depth Control"


class DepthControlUI:
    """
    Lightweight OpenCV depth-threshold control panel.

    Parameters
    ----------
    initial_threshold : int
        Starting slider position (0–100).
    """

    def __init__(self, initial_threshold: int = 40):
        self._threshold = max(0, min(100, initial_threshold))
        self._current_depth: float = 0.0
        self._open = False

    # ──────────────────────────────────────────────────────────────────────────
    def open(self) -> None:
        """Create the window and trackbar."""
        cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WIN, _W, _H)
        cv2.createTrackbar(_TRACKBAR, _WIN, self._threshold, 100, self._on_trackbar)
        self._open = True
        self._render()

    def close(self) -> None:
        """Destroy the control window."""
        if self._open:
            cv2.destroyWindow(_WIN)
            self._open = False

    # ──────────────────────────────────────────────────────────────────────────
    def update(self, current_depth: float) -> None:
        """
        Refresh the panel with the latest depth reading.

        Parameters
        ----------
        current_depth : float
            Estimated proximity of the current target (0–100).
        """
        if not self._open:
            return
        self._current_depth = max(0.0, min(100.0, current_depth))
        # Re-read slider in case user moved it
        self._threshold = cv2.getTrackbarPos(_TRACKBAR, _WIN)
        self._render()

    # ──────────────────────────────────────────────────────────────────────────
    def _on_trackbar(self, val: int) -> None:
        self._threshold = val

    def _render(self) -> None:
        """Draw the depth bar and labels onto the panel canvas."""
        canvas = np.zeros((_H, _W, 3), dtype=np.uint8)
        canvas[:] = (25, 25, 25)  # dark background

        bar_x0, bar_x1 = 16, _W - 16
        bar_y0, bar_y1 = 72, 100
        bar_w = bar_x1 - bar_x0

        # ── Background bar (grey) ─────────────────────────────────────────────
        cv2.rectangle(canvas, (bar_x0, bar_y0), (bar_x1, bar_y1), (60, 60, 60), -1)

        # ── Depth fill ────────────────────────────────────────────────────────
        fill_w    = int(bar_w * self._current_depth / 100.0)
        fill_x1   = bar_x0 + fill_w
        triggered = self._current_depth >= self._threshold
        fill_color = (0, 80, 255) if triggered else (0, 200, 100)
        if fill_w > 0:
            cv2.rectangle(canvas, (bar_x0, bar_y0), (fill_x1, bar_y1), fill_color, -1)

        # ── Threshold line ────────────────────────────────────────────────────
        thresh_x = bar_x0 + int(bar_w * self._threshold / 100.0)
        cv2.line(canvas, (thresh_x, bar_y0 - 4), (thresh_x, bar_y1 + 4), (0, 220, 255), 2)

        # ── Bar border ────────────────────────────────────────────────────────
        cv2.rectangle(canvas, (bar_x0, bar_y0), (bar_x1, bar_y1), (120, 120, 120), 1)

        # ── Labels ────────────────────────────────────────────────────────────
        font, fs, ft = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1

        cv2.putText(canvas, "DEPTH THRESHOLD CONTROL", (16, 22),
                    font, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

        status_text  = "ENGAGE" if triggered else "MONITOR"
        status_color = (0, 80, 255) if triggered else (0, 200, 100)
        cv2.putText(canvas, status_text, (bar_x1 - 68, 22),
                    font, 0.48, status_color, 1, cv2.LINE_AA)

        cv2.putText(canvas, f"Threshold: {self._threshold}%", (bar_x0, 54),
                    font, fs, (0, 220, 255), ft, cv2.LINE_AA)
        cv2.putText(canvas, f"Depth:     {self._current_depth:.1f}%", (bar_x0 + 150, 54),
                    font, fs, (200, 200, 200), ft, cv2.LINE_AA)

        label_y = bar_y1 + 18
        cv2.putText(canvas, "0", (bar_x0, label_y),
                    font, 0.38, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.putText(canvas, "50", (bar_x0 + bar_w // 2 - 6, label_y),
                    font, 0.38, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.putText(canvas, "100", (bar_x1 - 20, label_y),
                    font, 0.38, (120, 120, 120), 1, cv2.LINE_AA)

        cv2.imshow(_WIN, canvas)
        cv2.waitKey(1)

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def threshold(self) -> int:
        """Current threshold value from the trackbar (0–100)."""
        return self._threshold

    @property
    def is_open(self) -> bool:
        return self._open
