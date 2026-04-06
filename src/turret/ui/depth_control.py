"""
depth_control.py — Floating OpenCV depth threshold + on/off control panel.

Dual mode:
  mode="mm"  — threshold and bar in millimetres (for TOF / sonar)
                engage when distance <= threshold (target inside range)
  mode="pct" — threshold and bar in 0–100 % (for bbox proxy)
                engage when closeness >= threshold

Cross-platform / headless safe — all cv2 display calls wrapped in try/except.
"""

from __future__ import annotations
import cv2
import numpy as np


_W, _H    = 360, 170
_TRACKBAR = "Threshold"
_WIN      = "RAKSHAQ — Depth Control"


class DepthControlUI:
    """
    Floating OpenCV depth-threshold control panel.

    Parameters
    ----------
    initial_threshold : int
        Starting slider position (mm or %).
    initial_enabled : bool
        Whether depth module starts enabled.
    mode : str
        "mm" (distance in mm, engage when value <= threshold)
        "pct" (closeness %, engage when value >= threshold)
    max_value : int
        Trackbar maximum (e.g. 5000 for mm, 100 for %).
    """

    def __init__(
        self,
        initial_threshold: int  = 40,
        initial_enabled:   bool = True,
        mode:              str  = "pct",
        max_value:         int  = 100,
    ):
        self._threshold = max(0, min(initial_threshold, max_value))
        self._enabled   = initial_enabled
        self._mode      = mode.lower()          # "mm" or "pct"
        self._max_val   = max_value
        self._current   = 0.0
        self._open      = False
        self._headless  = False

    # ──────────────────────────────────────────────────────────────────────────
    def open(self) -> None:
        try:
            cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(_WIN, _W, _H)
            cv2.createTrackbar(_TRACKBAR, _WIN, self._threshold,
                               self._max_val, self._on_trackbar)
            self._open     = True
            self._headless = False
            self._render()
        except Exception as exc:
            unit = "mm" if self._mode == "mm" else "%"
            print(f"[DEPTH UI] Could not open window ({exc}). "
                  f"Using config threshold={self._threshold}{unit}.")
            self._headless = True

    def close(self) -> None:
        if self._open:
            try:
                cv2.destroyWindow(_WIN)
            except Exception:
                pass
            self._open = False

    # ──────────────────────────────────────────────────────────────────────────
    def update(self, current_value: float) -> None:
        """Refresh panel with latest sensor/bbox reading."""
        if not self._open or self._headless:
            return
        self._current = float(current_value)
        try:
            self._threshold = cv2.getTrackbarPos(_TRACKBAR, _WIN)
        except Exception:
            pass
        self._render()

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        if self._open and not self._headless:
            self._render()

    # ──────────────────────────────────────────────────────────────────────────
    def _triggered(self) -> bool:
        """True if depth condition met based on mode."""
        if self._mode == "mm":
            return 0 < self._current <= self._threshold
        else:
            return self._current >= self._threshold

    def _on_trackbar(self, val: int) -> None:
        self._threshold = val

    def _render(self) -> None:
        canvas = np.zeros((_H, _W, 3), dtype=np.uint8)
        canvas[:] = (25, 25, 25)
        font = cv2.FONT_HERSHEY_SIMPLEX

        unit = "mm" if self._mode == "mm" else "%"

        # Header
        cv2.putText(canvas, "DEPTH THRESHOLD CONTROL", (12, 22),
                    font, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

        # Enabled badge
        badge_txt = "ACTIVE" if self._enabled else "DISABLED"
        badge_col = (0, 200, 80) if self._enabled else (60, 60, 60)
        cv2.putText(canvas, badge_txt, (_W - 82, 22),
                    font, 0.45, badge_col, 1, cv2.LINE_AA)
        cv2.putText(canvas, "[d] toggle", (_W - 82, 36),
                    font, 0.32, (100, 100, 100), 1, cv2.LINE_AA)

        if not self._enabled:
            cv2.putText(canvas, "Depth trigger OFF", (12, 100),
                        font, 0.55, (80, 80, 80), 1, cv2.LINE_AA)
            self._show(canvas)
            return

        triggered = self._triggered()
        trig_col  = (0, 80, 255) if triggered else (0, 200, 100)

        # Readings
        cv2.putText(canvas, f"Threshold: {self._threshold}{unit}", (12, 54),
                    font, 0.44, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Reading:   {self._current:.0f}{unit}", (12, 72),
                    font, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

        status = ">>> ENGAGE <<<" if triggered else "MONITOR"
        cv2.putText(canvas, status, (200, 54),
                    font, 0.50, trig_col, 1, cv2.LINE_AA)

        # Bar
        bx0, bx1 = 12, _W - 12
        by0, by1 = 92, 120
        bw = bx1 - bx0

        cv2.rectangle(canvas, (bx0, by0), (bx1, by1), (55, 55, 55), -1)

        # Fill: mm mode → bar = current / max_val; pct mode → bar = current / max_val
        fill_ratio = min(self._current / max(self._max_val, 1), 1.0)
        fill_w     = int(bw * fill_ratio)
        if fill_w > 0:
            fill_col = trig_col
            cv2.rectangle(canvas, (bx0, by0), (bx0 + fill_w, by1), fill_col, -1)

        # Threshold tick
        tick_ratio = min(self._threshold / max(self._max_val, 1), 1.0)
        tick_x     = bx0 + int(bw * tick_ratio)
        cv2.line(canvas, (tick_x, by0 - 4), (tick_x, by1 + 4), (0, 220, 255), 2)

        cv2.rectangle(canvas, (bx0, by0), (bx1, by1), (110, 110, 110), 1)

        # Scale labels
        ly = by1 + 16
        cv2.putText(canvas, "0",
                    (bx0, ly), font, 0.35, (100, 100, 100), 1)
        cv2.putText(canvas, f"{self._max_val // 2}{unit}",
                    (bx0 + bw // 2 - 16, ly), font, 0.35, (100, 100, 100), 1)
        cv2.putText(canvas, f"{self._max_val}{unit}",
                    (bx1 - 32, ly), font, 0.35, (100, 100, 100), 1)

        self._show(canvas)

    def _show(self, canvas) -> None:
        try:
            cv2.imshow(_WIN, canvas)
            cv2.waitKey(1)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def threshold(self) -> int:
        if self._open and not self._headless:
            try:
                return cv2.getTrackbarPos(_TRACKBAR, _WIN)
            except Exception:
                pass
        return self._threshold

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def is_open(self) -> bool:
        return self._open and not self._headless
