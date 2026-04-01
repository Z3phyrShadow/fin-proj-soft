"""
depth_control.py — Floating OpenCV depth threshold + on/off control panel.

Cross-platform: works on Windows and Raspberry Pi OS with a display.
Falls back gracefully to config values if no display is available (headless Pi).

Controls rendered:
  • ON/OFF toggle via 'd' key in the main window (state shown here too)
  • Trackbar — depth trigger threshold 0–100
  • Live depth bar — green (safe) / red (triggered)
  • Threshold tick line on the bar
"""

from __future__ import annotations
import cv2
import numpy as np


_W, _H    = 340, 160
_TRACKBAR = "Depth Threshold"
_WIN      = "RAKSHAQ — Depth Control"


class DepthControlUI:
    """
    Floating OpenCV depth-threshold control panel.

    If the display is unavailable (headless / missing Qt backend), the window
    silently fails to open and `.threshold` returns the initial config value.

    Parameters
    ----------
    initial_threshold : int
        Starting slider position (0–100).
    initial_enabled : bool
        Whether depth module starts enabled.
    """

    def __init__(self, initial_threshold: int = 40, initial_enabled: bool = True):
        self._threshold = max(0, min(100, initial_threshold))
        self._enabled   = initial_enabled
        self._current_depth: float = 0.0
        self._open = False
        self._headless = False   # True if display unavailable

    # ──────────────────────────────────────────────────────────────────────────
    def open(self) -> None:
        """Create the window and trackbar. Silently skips if headless."""
        try:
            cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(_WIN, _W, _H)
            cv2.createTrackbar(_TRACKBAR, _WIN, self._threshold, 100,
                               self._on_trackbar)
            self._open     = True
            self._headless = False
            self._render()
        except Exception as e:
            print(f"[DEPTH UI] Could not open control window ({e}). "
                  f"Using config threshold={self._threshold}%.")
            self._headless = True

    def close(self) -> None:
        """Destroy the control window."""
        if self._open:
            try:
                cv2.destroyWindow(_WIN)
            except Exception:
                pass
            self._open = False

    # ──────────────────────────────────────────────────────────────────────────
    def update(self, current_depth: float) -> None:
        """Refresh the panel with the latest depth reading."""
        if not self._open or self._headless:
            return
        self._current_depth = max(0.0, min(100.0, current_depth))
        try:
            self._threshold = cv2.getTrackbarPos(_TRACKBAR, _WIN)
        except Exception:
            pass
        self._render()

    def set_enabled(self, enabled: bool) -> None:
        """Toggle the depth module on/off — reflects in the panel."""
        self._enabled = enabled
        if self._open and not self._headless:
            self._render()

    # ──────────────────────────────────────────────────────────────────────────
    def _on_trackbar(self, val: int) -> None:
        self._threshold = val

    def _render(self) -> None:
        """Draw the full panel onto the window canvas."""
        canvas = np.zeros((_H, _W, 3), dtype=np.uint8)
        canvas[:] = (25, 25, 25)

        font = cv2.FONT_HERSHEY_SIMPLEX

        # ── Header ────────────────────────────────────────────────────────────
        cv2.putText(canvas, "DEPTH THRESHOLD CONTROL", (12, 22),
                    font, 0.48, (200, 200, 200), 1, cv2.LINE_AA)

        # ── Enabled / Disabled badge ──────────────────────────────────────────
        if self._enabled:
            badge_txt, badge_col = "ACTIVE", (0, 200, 80)
        else:
            badge_txt, badge_col = "DISABLED", (60, 60, 60)

        # Status (top-right)
        cv2.putText(canvas, badge_txt, (_W - 80, 22),
                    font, 0.45, badge_col, 1, cv2.LINE_AA)
        cv2.putText(canvas, "[d] toggle", (_W - 82, 36),
                    font, 0.32, (100, 100, 100), 1, cv2.LINE_AA)

        if not self._enabled:
            # Grey out rest of panel
            cv2.putText(canvas, "Depth trigger OFF", (12, 95),
                        font, 0.55, (80, 80, 80), 1, cv2.LINE_AA)
            try:
                cv2.imshow(_WIN, canvas)
                cv2.waitKey(1)
            except Exception:
                pass
            return

        # ── Threshold / depth text ────────────────────────────────────────────
        triggered = self._current_depth >= self._threshold
        triggered_col = (0, 80, 255) if triggered else (0, 200, 100)

        cv2.putText(canvas, f"Threshold: {self._threshold}%", (12, 52),
                    font, 0.44, (0, 220, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"Depth: {self._current_depth:.1f}%", (180, 52),
                    font, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

        status_txt = ">>> ENGAGE <<<" if triggered else "MONITOR"
        cv2.putText(canvas, status_txt, (12, 72),
                    font, 0.5, triggered_col, 1, cv2.LINE_AA)

        # ── Depth bar ─────────────────────────────────────────────────────────
        bx0, bx1 = 12, _W - 12
        by0, by1 = 88, 116
        bw = bx1 - bx0

        cv2.rectangle(canvas, (bx0, by0), (bx1, by1), (55, 55, 55), -1)

        fill_w = int(bw * self._current_depth / 100.0)
        if fill_w > 0:
            cv2.rectangle(canvas, (bx0, by0),
                          (bx0 + fill_w, by1), triggered_col, -1)

        # Threshold tick
        tick_x = bx0 + int(bw * self._threshold / 100.0)
        cv2.line(canvas, (tick_x, by0 - 4), (tick_x, by1 + 4), (0, 220, 255), 2)

        cv2.rectangle(canvas, (bx0, by0), (bx1, by1), (110, 110, 110), 1)

        # Scale labels
        ly = by1 + 16
        cv2.putText(canvas, "0",   (bx0, ly),          font, 0.36, (100, 100, 100), 1)
        cv2.putText(canvas, "50",  (bx0 + bw//2 - 6, ly), font, 0.36, (100, 100, 100), 1)
        cv2.putText(canvas, "100", (bx1 - 24, ly),     font, 0.36, (100, 100, 100), 1)

        try:
            cv2.imshow(_WIN, canvas)
            cv2.waitKey(1)
        except Exception:
            pass

    # ──────────────────────────────────────────────────────────────────────────
    @property
    def threshold(self) -> int:
        """Current threshold from the trackbar (0–100)."""
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
