"""
axis_mapper.py — Rotation-aware pixel-error → physical-degree conversion.

When the camera is mounted sideways and the image is software-rotated,
the pixel axes in the displayed frame no longer align with the physical
pan/tilt motor axes.  This module un-rotates the error vector so the
motors always move in the correct direction.

cv2 rotation transforms (W×H original → rotated image):
    90CW:   (x, y) → (H-1-y,   x)      inverse: sensor_X =  screen_Y,  sensor_Y = -screen_X
    90CCW:  (x, y) → (y,       W-1-x)   inverse: sensor_X = -screen_Y,  sensor_Y =  screen_X
    180:    (x, y) → (W-1-x,   H-1-y)   inverse: sensor_X = -screen_X,  sensor_Y = -screen_Y

Terminology:
    screen_X / screen_Y  → pixel axes in the POST-rotation displayed image
    sensor_X / sensor_Y  → pixel axes on the PHYSICAL camera sensor
    pan  = physical horizontal rotation (positive = right)
    tilt = physical vertical rotation   (positive = up, per STM32 convention)
"""

from __future__ import annotations
import math


class AxisMapper:
    """
    Convert screen pixel offsets to physical (pan, tilt) degrees.

    Parameters
    ----------
    hfov_deg : float
        Horizontal FOV of the *displayed* (post-rotation) image.
    vfov_deg : float
        Vertical FOV of the *displayed* (post-rotation) image.
    camera_rotate : str
        "none" | "90cw" | "90ccw" | "180"
    pan_invert : bool
        Flip pan sign (for reversed motor wiring).
    tilt_invert : bool
        Flip tilt sign (for reversed motor wiring).
    """

    def __init__(
        self,
        hfov_deg: float      = 66.0,
        vfov_deg: float      = 52.0,
        camera_rotate: str   = "none",
        pan_invert: bool     = False,
        tilt_invert: bool    = False,
    ):
        self._hfov = hfov_deg
        self._vfov = vfov_deg
        self._rot  = camera_rotate.lower()
        self._pan_sign  = -1 if pan_invert  else 1
        self._tilt_sign = -1 if tilt_invert else 1

    # ──────────────────────────────────────────────────────────────────────────
    def to_degrees(
        self,
        error_x: float,
        error_y: float,
        frame_w: int,
        frame_h: int,
    ) -> tuple[float, float]:
        """
        Convert post-rotation pixel error to physical (pan_deg, tilt_deg).

        Parameters
        ----------
        error_x, error_y : float
            Pixel offset from the aim point.
            Positive error_x = target is to the RIGHT  of aim point.
            Positive error_y = target is BELOW the aim point.
        frame_w, frame_h : int
            Dimensions of the post-rotation frame (pixels).

        Returns
        -------
        (pan_deg, tilt_deg) : tuple[float, float]
            Physical motor degrees.
            pan  positive = should rotate right.
            tilt positive = should rotate up.
        """
        rot = self._rot

        if rot == "90cw":
            # sensor_X = screen_Y,  sensor_Y = -screen_X
            pan_deg  = (error_y  / frame_h) * self._vfov
            tilt_deg = (-error_x / frame_w) * self._hfov
        elif rot == "90ccw":
            # sensor_X = -screen_Y, sensor_Y = screen_X
            pan_deg  = (-error_y / frame_h) * self._vfov
            tilt_deg = (error_x  / frame_w) * self._hfov
        elif rot == "180":
            # sensor_X = -screen_X, sensor_Y = -screen_Y
            pan_deg  = (-error_x / frame_w) * self._hfov
            tilt_deg = (-error_y / frame_h) * self._vfov
        else:
            pan_deg  = (error_x / frame_w) * self._hfov
            tilt_deg = (error_y / frame_h) * self._vfov

        # Tilt convention: positive error_y = target below → need to tilt
        # DOWN.  STM32 convention: positive Y = UP.  So negate.
        tilt_deg = -tilt_deg

        # Apply wiring inversion
        pan_deg  *= self._pan_sign
        tilt_deg *= self._tilt_sign

        return (pan_deg, tilt_deg)
