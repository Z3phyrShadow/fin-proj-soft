"""
axis_mapper.py — Rotation-aware pixel-error → physical-degree conversion.

When the camera is mounted sideways and the image is software-rotated,
the pixel axes in the displayed frame no longer align with the physical
pan/tilt motor axes.  This module un-rotates the error vector so the
motors always move in the correct direction.

FOV convention (IMPORTANT):
    CAMERA_HFOV_DEG / CAMERA_VFOV_DEG in config are the PHYSICAL sensor
    FOV values *before* any software rotation (landscape orientation).
    This module swaps them when a 90° rotation is active so that:
        pan  always uses the FOV that spans the displayed horizontal axis
        tilt always uses the FOV that spans the displayed vertical axis

Angular projection:
    Uses the accurate pinhole / arctangent model:
        angle = atan( pixel_error × tan(fov/2) / (frame_half_size) )
    which is exact for any rectilinear lens, unlike the linear
    approximation (error/frame × fov) that underestimates by ~7–15%.

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

        # When the camera is mounted sideways (90CW/90CCW) and software-rotated,
        # the physical sensor HFOV spans the displayed vertical axis and VFOV
        # spans the displayed horizontal axis — so we swap them.
        # screen_X is always pan (left/right) and screen_Y is always tilt (up/down)
        # in the final post-rotation displayed image.
        if rot in ("90cw", "90ccw"):
            eff_hfov = self._vfov   # physical VFOV → displayed horizontal span
            eff_vfov = self._hfov   # physical HFOV → displayed vertical span
        else:
            eff_hfov = self._hfov
            eff_vfov = self._vfov

        # ── Accurate pinhole / arctangent angular projection ──────────────────
        # Linear approx (error/frame_size * fov) undershoots by 7–15% depending
        # on the error magnitude.  The correct model:
        #   focal_len_px = half_frame / tan(half_fov)
        #   angle        = atan(error / focal_len_px)
        #                = atan(error * tan(half_fov) / half_frame)
        pan_deg  = math.degrees(
            math.atan2(error_x * math.tan(math.radians(eff_hfov / 2.0)),
                       frame_w / 2.0)
        )
        tilt_deg = math.degrees(
            math.atan2(error_y * math.tan(math.radians(eff_vfov / 2.0)),
                       frame_h / 2.0)
        )

        # cv2 Y+ is DOWN. To follow a target going down, we must pitch DOWN.
        # STM32 Y+ is UP. So an error_y > 0 (down) requires a negative tilt degree.
        tilt_deg = -tilt_deg

        # Apply wiring inversion
        pan_deg  *= self._pan_sign
        tilt_deg *= self._tilt_sign

        return (pan_deg, tilt_deg)
