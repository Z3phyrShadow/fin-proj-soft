"""
parallax.py — Camera-to-barrel offset compensation.

The LED barrel is physically offset from the camera:
    • 13 cm to the right
    •  4 cm above
    • 14 cm forward  (mounted on turret tip)

At finite distances this means the barrel aim point and the camera
centre do not coincide.  This module computes the angular correction
(in physical pan/tilt degrees) so the *barrel* (not the camera) is
aimed at the target.

The correction is subtracted from the raw tracking error:

    corrected_pan  = raw_pan  - parallax_pan
    corrected_tilt = raw_tilt - parallax_tilt

When the target is at the corrected aim point, the barrel is on target
and the corrected error is zero — no motor movement.
"""

from __future__ import annotations
import math


class ParallaxCompensator:
    """
    Compute pan/tilt parallax correction in physical degrees.

    Parameters
    ----------
    barrel_right_cm : float
        Barrel offset to the RIGHT of the camera (cm).
    barrel_up_cm : float
        Barrel offset ABOVE the camera (cm).
    barrel_forward_cm : float
        Barrel offset in FRONT of the camera (cm).
    default_distance_mm : float
        Assumed distance when the ToF reads 0 (no data).
    """

    def __init__(
        self,
        barrel_right_cm: float   = 13.0,
        barrel_up_cm: float      =  4.0,
        barrel_forward_cm: float = 14.0,
        default_distance_mm: float = 3000.0,
    ):
        # Store in mm for consistent units
        self._right_mm   = barrel_right_cm   * 10.0
        self._up_mm      = barrel_up_cm      * 10.0
        self._forward_mm = barrel_forward_cm * 10.0
        self._default_mm = default_distance_mm

    # ──────────────────────────────────────────────────────────────────────────
    def correction(self, distance_mm: float) -> tuple[float, float]:
        """
        Return (pan_correction_deg, tilt_correction_deg).

        Subtract these from the raw tracking error to compensate for
        the barrel offset.

        Parameters
        ----------
        distance_mm : float
            Target distance from the ToF sensor in mm.
            If ≤ 0, falls back to default_distance_mm.

        Returns
        -------
        (pan_corr, tilt_corr) : tuple[float, float]
            pan_corr  > 0 means the target appears to the right of
                        camera centre when the barrel is on target.
            tilt_corr > 0 means the barrel is aimed higher, so the
                        target appears below camera centre.
        """
        d = distance_mm if distance_mm > 0 else self._default_mm

        # Effective distance along the aiming axis (barrel is forward of
        # camera, so the barrel-to-target distance is shorter).
        d_eff = max(d - self._forward_mm, 100.0)  # floor at 10 cm

        pan_corr  = math.degrees(math.atan2(self._right_mm, d_eff))
        tilt_corr = math.degrees(math.atan2(self._up_mm,    d_eff))

        return (pan_corr, tilt_corr)

    # ──────────────────────────────────────────────────────────────────────────
    def correct(
        self,
        pan_deg: float,
        tilt_deg: float,
        distance_mm: float,
    ) -> tuple[float, float]:
        """
        Apply parallax correction to raw tracking degrees.

        Parameters
        ----------
        pan_deg, tilt_deg : float
            Raw tracking error in physical pan/tilt degrees
            (positive pan = target to the right, positive tilt = should
            tilt up).
        distance_mm : float
            ToF distance in mm.

        Returns
        -------
        (corrected_pan, corrected_tilt) : tuple[float, float]
        """
        pc, tc = self.correction(distance_mm)

        # Barrel is to the RIGHT → when barrel is on target, the target
        # appears to the RIGHT of camera centre by pc degrees.
        # The raw error already says "target is X deg to the right".
        # We subtract pc so that the controller rests when the target
        # is at the barrel-aim point (pc deg to the right), not at the
        # camera centre.
        corrected_pan  = pan_deg  - pc

        # Barrel is ABOVE camera → when barrel is on target, the target
        # appears BELOW camera centre (because we tilted down to
        # compensate up-offset).  tilt_deg positive = need to tilt up.
        # Subtracting tc reduces the tilt-up command, effectively
        # tilting down.
        corrected_tilt = tilt_deg - tc

        return (corrected_pan, corrected_tilt)
