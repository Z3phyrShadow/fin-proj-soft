"""
Tracking package — PID-based turret tracking with parallax compensation.
"""

from .tracker import Tracker, TrackState
from .pid import PID
from .axis_mapper import AxisMapper
from .parallax import ParallaxCompensator

__all__ = [
    "Tracker",
    "TrackState",
    "PID",
    "AxisMapper",
    "ParallaxCompensator",
]
