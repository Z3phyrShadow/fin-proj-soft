"""
Action Layer - Operating Modes
Defines behavior for Standby, Monitor, Engage, and Abort modes
"""

from enum import Enum
from dataclasses import dataclass
import time


class ActionMode(Enum):
    """Action layer operating modes"""
    STANDBY = "standby"           # Passive - no movement
    MONITOR = "monitor"           # Track + log only  
    ENGAGE = "engage"             # Full engagement
    ABORT = "abort"               # Emergency stop


@dataclass
class ActionConfig:
    """Configuration for action mode behavior"""
    mode: ActionMode
    auto_track: bool              # Automatically track targets
    auto_engage: bool             # Automatically engage targets
    alert_on_detection: bool      # Trigger alerts
    log_detections: bool          # Log all detections
    allow_movement: bool          # Allow turret movement
    engagement_cooldown: float    # Seconds between engagements


# Predefined configurations for each mode
MODE_CONFIGS = {
    ActionMode.STANDBY: ActionConfig(
        mode=ActionMode.STANDBY,
        auto_track=False,
        auto_engage=False,
        alert_on_detection=False,
        log_detections=True,
        allow_movement=False,
        engagement_cooldown=0.0
    ),
    
    ActionMode.MONITOR: ActionConfig(
        mode=ActionMode.MONITOR,
        auto_track=True,
        auto_engage=False,
        alert_on_detection=True,
        log_detections=True,
        allow_movement=True,
        engagement_cooldown=0.0
    ),
    
    ActionMode.ENGAGE: ActionConfig(
        mode=ActionMode.ENGAGE,
        auto_track=True,
        auto_engage=True,
        alert_on_detection=True,
        log_detections=True,
        allow_movement=True,
        engagement_cooldown=2.0
    ),
    
    ActionMode.ABORT: ActionConfig(
        mode=ActionMode.ABORT,
        auto_track=False,
        auto_engage=False,
        alert_on_detection=False,
        log_detections=True,
        allow_movement=False,
        engagement_cooldown=0.0
    )
}


class ActionController:
    """Controls action layer behavior and mode transitions"""
    
    def __init__(self, initial_mode: ActionMode = ActionMode.STANDBY):
        """Initialize action controller"""
        self.current_mode = initial_mode
        self.config = MODE_CONFIGS[initial_mode]
        self.last_engagement_time = 0
        self.engaged_targets = set()
        
        print(f"[ACTION] Initialized in {initial_mode.value.upper()} mode")
    
    def set_mode(self, mode: ActionMode) -> bool:
        """Change operating mode with safety checks"""
        if mode == self.current_mode:
            return True
        
        # Validate transition (can't go directly from ABORT to ENGAGE)
        if self.current_mode == ActionMode.ABORT and mode == ActionMode.ENGAGE:
            print(f"[ACTION] Cannot transition directly from ABORT to ENGAGE (safety)")
            return False
        
        old_mode = self.current_mode
        self.current_mode = mode
        self.config = MODE_CONFIGS[mode]
        self.engaged_targets.clear()
        
        print(f"[ACTION] Mode changed: {old_mode.value.upper()} → {mode.value.upper()}")
        return True
    
    def should_track(self) -> bool:
        """Determine if targets should be tracked"""
        return self.config.allow_movement and self.config.auto_track
    
    def should_engage(self, target_centered: bool) -> bool:
        """Determine if target should be engaged"""
        if not self.config.auto_engage:
            return False
        
        if not target_centered:
            return False
        
        # Check cooldown
        time_since_last = time.time() - self.last_engagement_time
        if time_since_last < self.config.engagement_cooldown:
            return False
        
        return True
    
    def record_engagement(self, target_info: str):
        """Record that a target was engaged"""
        self.last_engagement_time = time.time()
        print(f"[ENGAGE] 🎯 {target_info}")