"""
Configuration for RAKSHAQ Turret System
Centralized settings for all modules
"""

# ==================== DETECTION ====================
MODEL_PATH = "models/yolo11n.pt"
CONFIDENCE = 0.50
IOU_THRESH = 0.45
TRACK_CLASSES = ["person"]  # None = detect all COCO classes
CAMERA_SOURCE = "auto"  # "auto", "picamera2", or device index
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
SHOW_FPS = True

# ==================== ACTION LAYER ====================

# Operating Modes
DEFAULT_MODE = "monitor"         # "standby", "monitor", "engage", "abort"

# Servo Control
PAN_SERVO_PIN = 17               # GPIO pin for pan servo
TILT_SERVO_PIN = 27              # GPIO pin for tilt servo
SERVO_FREQUENCY = 50             # PWM frequency (Hz)
SMOOTHING_FACTOR = 0.3           # Movement smoothing (0=instant, 1=no movement)

# Servo Limits
PAN_MIN_ANGLE = 0                # Minimum pan angle (degrees)
PAN_MAX_ANGLE = 180              # Maximum pan angle (degrees)
TILT_MIN_ANGLE = 45              # Minimum tilt angle (degrees)
TILT_MAX_ANGLE = 135             # Maximum tilt angle (degrees)

# Targeting
TARGETING_STRATEGY = "combined"  # "closest", "confident", "largest", "combined"
CENTER_TOLERANCE_X = 50          # Pixels - target considered centered
CENTER_TOLERANCE_Y = 50          # Pixels

# Engagement
ENGAGEMENT_COOLDOWN = 2.0        # Seconds between engagements
AUTO_SCAN_ON_NO_TARGET = True    # Scan when no targets detected
SCAN_INTERVAL = 30               # Seconds between scans

# ==================== DISPLAY ====================

# Visualization
SHOW_MODE_INDICATOR = True       # Show current mode on display
SHOW_TARGET_RETICLE = True       # Show crosshair on selected target
SHOW_POSITION_INFO = True        # Show pan/tilt angles

# Colors (BGR format)
COLOR_TARGET = (0, 0, 255)       # Red for selected target
COLOR_RETICLE = (0, 255, 255)    # Yellow crosshair
COLOR_TEXT = (255, 255, 255)     # White text
COLOR_MODE_STANDBY = (128, 128, 128)  # Gray
COLOR_MODE_MONITOR = (0, 255, 255)    # Yellow
COLOR_MODE_ENGAGE = (0, 0, 255)       # Red
COLOR_MODE_ABORT = (255, 0, 0)        # Blue

# ==================== DEPTH ====================

# Backend: "bbox" (bounding-box size proxy, for testing)
#          "ir"   (IR laser range sensor, for production)
DEPTH_BACKEND       = "bbox"

# IR sensor max range in cm (used only when DEPTH_BACKEND = "ir")
DEPTH_IR_MAX_CM     = 500.0

# Proximity % (0–100) above which the turret auto-engages
DEPTH_THRESHOLD     = 40

# Hysteresis — prevents flickering at the boundary
DEPTH_ENGAGE_FRAMES  = 3    # consecutive frames above threshold → switch to ENGAGE
DEPTH_RETREAT_FRAMES = 10   # consecutive frames below threshold → return to MONITOR

# Show the floating depth control window
SHOW_DEPTH_UI       = True