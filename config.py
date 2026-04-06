"""
config.py — RAKSHAQ Turret System — Central Configuration
Modify values here; no changes to source files required.
"""

# ==================== DETECTION ====================
MODEL_PATH    = "models/yolo11n.onnx"
CONFIDENCE    = 0.50
IOU_THRESH    = 0.45
IMGSZ         = 320         # inference resolution — try 320 for ~4× speedup
TRACK_CLASSES = ["person"]   # None = detect all COCO classes

# Run YOLO inference every N frames; reuse last detections in between.
# 1 = every frame (slowest), 2 = every other frame, 3 = every 3rd, etc.
DETECTION_SKIP_FRAMES = 2

# ==================== CAMERA ====================
CAMERA_SOURCE   = "auto"     # "auto" | "picamera2" | device index (0, 1, …)
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480
SHOW_FPS        = False
CAMERA_THREADED = True       # background capture thread (recommended on Pi)
CAMERA_ROTATE   = "90CW"     # "none" | "90CW" | "90CCW" | "180"

# ==================== HARDWARE — MOTORS ====================
STM32_PORT     = "/dev/ttyACM0"   # Nucleo serial port
STM32_BAUD     = 115200

# Camera field-of-view (Pi Camera v3 with 90° CW rotation: H/V swap)
# After 90° rotation: effective HFOV ≈ physical VFOV and vice-versa
CAMERA_HFOV_DEG = 66.0   # degrees horizontal  (tune if tracking overshoots)
CAMERA_VFOV_DEG = 52.0   # degrees vertical

# Motor step limits (software safety — beyond this, stop commanding)
PAN_STEP_LIMIT   = 3200  # ≈ 360° in each direction
TILT_STEP_LIMIT  = 800   # ≈ ±90°

# Max steps sent per frame (prevents violent snap-to-target)
MOTOR_MAX_STEPS_PER_FRAME = 200  # ≈ 22°

# Proportional gain for motor tracking (0.0–1.0).
# 1.0 = immediately correct full error each frame → oscillation.
# 0.3–0.5 = move a fraction each frame → smoother convergence.
TRACKING_P_GAIN = 0.4

# Pan/tilt direction inversion (flip if motor moves the wrong way)
PAN_INVERT  = True
TILT_INVERT = True

# ==================== HARDWARE — SENSORS ====================
TOF_PORT        = "/dev/ttyAMA0"
TOF_BAUD        = 921600
TOF_MAX_MM      = 5000    # sensor range ceiling for UI normalisation

SONAR_ECHO_PIN      = 24
SONAR_TRIGGER_PIN   = 23
SONAR_MAX_DISTANCE_M = 4.0  # gpiozero DistanceSensor max_distance (metres)

LASER_GPIO_PIN      = 27    # MOSFET gate driving the laser (OutputDevice)

# ==================== ACTION LAYER ====================
DEFAULT_MODE = "monitor"    # "standby" | "monitor" | "engage" | "abort"

TARGETING_STRATEGY    = "closest" # closest to frame center
CENTER_TOLERANCE_X    = 50   # px
CENTER_TOLERANCE_Y    = 50   # px
ENGAGEMENT_COOLDOWN   = 2.0  # seconds between recorded engagements
AUTO_SCAN_ON_NO_TARGET = True
SCAN_INTERVAL          = 30  # seconds between auto-scans when no target

# ==================== DISPLAY ====================
SHOW_MODE_INDICATOR  = True
SHOW_TARGET_RETICLE  = True
SHOW_POSITION_INFO   = True
SHOW_SENSOR_HUD      = True   # show TOF + sonar readings on main feed

COLOR_TARGET       = (0, 0, 255)
COLOR_RETICLE      = (0, 255, 255)
COLOR_TEXT         = (255, 255, 255)
COLOR_MODE_STANDBY = (128, 128, 128)
COLOR_MODE_MONITOR = (0, 255, 255)
COLOR_MODE_ENGAGE  = (0, 0, 255)
COLOR_MODE_ABORT   = (255, 0, 0)

# ==================== DEPTH / RANGE ====================
# Backend:
#   "bbox"       — bounding-box size proxy (no sensors needed, Windows testing)
#   "tof"        — TOF laser sensor (/dev/ttyAMA0)  [recommended on Pi]
#   "ultrasonic" — HC-SR04 sonar (GPIO 24/23)
DEPTH_BACKEND = "tof"

# Threshold in mm (for tof/ultrasonic): engage when distance <= threshold
# Threshold in %  (for bbox):           engage when closeness >= threshold
DEPTH_THRESHOLD = 2500   # mm  (≈ 2.5 m)

# Hysteresis
DEPTH_ENGAGE_FRAMES  = 3    # frames inside threshold → ENGAGE
DEPTH_RETREAT_FRAMES = 10   # frames outside threshold → MONITOR

# Enable depth trigger on startup (toggle at runtime with 'd' key)
DEPTH_ENABLED  = True
SHOW_DEPTH_UI  = True

# ==================== STREAMING ====================
ENABLE_STREAM = True
STREAM_PORT   = 5000
STREAM_QUALITY = 70   # JPEG quality 0–100
