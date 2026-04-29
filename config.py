"""
config.py — RAKSHAQ Turret System — Central Configuration
Modify values here; no changes to source files required.
"""

# ==================== DETECTION ====================
MODEL_PATH    = "models/yolo11n_ncnn_model"
CONFIDENCE    = 0.40
IOU_THRESH    = 0.40
IMGSZ         = 320         # inference resolution — try 320 for ~4× speedup
TRACK_CLASSES = ["person"]   # None = detect all COCO classes

# Run YOLO inference every N frames; reuse last detections in between.
# 1 = every frame (slowest), 2 = every other frame, 3 = every 3rd, etc.
DETECTION_SKIP_FRAMES = 2

# ==================== CAMERA ====================
CAMERA_SOURCE   = "auto"     # "auto" | "picamera2" | device index (0, 1, …)
FRAME_WIDTH     = 480
FRAME_HEIGHT    = 360
SHOW_FPS        = True
CAMERA_THREADED = True       # background capture thread (recommended on Pi)
CAMERA_ROTATE   = "90CW"     # "none" | "90CW" | "90CCW" | "180"

# ==================== HARDWARE — MOTORS ====================
STM32_PORT     = "/dev/ttyACM0"   # Nucleo serial port
STM32_BAUD     = 115200

# Camera field-of-view — PHYSICAL sensor values in landscape orientation.
# AxisMapper swaps HFOV/VFOV internally when CAMERA_ROTATE is 90CW/90CCW,
# so that pan always uses the FOV spanning the displayed horizontal axis
# and tilt uses the one spanning the displayed vertical axis.
# Pi Camera v3 landscape: HFOV ≈ 66°, VFOV ≈ 52°
CAMERA_HFOV_DEG = 66.0   # physical sensor horizontal FOV (landscape)
CAMERA_VFOV_DEG = 52.0   # physical sensor vertical FOV   (landscape)

# Motor step limits (software safety — beyond this, stop commanding)
PAN_STEP_LIMIT   = 3200  # ≈ 360° in each direction
TILT_STEP_LIMIT  = 800   # ≈ ±90°

# Max steps sent per frame (prevents violent snap-to-target)
MOTOR_MAX_STEPS_PER_FRAME = 200  # ≈ 22°

# Pan/tilt direction inversion (flip if motor moves the wrong way).
# These only control motor wiring direction — axis swapping due to
# camera rotation is handled automatically by AxisMapper.
PAN_INVERT  = False
TILT_INVERT = True

# ==================== BARREL GEOMETRY ====================
# Physical offset of the LED barrel relative to the camera.
# All values in centimetres.
BARREL_RIGHT_CM   = 13.0     # barrel is 13 cm to the right of camera
BARREL_UP_CM      =  4.0     # barrel is 4 cm above camera
BARREL_FORWARD_CM = 14.0     # barrel sticks out 14 cm forward (turret tip)

# ==================== PID TRACKING ====================
# PID gains for pan and tilt axes.
# Tuned for smooth tracking (low P, moderate D, small I).
TRACK_KP = 0.35              # proportional: main responsiveness
TRACK_KI = 0.02              # integral: eliminates steady-state offset
TRACK_KD = 0.15              # derivative: dampens oscillation

# Anti-windup: max accumulated integral (degrees)
TRACK_I_MAX = 50.0

# Dead zone: don't command motors when target is within this many
# pixels of the aim point.  Prevents micro-jitter when centred.
TRACK_DEADZONE_PX = 15

# ==================== CHASE / SEARCH ====================
# When the target is lost, the tracker chases its predicted trajectory
# for CHASE_TIMEOUT_S seconds, then sweeps the area for SEARCH_TIMEOUT_S
# seconds before giving up.
CHASE_TIMEOUT_S      = 5.0       # seconds chasing predicted path
SEARCH_TIMEOUT_S     = 10.0      # seconds sweeping area
CHASE_PREDICT_FRAMES = 5         # rolling window for velocity estimate

# ==================== TARGET SELECTION ====================
TARGETING_STRATEGY    = "closest"   # "closest" to frame center (recommended)
CENTER_TOLERANCE_X    = 50          # px (for "is_centered" check)
CENTER_TOLERANCE_Y    = 50          # px
TARGET_SWITCH_HYSTERESIS = 40       # px — only switch targets if new one is
                                    # this much closer to centre

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
STREAM_QUALITY = 50   # JPEG quality 0–100 (lower = less CPU)
TRACKING_ENABLED  = True        # Use ByteTrack tracker instead of raw detect
TRACKER_TYPE      = "bytetrack" # "bytetrack" (recommended) | "botsort"