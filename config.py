"""
config.py — Central configuration for the turret detection system.
Tweak these values to adjust detection behaviour, display, and camera settings.
"""

import os

# ─── Model ────────────────────────────────────────────────────────────────────
MODEL_PATH   = "models/yolo11n.pt"   # downloaded automatically on first run
CONFIDENCE   = 0.50                   # minimum detection confidence (0–1)
IOU_THRESH   = 0.45                   # NMS IoU threshold
IMGSZ        = 640                    # inference input size (px)

# Classes to track — None tracks all COCO classes; set to a list to filter.
# Example: TRACK_CLASSES = ["person"]
TRACK_CLASSES = ["person"]

# ─── Camera ───────────────────────────────────────────────────────────────────
# On Windows / Linux with USB webcam set to an integer (0 = default cam).
# On Raspberry Pi with the official camera module, set to "picamera2".
CAMERA_SOURCE = "auto"   # "auto" | 0 | 1 | "picamera2"
FRAME_WIDTH   = 1280
FRAME_HEIGHT  = 720
CAMERA_FPS    = 30

# ─── Display ──────────────────────────────────────────────────────────────────
WINDOW_TITLE  = "Turret — Detection Feed"
SHOW_FPS      = True
SHOW_LABELS   = True
SHOW_CONF     = True

# Box colours (BGR) per class; falls back to DEFAULT_BOX_COLOR for unknowns.
CLASS_COLORS = {
    "person": (0, 230, 118),
}
DEFAULT_BOX_COLOR = (0, 165, 255)

# ─── Paths ────────────────────────────────────────────────────────────────────
ROOT_DIR   = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(ROOT_DIR, "models")
LOGS_DIR   = os.path.join(ROOT_DIR, "logs")
