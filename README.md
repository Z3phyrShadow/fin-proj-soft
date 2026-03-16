# Raspberry Pi Auto-Turret — Fin-Proj-Soft

A computer-vision-driven automatic turret built around a Raspberry Pi.
This repository contains the detection module (Phase 1) using **YOLOv11n**,
with motor/servo control to follow in a later phase.

---

## Architecture

```
fin-proj-soft/
├── main.py                          # Entry point
├── config.py                        # All tunable parameters
├── models/                          # YOLO weights (auto-downloaded)
├── src/
│   └── turret/
│       ├── detection/
│       │   ├── camera.py            # Platform-aware camera (Pi cam / webcam)
│       │   └── detector.py          # YOLOv11n wrapper → Detection objects
│       └── utils/
│           └── visualizer.py        # Bounding-box + HUD overlay
├── requirements.txt                 # Core deps (Windows & Pi)
└── requirements-pi.txt              # Pi-only deps (picamera2)
```

---

## Quick Start

### Windows (development — laptop webcam)

```bash
# 1. Create a virtual environment (recommended)
python -m venv .venv
.venv\Scripts\activate

# 2. Install dependencies
pip install -r requirements.txt

# 3. Run
python main.py
```

The first run will automatically download `yolo11n.pt` (~6 MB) into `models/`.

### Raspberry Pi (production)

```bash
# 1. Install system-level camera support
sudo apt update
sudo apt install python3-picamera2

# 2. Install Python deps
pip install -r requirements.txt
pip install -r requirements-pi.txt

# 3. Run
python main.py
```

The code auto-detects the platform — no changes needed between environments.

---

## Configuration (`config.py`)

| Key | Default | Description |
|---|---|---|
| `MODEL_PATH` | `models/yolo11n.pt` | Path to YOLO weights |
| `CONFIDENCE` | `0.50` | Minimum detection confidence |
| `IOU_THRESH` | `0.45` | NMS IoU threshold |
| `TRACK_CLASSES` | `["person"]` | COCO classes to detect (`None` = all) |
| `CAMERA_SOURCE` | `"auto"` | `"auto"` / `"picamera2"` / device index |
| `FRAME_WIDTH/HEIGHT` | `1280 × 720` | Capture resolution |
| `SHOW_FPS` | `True` | FPS counter in preview window |

---

## Controls

| Key | Action |
|---|---|
| `q` or `Esc` | Quit |

---

## Detection Output

Each detected object is represented by a `Detection` object:

```python
Detection(
    class_id   = 0,
    class_name = "person",
    confidence = 0.87,
    x1=120, y1=45, x2=380, y2=610,   # bounding box
)
# helpers:
det.center   # (cx, cy) — useful for turret targeting
det.width    # px
det.height   # px
```

---

## Roadmap

- [x] Phase 1 — Real-time detection (YOLOv11n)
- [ ] Phase 2 — Turret targeting (servo/motor control via GPIO)
- [ ] Phase 3 — Tracking loop (PID controller)
- [ ] Phase 4 — Safety features & arming logic

---

## Requirements

- Python 3.10+
- Raspberry Pi 4 / 5 (or any Linux/Windows machine for dev)
- Pi Camera Module v2/v3 **or** any USB webcam (for testing)
