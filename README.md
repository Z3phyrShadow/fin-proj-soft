# RAKSHAQ — Autonomous AI Turret System

A Raspberry Pi-based automatic turret with real-time object detection and servo control.
Built with **YOLOv11n** for detection and a four-mode action layer for autonomous targeting.

---

## Architecture

```
fin-proj-soft/
├── main.py                              # Entry point (RakshaqSystem class)
├── config.py                            # All tunable parameters
├── models/                              # YOLO weights (auto-downloaded)
├── pyproject.toml                       # uv dependencies
└── src/turret/
    ├── detection/
    │   ├── camera.py                    # Platform-aware camera (Pi cam / webcam)
    │   ├── detector.py                  # YOLOv11n wrapper → Detection objects
    │   └── depth.py                     # DepthEstimator (bbox proxy / IR sensor)
    ├── action/
    │   ├── modes.py                     # ActionMode enum + ActionController
    │   ├── controller.py                # TurretController (GPIO / Mock GPIO)
    │   └── targeting.py                 # TargetSelector (priority strategies)
    ├── ui/
    │   └── depth_control.py             # Floating depth threshold control window
    └── utils/
        └── visualizer.py                # Bounding-box + HUD overlay
```

---

## Quick Start

### Windows (development — laptop webcam)

```bash
# Sync dependencies (creates .venv automatically)
uv sync

# Run
uv run python main.py
```

The first run will automatically download `yolo11n.pt` (~6 MB) into `models/`.
On Windows, `RPi.GPIO` is unavailable — the system switches to **Mock GPIO** automatically,
so servo commands are printed to the console instead of moving real hardware.

### Raspberry Pi 5 (production)

```bash
# 1. Install picamera2 via apt
sudo apt update
sudo apt install python3-picamera2

# 2. Create the venv with system-site-packages so picamera2 is visible inside it
uv venv --system-site-packages

# 3. Sync the remaining Python deps
uv sync

# 4. Run
uv run python main.py
```

---

## Operating Modes

Switch modes at runtime with keyboard shortcuts:

| Key | Mode       | Behaviour |
|-----|------------|-----------|
| `1` | STANDBY    | No movement, passive logging only |
| `2` | MONITOR    | Track targets, alert on detection, no engagement |
| `3` | ENGAGE     | Full tracking + engagement (with cooldown) |
| `0` | ABORT      | Emergency stop — cannot jump directly to ENGAGE from here |

Default mode on startup is set via `DEFAULT_MODE` in `config.py` (default: `"monitor"`).

---

## Keyboard Controls

| Key      | Action |
|----------|--------|
| `q` / `Esc` | Quit |
| `r`      | Reset turret to center |
| `s`      | Manual scan sweep |
| `d`      | Toggle depth trigger ON / OFF |
| `1–3, 0` | Switch operating mode |

---

## Targeting Strategies

Set via `TARGETING_STRATEGY` in `config.py`:

| Strategy   | Description |
|------------|-------------|
| `closest`  | Target nearest to frame centre |
| `confident`| Target with highest confidence score |
| `largest`  | Target with largest bounding box |
| `combined` | Weighted score of confidence × proximity × size *(recommended)* |

---

## Depth System

A floating **Depth Control** window opens alongside the main feed.
Use the trackbar to set the proximity threshold (0–100).
When the selected target's depth score crosses the threshold for 3 consecutive frames,
the turret automatically switches from **MONITOR → ENGAGE**.
When it retreats for 10 frames, it returns to **MONITOR**.

### Visual indicators
- A small bar renders below each target bounding box — green = safe, red = triggered
- A yellow tick on the bar marks the current threshold position
- The mode label in the main feed switches colour when ENGAGE fires

### Backends

| `DEPTH_BACKEND` | How depth is measured | Use case |
|---|---|---|
| `"bbox"` | Bounding-box height ÷ frame height × 100 | Testing (now) |
| `"ir"` | IR laser range sensor reading, normalised to 0–100 | Production (Pi) |

Swap backend in `config.py` — no code changes needed anywhere else.

---

## Configuration (`config.py`)

### Detection
| Key | Default | Description |
|-----|---------|-------------|
| `MODEL_PATH` | `models/yolo11n.pt` | YOLO weights path |
| `CONFIDENCE` | `0.50` | Min detection confidence |
| `TRACK_CLASSES` | `["person"]` | COCO classes to track (`None` = all) |
| `CAMERA_SOURCE` | `"auto"` | `"auto"` / `"picamera2"` / device index |
| `FRAME_WIDTH/HEIGHT` | `1280×720` | Capture resolution |

### Action Layer
| Key | Default | Description |
|-----|---------|-------------|
| `DEFAULT_MODE` | `"monitor"` | Startup mode |
| `PAN_SERVO_PIN` | `17` | GPIO BCM pin for pan servo |
| `TILT_SERVO_PIN` | `27` | GPIO BCM pin for tilt servo |
| `SMOOTHING_FACTOR` | `0.3` | Movement smoothing (0 = instant, 1 = no movement) |
| `TARGETING_STRATEGY` | `"combined"` | Target selection strategy |
| `CENTER_TOLERANCE_X/Y` | `50` | Px tolerance to consider target centred |
| `ENGAGEMENT_COOLDOWN` | `2.0` | Seconds between engagements |
| `AUTO_SCAN_ON_NO_TARGET` | `True` | Sweep when no targets detected |

### Depth
| Key | Default | Description |
|-----|---------|-------------|
| `DEPTH_ENABLED` | `True` | Enable depth trigger on startup (`d` key toggles at runtime) |
| `DEPTH_BACKEND` | `"bbox"` | `"bbox"` (testing) or `"ir"` (production) |
| `DEPTH_THRESHOLD` | `40` | Proximity % that triggers ENGAGE |
| `DEPTH_ENGAGE_FRAMES` | `3` | Consecutive frames above threshold → ENGAGE |
| `DEPTH_RETREAT_FRAMES` | `10` | Consecutive frames below threshold → MONITOR |
| `DEPTH_IR_MAX_CM` | `500.0` | IR sensor range for normalisation (ir backend only) |
| `SHOW_DEPTH_UI` | `True` | Show floating depth control window |

---

## Detection Output

Each detected object is a `Detection` object:

```python
Detection(
    class_id=0, class_name="person", confidence=0.87,
    x1=120, y1=45, x2=380, y2=610,
)
det.center   # (cx, cy) — used for servo targeting
```

---

## Servo Wiring (Raspberry Pi)

```
Pan servo  signal  →  GPIO 17 (BCM)
Tilt servo signal  →  GPIO 27 (BCM)
Both servos VCC    →  5V rail
Both servos GND    →  GND
```

Servo angle range: Pan 0–180°, Tilt 45–135°. Both default to 90° (centre).

---

## Requirements

- Python 3.10+
- **Raspberry Pi 5** (or any Windows/Linux machine for development)
- **Pi Camera Module v3** (production) **or** USB webcam / laptop cam (testing)
- 2× SG90 (or similar) servo motors

---

## Roadmap

- [x] Phase 1 — Real-time YOLOv11n detection
- [x] Phase 2 — Action layer (modes, targeting, servo control scaffold)
- [x] Phase 2b — Depth-based auto-engage trigger (bbox proxy + IR sensor interface)
- [ ] Phase 3 — PID controller for smooth tracking
- [ ] Phase 4 — Safety features, arming logic & trigger control
