# RAKSHAQ — Autonomous AI Turret System

A Raspberry Pi 5-based autonomous turret with real-time YOLOv11n detection, STM32 stepper motor control, TOF laser ranging, and sonar proximity sensing.

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
    │   ├── camera.py                    # Platform-aware camera (Pi / webcam) + threaded capture
    │   ├── detector.py                  # YOLOv11n wrapper → Detection objects
    │   └── depth.py                     # DepthEstimator (TOF mm / sonar mm / bbox %)
    ├── hardware/
    │   ├── stm32.py                     # STM32/Nucleo serial motor controller (X/Y steps)
    │   └── sensors.py                   # TOFSensor (UART binary) + UltrasonicSensor (gpiozero)
    ├── action/
    │   ├── modes.py                     # ActionMode enum + ActionController
    │   ├── controller.py                # TurretController (pixel error → step commands)
    │   └── targeting.py                 # TargetSelector (priority strategies)
    ├── ui/
    │   └── depth_control.py             # Floating depth threshold control window
    └── utils/
        ├── visualizer.py                # Bounding-box + HUD overlay
        └── streamer.py                  # Flask MJPEG live feed (port 5000)
```

---

## Hardware

| Component | Interface | Details |
|---|---|---|
| **Raspberry Pi 5** | — | Main compute |
| **Pi Camera Module v3** | CSI | Mounted sideways → 90° CW rotation in software |
| **STM32/Nucleo** | Serial `/dev/ttyACM0` @ 115200 | Pan/tilt stepper driver |
| **TOF Laser Sensor** | UART `/dev/ttyAMA0` @ 921600 | 16-byte binary frames, distance in mm |
| **HC-SR04 Sonar** | GPIO echo=24, trigger=23 | Secondary ranging via gpiozero |

### Motor Math (1/16 microstepping)
```
400 steps = 45°   →   8.888 steps/degree
X<steps>  = pan  (+ right, – left)
Y<steps>  = tilt (+ up,   – down)
```

### Wiring
```
STM32 USB  →  /dev/ttyACM0   (Nucleo virtual COM port)
TOF TX     →  Pi UART RX  (/dev/ttyAMA0)
Sonar ECHO →  GPIO 24
Sonar TRIG →  GPIO 23
```

---

## Quick Start

### Windows (development — laptop webcam)

```bash
uv sync
uv run python main.py
```

Set `DEPTH_BACKEND = "bbox"` in `config.py` — uses bounding-box size as a depth proxy (no sensors needed).

### Raspberry Pi 5 (production)

```bash
# 1. Install system packages (apt-managed, not in venv)
sudo apt update
sudo apt install python3-picamera2 python3-gpiozero

# 2. Create venv that can see the apt-installed packages
uv venv --system-site-packages

# 3. Install remaining Python deps
uv sync

# 4. Run
uv run python main.py
```

The first run downloads `yolo11n.pt` (~6 MB) into `models/` automatically.

### Remote access

**VNC (full desktop):**
```bash
sudo raspi-config  # Interface Options → VNC → Enable
```
Connect with [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/).

**SSH + X11 (for OpenCV window over network):**
```bash
# Windows: install VcXsrv, then:
ssh -Y pi@<pi-ip>
uv run python main.py
```

**Browser stream:**  
Access `http://<pi-ip>:5000/` — annotated MJPEG feed, no SSH needed.

---

## Operating Modes

| Key | Mode | Behaviour |
|-----|------|-----------|
| `1` | STANDBY | No movement, passive logging only |
| `2` | MONITOR | Track targets, no engagement |
| `3` | ENGAGE | Full tracking + engagement |
| `0` | ABORT | Emergency stop (cannot jump to ENGAGE from here) |

Default mode set via `DEFAULT_MODE` in `config.py`.

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| `q` / `Esc` | Quit |
| `r` | Reset turret to home position |
| `s` | Manual scan sweep (±45°) |
| `d` | Toggle depth trigger ON / OFF |
| `1–3, 0` | Switch operating mode |

---

## Depth System

A floating **Depth Control** window shows current sensor reading vs threshold.  
When the target is within range for 3 consecutive frames → **MONITOR → ENGAGE**.  
When it retreats for 10 frames → back to **MONITOR**.

| `DEPTH_BACKEND` | Source | Engage when |
|---|---|---|
| `"tof"` | TOF laser, `/dev/ttyAMA0` | `distance_mm ≤ threshold` |
| `"ultrasonic"` | HC-SR04, GPIO 24/23 | `distance_mm ≤ threshold` |
| `"bbox"` | Bounding-box height (testing) | `closeness% ≥ threshold` |

The sensor HUD (top-right of the main feed) shows live **LASER** and **SONAR** readings in mm.

---

## Targeting Strategies

Set via `TARGETING_STRATEGY` in `config.py`:

| Strategy | Description |
|----------|-------------|
| `closest` | Nearest to frame centre |
| `confident` | Highest confidence score |
| `largest` | Largest bounding box |
| `combined` | Weighted score of confidence × proximity × size *(default)* |

---

## Configuration (`config.py`)

### Camera
| Key | Default | Description |
|-----|---------|-------------|
| `CAMERA_SOURCE` | `"auto"` | `"auto"` / `"picamera2"` / device index |
| `FRAME_WIDTH/HEIGHT` | `1280×720` | Capture resolution |
| `CAMERA_ROTATE` | `"90CW"` | `"none"` / `"90CW"` / `"90CCW"` / `"180"` |
| `CAMERA_THREADED` | `True` | Background capture thread (recommended on Pi) |

### Hardware — Motors
| Key | Default | Description |
|-----|---------|-------------|
| `STM32_PORT` | `/dev/ttyACM0` | Nucleo serial port |
| `STM32_BAUD` | `115200` | Baud rate |
| `CAMERA_HFOV_DEG` | `66.0` | Camera horizontal FOV (degrees) — tune if tracking overshoots |
| `CAMERA_VFOV_DEG` | `52.0` | Camera vertical FOV |
| `MOTOR_MAX_STEPS_PER_FRAME` | `200` | Step cap per frame (~22°) — prevents violent snap |
| `PAN_INVERT / TILT_INVERT` | `False` | Flip motor direction if wired reversed |

### Hardware — Sensors
| Key | Default | Description |
|-----|---------|-------------|
| `TOF_PORT` | `/dev/ttyAMA0` | TOF laser UART port |
| `TOF_BAUD` | `921600` | TOF baud rate |
| `TOF_MAX_MM` | `5000` | Sensor range ceiling (mm) |
| `SONAR_ECHO_PIN` | `24` | HC-SR04 echo GPIO pin |
| `SONAR_TRIGGER_PIN` | `23` | HC-SR04 trigger GPIO pin |

### Depth Trigger
| Key | Default | Description |
|-----|---------|-------------|
| `DEPTH_BACKEND` | `"tof"` | `"tof"` / `"ultrasonic"` / `"bbox"` |
| `DEPTH_THRESHOLD` | `1500` | Engage threshold (mm for tof/sonar, % for bbox) |
| `DEPTH_ENGAGE_FRAMES` | `3` | Frames in-range before → ENGAGE |
| `DEPTH_RETREAT_FRAMES` | `10` | Frames out-of-range before → MONITOR |
| `DEPTH_ENABLED` | `True` | Enable on startup (`d` key toggles at runtime) |

### Streaming
| Key | Default | Description |
|-----|---------|-------------|
| `ENABLE_STREAM` | `True` | Start Flask MJPEG server |
| `STREAM_PORT` | `5000` | HTTP port |
| `STREAM_QUALITY` | `70` | JPEG quality (0–100) |

---

## Performance Notes

- **FPS bottleneck**: YOLOv11n on Pi 5 CPU ≈ 10–20 FPS with threaded camera
- **NCNN export** (optional, ~3–5× faster): `uv run python -c "from ultralytics import YOLO; YOLO('models/yolo11n.pt').export(format='ncnn')"`  
  Then set `MODEL_PATH = "models/yolo11n_ncnn_model"` in config

---

## Requirements

- Python 3.10+
- **Raspberry Pi 5** (or any Windows/Linux machine for development)
- **Pi Camera Module v3** (production) or USB webcam (testing)
- STM32/Nucleo board running pan/tilt stepper firmware
- TOF laser sensor (UART, `/dev/ttyAMA0`)
- HC-SR04 ultrasonic sensor
- 2× stepper motors (1/16 microstepping)

---

## Roadmap

- [x] Phase 1 — Real-time YOLOv11n detection
- [x] Phase 2 — STM32 stepper motor control via serial
- [x] Phase 2b — TOF + sonar depth-based auto-engage trigger
- [x] Phase 2c — Flask MJPEG remote stream
- [ ] Phase 3 — PID controller for smooth tracking
- [ ] Phase 4 — Safety arming logic & trigger control
