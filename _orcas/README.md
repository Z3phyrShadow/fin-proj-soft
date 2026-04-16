# Rakshaq: Autonomous Target Engagement System

Rakshaq is a motorised turret system designed to autonomously detect and engage targets. Powered by a Raspberry Pi 5 acting as the master node and an STM32 Nucleo for raw hardware motor control, it actively scans for targets via camera, runs YOLOv11 for bounding box acquisition, and engages a MOSFET-switched laser mechanism.

## High-Level Architecture
- **Perception:** Raspberry Pi Camera Module 3 grabs raw video. `target_detector.py` invokes YOLOv11 nano inference to detect targets (defaulting to the "person" class).
- **Control:** The pixel-offset error is passed into a PID-controlled state machine in `tracking_controller.py`. Once a target is homed in, the controller fires a laser beam automatically.
- **Hardware Interface:** The Pi sends precise pan/tilt angle commands via a custom 9600-baud UART protocol (`orcas_serial.py`) down to the STM32 board.
- **Actuation:** The STM32 reads these commands and generates STEP/DIR signals for the connected A4988 motor drivers to move the physical tracking turret.

## Repository Contents
- **`Software/`**: Contains the Raspberry Pi Python ecosystem.
  - `main_headless.py`: Main executable that orchestrates the system.
  - `target_detector.py`: YOLOv11 target detection module.
  - `tracking_controller.py`: PID logic for aiming.
  - `laser_control.py`: GPIO 18 driver for the MOSFET laser switch.
  - `orcas_serial.py`: UART abstraction layer.
- **`Firmware/`**: Holds the STM32CubeIDE project for the microcontroller motor code.

## Quick Start
1. Ensure the STM32 is flashed with the provided firmware.
2. Ensure both the 5V logic and 12V motor power routes are active.
3. On the Raspberry Pi, navigate to the `Software` folder and install dependencies:
   ```bash
   pip3 install ultralytics pyserial gpiozero opencv-python --break-system-packages
   ```
4. Run the main controller script:
   ```bash
   python3 main_headless.py
   ```

*For complete wiring diagrams, pin setups, and in-depth software explanation, please refer to [DOCUMENTATION.md](DOCUMENTATION.md).*
