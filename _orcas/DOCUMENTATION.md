# Rakshq Technical Documentation

This document serves as the fundamental onboard manual for new engineers working on the Rakshaq turret system. It details the complete hardware configuration layer, pin mapping, required setup procedures, and software operation guide.

---

## 1. System Requirements & Hardware List

- **Master Controller:** Raspberry Pi 5 (8GB)
- **Vision:** Raspberry Pi Camera Module 3
- **Microcontroller:** STM32 Nucleo (STM32F103)
- **Motor Control Board:** CNC Shield V3 (Stackable on Nucleo)
- **Stepper Drivers:** A4988 (x2)
- **Actuators:** Pan Stepper Motor (X-Axis), Tilt Stepper Motor (Y-Axis)
- **Weapon System:** Low-power Laser Diode
- **Switching mechanism:** N-Channel Logic-Level MOSFET (e.g. IRLZ44N)
- **Sensors:** Serial UART Laser Range Sensor
- **Power Delivery:** 12V supply feeding an industrial 12V-to-5V Buck Converter for logic circuits.

---

## 2. Wiring and Pin Mappings

### 2.1 Raspberry Pi 5 GPIO Mappings
| Pin Definition | BCM Pin / Board Pin | Connection | Note |
| --- | --- | --- | --- |
| **Laser Trigger Gate** | BCM 18 (Pin 12) | MOSFET Gate | Sends 3.3v HIGH to close the MOSFET circuit. Use a logic-level MOSFET. |
| **UART TX** | BCM 14 (Pin 8) | STM32 PA10 (RX) | Master command transmission. Ensure grounds are tied. |
| **UART RX** | BCM 15 (Pin 10) | STM32 PA9 (TX) | Confirmation/Acknowledgement stream back from STM32. |

### 2.2 STM32 Nucleo / CNC Shield Mappings
The CNC Shield stacks onto the Nucleo headers natively.
- **X-Axis A4988 Slot:** Pan Stepper Motor.
- **Y-Axis A4988 Slot:** Tilt Stepper Motor.
- **Pan Home Switch:** Connected to `PAN_MOTOR_ORIGIN_Pin` (Active LOW internally pulled up).

> [!WARNING]
> DO NOT power the stepper motors using the 5V line. The CNC Shield requires a completely independent 12V power supply inserted into its motor power block terminal.

---

## 3. Deployment & Setup Procedures

### 3.1 Pi Environment Preparation
You must enable the physical hardware UART on the Raspberry Pi 5 prior to launching.
1. Run `sudo raspi-config`
2. Navigate to **Interface Options -> Serial Port**
3. Select **NO** to "login shell over serial"
4. Select **YES** to "enable serial port hardware"
5. Reboot the Pi.

### 3.2 Software Library Setup
Inside the `Software/` directory, perform standard dependency resolutions:
```bash
sudo apt update && sudo apt install python3-pip python3-opencv
pip3 install ultralytics pyserial gpiozero opencv-python --break-system-packages
```

### 3.3 Vision Model (YOLOv11) Setup
The `target_detector.py` script utilizes Ultralytics. The code assumes local resolution of `yolo11n.pt`. 
If `yolo11n.pt` is physically present in the same directory as the executable script, the framework will use the file locally, eliminating standard download delays. We advise manually saving `yolo11n.pt` into the `Software/` folder.

---

## 4. Software Architecture Flow

1. **State Boot:** `main_headless.py` configures the UART abstraction layer through `orcas_serial.py`.
2. **Homing:** During initialization, the `TrackingController` shifts state into `tilt_homing` instructing the STM32 to mathematically reset the barrel pitch to center.
3. **Observation:** With homing complete, state transitions to `searching`. The CV2 camera object starts polling.
4. **Detection Mapping:** Each frame is ingested by `TargetDetector`. A bounding box query runs looking for Class `0` (Person). The bounding box geometric center `(CX, CY)` is captured.
5. **PID Alignment:** The mathematical pixel delta between `(CX, CY)` and the image center is fed through the Proportional, Integral, and Derivative gains defined inside `TrackingController` `[Kp=0.4, Ki=0.02, Kd=0.08]`. 
6. **Actuation:** `orcas_serial.py` encodes the PID offset array output into a scaled big-endian 16-bit payload wrapped precisely in the `0x5A` payload protocol, dictating exact angles for STM32 to instruct raw stepping on the CNC shield. 
7. **Laser Fire:** Once pixel proximity asserts that the mechanical target overlaps the perceived target, `laser_control.py` sets BCM 18 high, illuminating the target indefinitely as long as constraints match.
