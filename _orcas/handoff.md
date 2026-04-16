**ORCAS**

Autonomous Target Engagement System

_Software Handoff Guide - Revised Architecture_

| **AUDIENCE** | This document is for the software developer who will write or modify code for the updated ORCAS hardware setup. No prior knowledge of the project is assumed. Read this document from top to bottom before touching any code. |
| ------------ | ----------P------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

| **DATE** | April 2026 \| Hardware: Raspberry Pi 5 (8 GB) + STM32 Nucleo (CNC Shield V3 + A4988 drivers) + Pi Camera Module 3 \| Removed: Ultrasonic sensor, Radar motor assembly |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

# **1\. Project Overview**

ORCAS is a motorised turret system designed to autonomously detect and engage targets. The system runs a YOLOv11 nano model on a Raspberry Pi 5 to identify targets in a camera feed, then commands an STM32 microcontroller over UART to move two stepper motors (pan and tilt) and fire a laser.

## **1.1 What Changed From the Original Codebase**

The original codebase in the repository (ORCAS-main) was built around a different hardware stack. The table below maps what has been removed, what has been kept, and what is new.

| **REMOVED**                                                                                                                                                                             | **KEPT**                                                                                                                                                                                                                                                    | **NEW / CHANGED**                                                                                                                                                                                                                       |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Ultrasonic sensor (GY-US42)<br><br>Radar motor + ToF sensor sweep<br><br>AEG motor (airsoft gearbox)<br><br>Ammo feeder stepper<br><br>Mini Buck Converter for servo<br><br>Servo motor | STM32 Nucleo board (STL Dev Board)<br><br>CNC Shield V3 + A4988 drivers<br><br>Pan stepper motor<br><br>Tilt stepper motor<br><br>UART serial link (RPi &lt;-&gt; STM32)<br><br>Industrial Buck Converter (12V motor power)<br><br>Camera Module 3 (via Pi) | YOLOv11 nano target detection (replaces HSV color tracking)<br><br>MOSFET-switched LASER (replaces AEG firing)<br><br>No ultrasonic/radar; Pi does all target localisation<br><br>No gearbox - firing is now digital (GPIO gate signal) |

# **2\. Hardware Architecture**

## **2.1 System Block Diagram (Text Description)**

Power flows from a single external power supply. An industrial buck converter steps voltage down to 5 V for the Raspberry Pi 5. A second path powers the CNC Shield directly at 12 V for the stepper motors. The MOSFET receives switched power from the supply and controls the laser via a gate signal from the Pi GPIO.

Data flow: Camera Module 3 and the Laser Range Sensor feed data into the Pi. The Pi processes everything with YOLO and sends UART commands to the STM32. The STM32 translates those commands into Step/Dir pulses on the CNC Shield which drives the two A4988 stepper drivers (pan axis, tilt axis).

## **2.2 Component Inventory**

| **Raspberry Pi 5**               | 8 GB RAM. Master controller. Runs Python, YOLOv11, and UART command loop.                   |
| -------------------------------- | ------------------------------------------------------------------------------------------- |
| **Camera Module 3**              | Connected via CSI ribbon. Provides live BGR frames to the Pi.                               |
| **STM32 Nucleo (STL Dev Board)** | STM32F103 series. Receives UART commands from Pi at 9600 baud and controls motors.          |
| **CNC Shield V3**                | Plugs onto the Nucleo breakout. Provides STEP/DIR/ENABLE headers for each axis.             |
| **A4988 Driver x2**              | Inserted into X-axis and Y-axis slots of the CNC Shield. One for pan, one for tilt stepper. |
| **Pan Stepper Motor**            | Rotates the turret left/right. Connected to X-axis A4988 on CNC Shield.                     |
| **Tilt Stepper Motor**           | Pitches the barrel up/down. Connected to Y-axis A4988 on CNC Shield.                        |
| **MOSFET**                       | Receives gate signal from Pi GPIO pin. Switches laser power on/off.                         |
| **LASER**                        | Switched by MOSFET. Fired under Pi control when target is acquired.                         |
| **Laser Range Sensor**           | Provides target distance data to Pi. Used for aiming calculations.                          |
| **Industrial Buck Converter**    | Steps supply voltage to 5 V for the Pi.                                                     |

## **2.3 Critical Wiring Notes**

| **WARNING** | The CNC Shield requires two separate power supplies: the 5 V logic power from the Nucleo USB/3.3V rail, AND a separate 12 V on the shield's motor power terminal. Do NOT power the motors from the Pi's 5 V. The A4988 VREF must be set for your stepper's current rating: VREF = I_max x 8 x Rs (Rs = 0.1 ohm on most A4988 boards). |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

- Pi GPIO pin (to be assigned) drives MOSFET gate for laser. Keep this signal 3.3 V - do not drive a 5 V MOSFET gate directly from Pi GPIO without a level shifter.
- UART TX from Pi connects to STM32 UART RX (PA10 on STM32F103). UART RX on Pi connects to STM32 TX (PA9). Use 115200 or 9600 baud - current firmware uses 9600.
- Pan motor home switch: active LOW, pulled up internally (EXTI falling edge interrupt on PAN_MOTOR_ORIGIN_Pin).
- Tilt motor has no physical endstop - tilt limits are enforced in software (+40 deg up, -15 deg down).

# **3\. Firmware Deep Dive (STM32 Side)**

The STM32 firmware lives in ORCAS-main/Firmware/Core/Src/. It is written with STM32 HAL and compiled with STM32CubeIDE. The developer should NOT need to touch the STM32 firmware unless changing motor parameters - all high-level behaviour is driven from the Pi over UART. This section documents what the firmware does so the software developer understands the contract.

## **3.1 Firmware Architecture**

The firmware uses a cooperative multitasking pattern. A single TIM2 timer interrupt fires every 250 microseconds. Inside that ISR, the firmware sets bitmask flags. The main while(1) loop checks those flags and dispatches to the appropriate periodic routine. This keeps ISR code minimal.

| **TIM2 Period**        | 250 us. Master tick. Sets dispatch flags.                                             |
| ---------------------- | ------------------------------------------------------------------------------------- |
| **UART_CMDS handling** | Runs every 10 ms. Parses incoming serial commands, sends ACK.                         |
| **ORIENTATION_CTRL**   | Runs every 250 us. Generates STEP pulses with acceleration/deceleration profile.      |
| **FIRE_CTRL**          | Runs every 1 ms (if compiled in). Controls AEG motor + feeder. NOT USED in new build. |
| **UART Watchdog**      | 1000 ms timeout. Resets UART receiver if no complete command received.                |

## **3.2 Motor Control Internals**

The orientation_ctrl_utils.c file implements a trapezoidal velocity profile (acc / constant speed / dec) for both pan and tilt motors using lookup tables of half-period values. The stepper pulse is toggled every N timer ticks, where N is looked up from the table based on the current step index.

- Pan motor: 8009 steps/revolution, 0.045 deg/step. Absolute position tracked in curr_pan_angle_in_steps. Homed to origin switch on boot.
- Tilt motor: 0.104 deg/step. Position tracked as floating-point angle (curr_tilt_angle) read from MMA845x accelerometer. NOTE: In the new build, the accelerometer may not be present. If TILT_CTRL macro is not defined, tilt tracking defaults to step counting.
- Both motors have configurable acceleration/deceleration ramp tables compiled at boot from pan_acc_dec_half_period_lookup\[\] and tilt_acc_dec_half_period_lookup\[\].

| **NOTE** | If you're not seeing smooth motion, check that the A4988 VREF is correctly set and microstepping jumpers on the CNC Shield match the steps-per-revolution values above. The firmware assumes full-step or a known microstepping factor. |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

## **3.3 Compile-time Feature Flags (main.h)**

The firmware conditionally compiles features based on #define macros in main.h. For the new build, you should have:

# define PAN_CTRL // Enable pan motor

// #define TILT_CTRL // Disable if no accelerometer

// #define FIRE_CTRL // Disable - no AEG in new build

// #define TOF_SENSORS_CTRL // Disable - no ultrasonic

| **IMPORTANT** | If FIRE_CTRL is disabled, the STM32 will return an error for any fire-related UART commands (0x02, 0x03, 0x04, 0x05, 0x06, 0x07). The Pi software must not send those commands unless the firmware is recompiled with FIRE_CTRL enabled. |
| ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

# **4\. UART Serial Protocol**

This is the most important section for the Pi-side developer. ALL communication between the Pi and STM32 goes through a single UART line at 9600 baud, 8N1. Every message (in both directions) follows a fixed binary framing format.

## **4.1 Frame Format**

**Command (Pi to STM32):**

\[0x5A\] \[CMD_CODE\] \[PARAM_0\] \[PARAM_1\] ... \[CHECKSUM\]

Checksum = sum of all bytes before checksum, masked to 0xFF.

**Acknowledgement (STM32 to Pi):**

\[0x5A\] \[ACK_CODE\] \[DATA_0\] \[DATA_1\] ... \[CHECKSUM\]

ACK_CODE 0x00 = SUCCESS. Any other value is an error code (see Section 4.3).

## **4.2 Full Command Reference**

| **CMD Name**              | **Code** | **Params** | **Description / Payload**                                                                                                                                             |
| ------------------------- | -------- | ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| GET_FW_VER                | 0x01     | none       | Returns 15-byte ASCII firmware version string. Use this to verify UART link is alive on startup.                                                                      |
| GET_PAN_TILT_CURR_ANGLE   | 0x09     | none       | Returns 4 bytes: pan_angle\*10 (int16, big-endian), tilt_angle\*10 (int16, big-endian). Pan: -180 to +180 deg. Tilt: -15 to +40 deg.                                  |
| SET_PAN_TILT_ROTATE_ANGLE | 0x0A     | 4 bytes    | Params: pan_delta\*10 (int16 BE), tilt_delta\*10 (int16 BE). Commands relative rotation in degrees (not absolute position). Positive pan = right. Positive tilt = up. |
| GET_PAN_TILT_STEP_ANGLE   | 0x08     | none       | Returns step resolution: pan_deg_per_step\*1000 (int16 BE), tilt_deg_per_step\*1000 (int16 BE). Use at startup for calibration.                                       |
| SET_FIRE_SAFETY           | 0x05     | 1 byte     | 0x01 = engage safety (stop firing). 0x00 = disengage safety. FIRE_CTRL must be compiled in.                                                                           |
| SET_FIRE_COUNT            | 0x03     | 1 byte     | Number of shots to fire (1 = semi, 3 = burst). FIRE_CTRL must be compiled in.                                                                                         |
| GET_FIRE_CTRL_STATUS      | 0x02     | none       | Returns 1-byte status bitmask: bit0=safety, bit1=AEG motor, bit2=piston timeout, bit3=red dot, etc.                                                                   |
| SET_SEARCHLIGHT_PWM       | 0x10     | 1 byte     | PWM duty 0-100 for the searchlight/targeting LED on TIM3 CH4. 0 = off.                                                                                                |
| SET_FIRE_CTRL_CONFIG      | 0x04     | 3 bytes    | semi_duty (0-100), auto_duty (0-100), piston_timeout/10. Configures AEG motor parameters.                                                                             |
| SET_AMMO_FEEDER_CONFIG    | 0x06     | 1 byte     | steps_per_shot / 10. Default 220 steps. FIRE_CTRL required.                                                                                                           |
| SET_AMMO_FEEDING          | 0x07     | 3 bytes    | dir (0=load, 1=unload), steps_high, steps_low. Manual feeder control.                                                                                                 |

## **4.3 Error / ACK Codes**

| **0x00** | SUCCESS                                                             |
| -------- | ------------------------------------------------------------------- |
| **0x01** | ERR\_\_INVALID_UART_CMD_CODE - Sent an unknown command byte.        |
| **0x02** | ERR\_\_INVALID_UART_CMD_CHECKSUM - Checksum mismatch. Retry.        |
| **0x10** | ERR\_\_MMA845X_COMM_FAIL - Accelerometer I2C failure (tilt sensor). |
| **0x11** | ERR\_\_TILT_MOTOR_STEP_INIT_FAIL - Tilt calibration failed at boot. |
| **0x12** | ERR\_\_TILT_ENDSTOP_REACHED - Tilt limit hit (+40 or -15 deg).      |
| **0x13** | ERR\_\_PAN_MOTOR_STEP_INIT_FAIL - Pan homing failed at boot.        |
| **0x30** | ERR\_\_AEG_PISTON_PULLING_TIMEOUT - AEG motor timeout (500 ms).     |

| **IMPORTANT** | The STM32 has a UART watchdog: if it does not receive a complete, valid command within 1000 ms, it resets the receiver state machine. The Pi should send commands at least every 500 ms to keep the link alive, or implement a heartbeat (e.g., periodic GET_PAN_TILT_CURR_ANGLE). |
| ------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

## **4.4 Angle Encoding**

Angles are encoded as signed 16-bit integers scaled by 10 (i.e., degrees \* 10), transmitted big-endian (high byte first).

\# Python example: send pan=+15.5 deg, tilt=-5.0 deg

pan = int(15.5 \* 10) # = 155

tilt = int(-5.0 \* 10) # = -50, must be sent as uint16 two's complement

data = struct.pack('>hh', pan, tilt) # 4 bytes

\# Decode response from GET_PAN_TILT_CURR_ANGLE:

pan_raw = struct.unpack('>h', data\[0:2\])\[0\] # signed int16

pan_deg = pan_raw / 10.0

# **5\. Raspberry Pi Software Architecture**

The original Software/ directory has six Python modules. In the new build several are obsolete (radar, ToF control) and the detection logic needs replacing. Below is a module-by-module breakdown.

## **5.1 Module Map**

| **orcas_serial.py**                  | KEEP. Handles all UART framing, send/receive, and checksum. Contains the communicate() loop. Modify only to add new command wrappers if needed. |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **orcas_attributes.py**              | KEEP / EDIT. Global state dictionary (calibration, status, cmds). Remove radar and AEG fields. Add YOLO and laser fields.                       |
| **orcas_camera.py**                  | KEEP. Manages camera capture. You may want to switch it to use Picamera2 for Pi Camera Module 3 instead of OpenCV VideoCapture.                 |
| **orcas*scenarios*\_tracking.py**    | MAJOR REWRITE. Currently uses HSV color blob detection. Must be rewritten to use YOLOv11 detections. PID control logic can stay.                |
| **orcas*scenarios*\_interacting.py** | MINOR EDIT. Remove radar-based pan logic. Keep touch-to-aim logic if a UI is still needed.                                                      |
| **orcas*scenarios*\_calibrating.py** | LOW PRIORITY. Camera calibration helpers. Leave unless recalibrating.                                                                           |
| **orcas_server.py**                  | KEEP or REMOVE. Flask web server for a phone/tablet UI. Irrelevant if running headlessly.                                                       |
| **orcas_radar.py**                   | DELETE or STUB OUT. Radar is removed. Any import of this module will error.                                                                     |

## **5.2 The communicate() Loop (orcas_serial.py)**

This function runs in its own thread (called from the server or main script). It loops every 0.2 seconds and performs the following in order:

- GET_PAN_TILT_CURR_ANGLE - reads current motor position, updates orcas_attributes.status\['aeg_pan_angle'\] and \['aeg_tilt_angle'\].
- GET_AIMING_DISTANCE - reads laser range sensor distance, updates \['aiming_distance'\].
- Processes outgoing movement commands from orcas_attributes.cmds\['pan_angle'\] / \['tilt_angle'\] - sends SET_PAN_TILT_ROTATE_ANGLE.
- Processes touch coordinate commands - converts pixel offset from aim crosshair to angle delta using FOV + distance math, then sends SET_PAN_TILT_ROTATE_ANGLE.
- Processes fire commands - sends SET_FIRE_COUNT (in new build: control laser via GPIO instead).
- Processes safety, searchlight, ammo, radar enable/disable commands.

| **NOTE** | In the new build, firing is done via a Pi GPIO pin controlling the MOSFET gate, NOT via the STM32 SET_FIRE_COUNT command (since there is no AEG). You need to add GPIO laser control code. Suggested: RPi.GPIO or gpiozero library. Assign a GPIO pin, drive it HIGH to fire, LOW to stop. |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |

## **5.3 Pixel-to-Angle Conversion (Key Math)**

When a target is detected at pixel coordinates (target*x, target_y), the angle error is computed relative to the aim crosshair centre. The existing interpolation logic in communicate() and orcas_scenarios*\_tracking.py is correct and should be reused.

max_opposite = distance_mm \* tan(radians(fov_deg / 2))

opposite = interpolate(pixel_offset, 0, 0, frame_width/2, max_opposite)

angle_deg = degrees(atan(opposite / distance_mm))

Camera FOV calibration values (from orcas_attributes.py):

- Horizontal FOV: 48.8 deg
- Vertical FOV: 62.2 deg

## **5.4 YOLOv11 Integration (New)**

The original tracking scenario (orcas*scenarios*\_tracking.py) uses OpenCV HSV colour thresholding to find blobs. This must be replaced with YOLOv11 nano inference. Below is the recommended integration pattern.

from ultralytics import YOLO

model = YOLO('yolo11n.pt') # or .ncnn format for best performance

def detect_target(frame):

results = model(frame, imgsz=320, conf=0.4)

for r in results:

boxes = r.boxes

if len(boxes):

b = boxes\[0\] # take highest confidence box

x1,y1,x2,y2 = map(int, b.xyxy\[0\])

cx = (x1+x2)//2

cy = (y1+y2)//2

return cx, cy

return -1, -1

| **PERFORMANCE** | For maximum FPS on RPi 5, export the model to NCNN format: model.export(format='ncnn') then load with YOLO('yolo11n_ncnn_model'). Expected speedup is 3-4x over .pt. Also set imgsz=320 rather than 640 to halve inference time. Target class filtering: use classes=\[0\] for person or the specific class index for your target. |
| --------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

# **6\. Laser Control (New - MOSFET via GPIO)**

In the new build there is no AEG/gearbox. The 'firing' mechanism is a laser switched by a MOSFET whose gate is driven by a Pi GPIO pin. This replaces all STM32 FIRE_CTRL functionality.

## **6.1 GPIO Laser Control Code**

import gpiozero

LASER_GPIO_PIN = 18 # BCM numbering - confirm with your wiring

laser = gpiozero.DigitalOutputDevice(LASER_GPIO_PIN, active_high=True, initial_value=False)

def fire_laser(duration_s=0.1):

laser.on()

time.sleep(duration_s)

laser.off()

| **WIRING** | Confirm which BCM pin is wired to the MOSFET gate. If the MOSFET requires 5 V gate drive and Pi GPIO is 3.3 V, use a logic level MOSFET (e.g., IRLZ44N or similar) or add a gate driver. Never exceed 3.3 V on Pi GPIO output pins. |
| ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |

## **6.2 Integrating into the Control Loop**

Replace all calls to set_fire_count() and related FIRE_CTRL serial commands with direct GPIO laser control. In orcas_attributes.cmds, replace 'direct_fire' handling:

\# In communicate() loop, replace:

\# if cmds\['direct_fire'\] == 1: set_fire_count(1)

\# With:

if cmds\['direct_fire'\] == 1:

fire_laser(duration_s=0.15)

cmds\['direct_fire'\] = 0

# **7\. Modified Tracking Scenario Pseudocode**

Below is the high-level pseudocode for the new tracking scenario to replace orcas*scenarios*\_tracking.py. The state machine structure stays the same; only the detection step changes.

States: standby -> tilt_homing -> searching -> firing -> standby

standby:

Wait N seconds, then transition to tilt_homing.

tilt_homing:

Send SET_PAN_TILT_ROTATE_ANGLE(0, -curr_tilt_angle) # level the barrel

When abs(curr_tilt_angle) &lt; 1 deg -&gt; go to searching

searching:

frame = camera.read()

cx, cy = detect_target(frame) # YOLOv11

if cx >= 0:

offset_x = cx - aim_crosshair_x

offset_y = cy - aim_crosshair_y

pan_delta, tilt_delta = PID_ctrl(offset_x, offset_y)

send SET_PAN_TILT_ROTATE_ANGLE(pan_delta, tilt_delta)

if abs(offset_x) < aim_icon_size and abs(offset_y) < aim_icon_size:

fire_laser() # target acquired

else:

searching_timeout += dt

if searching_timeout > MAX_TIMEOUT: go to standby

## **7.1 PID Tuning**

The existing PID constants are tuned for HSV blob tracking at low speed. With YOLO at potentially lower FPS, you may need to reduce Kp to avoid oscillation, or add frame interpolation. Start with:

| **Kp (pan, tilt)** | 0.4, 0.4 (reduce from 0.6 if oscillating)                             |
| ------------------ | --------------------------------------------------------------------- |
| **Ki (pan, tilt)** | 0.02, 0.02 (reduce from 0.05 if windup)                               |
| **Kd (pan, tilt)** | 0.08, 0.08                                                            |
| **Loop rate**      | Set to your actual YOLO inference rate (~5-15 FPS on RPi 5 with NCNN) |

# **8\. Setup & Getting Started**

## **8.1 Pi Environment Setup**

sudo apt update && sudo apt install python3-pip python3-opencv

pip3 install ultralytics pyserial gpiozero picamera2 --break-system-packages

\# Enable UART (disable serial console, keep UART hardware):

sudo raspi-config -> Interface Options -> Serial Port

\# Disable serial login shell, Enable serial port hardware

\# UART device: /dev/ttyS0 (mini UART) or /dev/ttyAMA0 (full UART)

\# Check: ls -la /dev/serial\*

## **8.2 STM32 Firmware Build**

- Open ORCAS-main/Firmware/ in STM32CubeIDE.
- Verify macros in main.h: define PAN_CTRL, undefine FIRE_CTRL and TOF_SENSORS_CTRL.
- Build and flash via ST-Link (USB on Nucleo board).
- On first boot, the STM32 will run the pan homing sequence - the pan motor will sweep until it finds the origin switch.

## **8.3 Verifying the Serial Link**

\# Quick test from Pi terminal:

python3 -c "

import serial, time

s = serial.Serial('/dev/ttyS0', 9600, timeout=1)

\# Send GET_FW_VER: header 0x5A, code 0x01, checksum 0x5B

s.write(bytes(\[0x5A, 0x01, 0x5B\]))

time.sleep(0.2)

print(s.read(18).hex()) # expect 5A 00 &lt;15 bytes version&gt; &lt;checksum&gt;

"

## **8.4 First Run Checklist**

- Flash STM32 firmware with correct compile flags.
- Connect UART (Pi TX -> STM32 RX, Pi RX -> STM32 TX, common ground).
- Power the CNC Shield 12 V motor rail separately from Pi 5 V.
- Set A4988 VREF for your stepper current rating before enabling motor power.
- Run the UART ping test above and confirm firmware version string appears.
- Test pan and tilt movement with a simple Python script calling set_pan_tilt_rotate_angle().
- Test YOLO detection independently on a single frame before integrating.
- Test laser GPIO fire (short pulse, safely aimed) before connecting tracking loop.
- Integrate YOLO into tracking loop. Tune PID until stable tracking.

# **9\. Known Issues & Things to Watch Out For**

| **orcas_radar.py imports** | orcas_attributes.py and orcas_serial.py both import orcas_radar. You must stub or remove all orcas_radar references or the script will crash at import time.                                                                           |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **TILT_CTRL macro**        | If the tilt firmware module is disabled (no accelerometer), GET_PAN_TILT_CURR_ANGLE will return 0 for tilt angle. Do not rely on tilt angle feedback unless the accelerometer is physically present and MMA845x is responding on I2C2. |
| **UART timeout**           | Serial read calls in orcas_serial.py use a 1-second timeout. If STM32 doesn't respond, the function returns -1. Always check the return code before using the data.                                                                    |
| **Thread safety**          | communicate() is designed to run in a background thread. orcas_attributes.status and .cmds are plain Python dicts - no locks. Race conditions are possible if you add more threads. Use threading.Lock if needed.                      |
| **Pi UART pin**            | On Pi 5, the default UART for /dev/ttyS0 may be the debug UART. Use /dev/ttyAMA0 (or the appropriate numbered ttyAMAx) for the hardware UART connected to GPIO 14/15. Verify with: ls -la /dev/serial0.                                |
| **YOLO model path**        | If running from different working directories, use absolute paths for the YOLO model file.                                                                                                                                             |
| **Laser duty cycle**       | Do not keep the laser on continuously for extended periods without checking the laser module's rated duty cycle. Use short pulses.                                                                                                     |

# **10\. Quick Reference Card**

## **10.1 Most-Used Serial Commands (Python Wrappers)**

import orcas_serial

orcas_serial.init() # init + version check

orcas_serial.get_pan_tilt_curr_angle() # -> (0, pan_deg, tilt_deg)

orcas_serial.set_pan_tilt_rotate_angle(p, t) # relative move in degrees

orcas_serial.set_fire_safety(1) # engage safety

orcas_serial.set_searchlight_pwm(80) # 80% duty searchlight

## **10.2 Key Constants in Existing Code**

| **Pan step angle**     | 0.045 deg/step (8009 steps/rev)                                  |
| ---------------------- | ---------------------------------------------------------------- |
| **Tilt step angle**    | 0.104 deg/step                                                   |
| **Tilt up limit**      | +40.0 degrees (software enforced in STM32)                       |
| **Tilt down limit**    | \-15.0 degrees (software enforced in STM32)                      |
| **UART baud rate**     | 9600 bps, 8N1, no flow control                                   |
| **UART port (Pi)**     | /dev/ttyS0 (adjust to your Pi serial config)                     |
| **Camera H-FOV**       | 48.8 degrees                                                     |
| **Camera V-FOV**       | 62.2 degrees                                                     |
| **Tracking loop rate** | Every 0.2 s (5 Hz) in communicate() - adjust for YOLO throughput |
| **UART watchdog**      | 1000 ms - send a command at least every 800 ms                   |

## **10.3 File Structure**

ORCAS-main/

Firmware/Core/Src/

main.c -- STM32 entry, timer ISR, feature flags

uart_cmds_handle.c -- Serial framing & command dispatch

orientation_ctrl_utils.c -- Pan/tilt stepper motor control

fire_ctrl_utils.c -- AEG firing (not used in new build)

tof_sensors_ctrl_utils.c -- Radar/ultrasonic (not used)

Software/

orcas_serial.py -- UART driver (keep)

orcas_attributes.py -- Global state (edit)

orcas_camera.py -- Camera capture (keep/update)

orcas*scenarios*\_tracking.py -- REWRITE with YOLO

orcas*scenarios*\_interacting.py -- minor edits

orcas_radar.py -- DELETE / stub out

orcas_server.py -- Flask UI (optional)

_ORCAS Software Handoff Guide | Prepared April 2026_