import time
import numpy as np
import orcas_serial
from picamera2 import Picamera2
from tracking_controller import TrackingController

def main():
    print("Starting ORCAS Autonomous Tracker...")

    # ── STM32 UART init ────────────────────────────────────────────────────────
    # The Nucleo resets when the serial port is opened (DTR toggles). Wait long
    # enough for it to boot and finish the pan homing sequence before sending
    # any commands. 3 seconds is conservative; reduce to 2 s once confirmed.
    print("Waiting for STM32 to boot and home...")
    time.sleep(3)
    if orcas_serial.init() != 0:
        print("WARNING: STM32 UART init failed. Motors will not respond.")
        print("  - Check USB cable (Nucleo micro-USB → Pi USB-A)")
        print("  - Verify firmware is flashed on the Nucleo")
        print("  - Run: ls /dev/ttyACM* to confirm device exists")
        # Continue anyway so camera/YOLO can still be tested independently.

    # ── Camera init (Pi Camera Module 3 via picamera2/libcamera) ──────────────
    # Pi Camera Module 3 is NOT a V4L2 device — cv2.VideoCapture(0) will NOT
    # work. It must be accessed through picamera2 / libcamera.
    print("Initialising Pi Camera Module 3...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)   # allow auto-exposure to settle
    print("Camera ready.")

    tracker = TrackingController()

    print("Entering Main Loop. Press Ctrl+C to stop.")
    try:
        while True:
            start_time = time.time()

            # capture_array() returns an ndarray (RGB888) directly usable by YOLO
            frame = picam2.capture_array()
            if frame is None or frame.size == 0:
                print("Failed to grab frame from camera. Retrying...")
                time.sleep(0.5)
                continue

            # Run the autonomous state machine
            tracker.update(frame)

            # Maintain loop rate
            elapsed = time.time() - start_time
            sleep_time = tracker.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Stopping ORCAS...")

    except Exception as e:
        print(f"Runtime error: {e}")
        import traceback; traceback.print_exc()

    finally:
        picam2.stop()
        orcas_serial.kill()
        tracker.laser.cleanup()
        print("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()
