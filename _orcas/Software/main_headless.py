import time
import cv2
import orcas_serial
from tracking_controller import TrackingController

def main():
    print("Starting ORCAS Autonomous Tracker...")
    
    # Initialize UART connection to STM32
    if orcas_serial.init() != 0:
        print("Failed to initialize STM32 via UART. Check connection.")
        # Proceeding anyway just in case the user wants to test YOLO logic without STM32?
        # Actually it's better to keep going for headless testing where hardware isn't fully set up.
    
    # Initialize Camera
    cap = cv2.VideoCapture(0)
    # Recommended resolution for YOLO nano speed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # Let the camera warm up
    time.sleep(2)
    
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    tracker = TrackingController()
    
    print("Entering Main Loop. Press Ctrl+C to stop.")
    try:
        while True:
            start_time = time.time()
            
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame from camera. Retrying...")
                time.sleep(0.5)
                continue
                
            # Run the autonomous state machine
            tracker.update(frame)
            
            # Maintain a roughly constant loop rate based on the tracker's dt
            elapsed = time.time() - start_time
            sleep_time = tracker.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Stopping ORCAS...")
        
    except Exception as e:
        print(f"Runtime error: {e}")
        
    finally:
        cap.release()
        orcas_serial.kill()
        tracker.laser.cleanup()
        print("Cleanup complete. Exiting.")

if __name__ == "__main__":
    main()
