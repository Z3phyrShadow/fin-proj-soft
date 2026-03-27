"""
Turret Controller - Servo/Motor Control
Platform-aware GPIO control for pan/tilt servos
"""

import time
from typing import Tuple

# Platform-aware GPIO import
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("[TURRET] Real GPIO detected (Raspberry Pi)")
except (ImportError, RuntimeError):
    # Mock GPIO for development/testing
    class MockGPIO:
        BCM = "BCM"
        OUT = "OUT"
        
        @staticmethod
        def setmode(mode): pass
        
        @staticmethod
        def setwarnings(flag): pass
        
        @staticmethod
        def setup(pin, mode): 
            print(f"[MOCK GPIO] Setup pin {pin} as {mode}")
        
        @staticmethod
        def cleanup(pins=None): 
            print(f"[MOCK GPIO] Cleanup {pins}")
        
        class PWM:
            def __init__(self, pin, freq):
                self.pin = pin
                print(f"[MOCK GPIO] PWM on pin {pin} @ {freq}Hz")
            
            def start(self, duty): pass
            
            def ChangeDutyCycle(self, duty):
                print(f"[MOCK GPIO] Pin {self.pin} duty: {duty:.1f}%")
            
            def stop(self): pass
    
    GPIO = MockGPIO()
    GPIO_AVAILABLE = False
    print("[TURRET] Mock GPIO (development mode)")


class TurretController:
    """
    Controls pan and tilt servos for turret movement
    
    Coordinate system:
    - Pan: 0° (left) to 180° (right), 90° = center
    - Tilt: 45° (down) to 135° (up), 90° = center
    """
    
    def __init__(
        self,
        pan_pin: int = 17,
        tilt_pin: int = 27,
        servo_frequency: int = 50,
        smoothing_factor: float = 0.3
    ):
        """Initialize turret controller"""
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        self.servo_frequency = servo_frequency
        self.smoothing_factor = smoothing_factor
        
        # Current positions (degrees)
        self.pan_angle = 90.0
        self.tilt_angle = 90.0
        
        # Servo limits
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 45
        self.tilt_max = 135
        
        # Setup GPIO
        self._setup_gpio()
        
        # Center position
        self.reset()
        
        print(f"[TURRET] Initialized (Pan: GPIO{pan_pin}, Tilt: GPIO{tilt_pin})")
    
    def _setup_gpio(self):
        """Initialize GPIO pins and PWM"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        
        self.pan_pwm = GPIO.PWM(self.pan_pin, self.servo_frequency)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, self.servo_frequency)
        
        self.pan_pwm.start(0)
        self.tilt_pwm.start(0)
    
    def _angle_to_duty_cycle(self, angle: float) -> float:
        """Convert angle (0-180°) to PWM duty cycle (2-12%)"""
        angle = max(0, min(180, angle))
        return 2.0 + (angle / 180.0) * 10.0
    
    def set_pan(self, angle: float):
        """Set absolute pan angle"""
        angle = max(self.pan_min, min(self.pan_max, angle))
        duty = self._angle_to_duty_cycle(angle)
        
        self.pan_pwm.ChangeDutyCycle(duty)
        time.sleep(0.015)
        self.pan_pwm.ChangeDutyCycle(0)
        
        self.pan_angle = angle
    
    def set_tilt(self, angle: float):
        """Set absolute tilt angle"""
        angle = max(self.tilt_min, min(self.tilt_max, angle))
        duty = self._angle_to_duty_cycle(angle)
        
        self.tilt_pwm.ChangeDutyCycle(duty)
        time.sleep(0.015)
        self.tilt_pwm.ChangeDutyCycle(0)
        
        self.tilt_angle = angle
    
    def adjust(self, pan_delta: float, tilt_delta: float):
        """Adjust position by delta angles with smoothing"""
        pan_delta *= self.smoothing_factor
        tilt_delta *= self.smoothing_factor
        
        new_pan = self.pan_angle + pan_delta
        new_tilt = self.tilt_angle + tilt_delta
        
        self.set_pan(new_pan)
        self.set_tilt(new_tilt)
    
    def track_target(self, target_cx: int, target_cy: int, frame_width: int, frame_height: int):
        """
        Calculate and apply movement to center target in frame
        
        Args:
            target_cx: Target center X coordinate
            target_cy: Target center Y coordinate  
            frame_width: Frame width in pixels
            frame_height: Frame height in pixels
        """
        # Calculate error from frame center
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2
        
        error_x = target_cx - frame_center_x
        error_y = target_cy - frame_center_y
        
        # Convert pixel error to angle adjustment
        # Assume ~60° horizontal FOV, ~45° vertical FOV
        pan_adjustment = (error_x / frame_width) * 30   # ±15°
        tilt_adjustment = -(error_y / frame_height) * 20 # ±10° (Y inverted)
        
        self.adjust(pan_adjustment, tilt_adjustment)
    
    def reset(self):
        """Reset turret to center position"""
        print("[TURRET] Resetting to center")
        self.set_pan(90)
        self.set_tilt(90)
        time.sleep(0.5)
    
    def scan_area(self):
        """Perform horizontal scan sweep"""
        print("[TURRET] Scanning area...")
        for angle in [60, 90, 120, 90]:
            self.set_pan(angle)
            time.sleep(0.5)
    
    def emergency_stop(self):
        """Emergency stop - return to center"""
        print("[TURRET] ⚠️ EMERGENCY STOP")
        self.reset()
    
    def get_position(self) -> Tuple[float, float]:
        """Get current (pan, tilt) angles"""
        return (self.pan_angle, self.tilt_angle)
    
    def is_centered(self, target_cx: int, target_cy: int, 
                    frame_width: int, frame_height: int,
                    tolerance_x: int = 50, tolerance_y: int = 50) -> bool:
        """Check if target is centered within tolerance"""
        frame_center_x = frame_width / 2
        frame_center_y = frame_height / 2
        
        return (abs(target_cx - frame_center_x) < tolerance_x and 
                abs(target_cy - frame_center_y) < tolerance_y)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        print("[TURRET] Cleaning up...")
        self.reset()
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        GPIO.cleanup([self.pan_pin, self.tilt_pin])