import time
import gpiozero

LASER_GPIO_PIN = 18

class LaserControl:
    def __init__(self):
        try:
            self.laser = gpiozero.DigitalOutputDevice(LASER_GPIO_PIN, active_high=True, initial_value=False)
            print(f"Laser initialized on BCM GPIO {LASER_GPIO_PIN}")
        except Exception as e:
            print(f"Failed to initialize laser on GPIO {LASER_GPIO_PIN}: {e}")
            self.laser = None

    def fire(self, duration_s=0.15):
        if self.laser:
            self.laser.on()
            time.sleep(duration_s)
            self.laser.off()
            print(f"Laser fired for {duration_s}s")
        else:
            print(f"Laser fire command ignored (MOSFET dummy run for {duration_s}s)")

    def cleanup(self):
        if self.laser:
            self.laser.off()
            self.laser.close()
