import time
import math
import orcas_serial
from laser_control import LaserControl
from target_detector import TargetDetector

class TrackingController:
    def __init__(self):
        self.state = 'standby'
        self.dt = 0.2  # 5 Hz nominal loop time
        
        self.standby_timeout = 5.0
        self.standby_timer = 0.0
        
        self.searching_timeout = 5.0
        self.searching_timer = 0.0

        # Laser control logic
        self.laser = LaserControl()
        self.detector = TargetDetector()

        # PID configurations (from handoff)
        self.Kp = {'pan': 0.4, 'tilt': 0.4}
        self.Ki = {'pan': 0.02, 'tilt': 0.02}
        self.Kd = {'pan': 0.08, 'tilt': 0.08}
        
        self.I_term_prev = {'pan': 0, 'tilt': 0}
        self.offset_prev = {'pan': 0, 'tilt': 0}

        # Expected Camera properties 
        self.camera_fov = {'width': 48.8, 'height': 62.2}
        self.aim_icon_size = 60
        self.frame_center = {'x': 320, 'y': 240} # Assuming 640x480 resolution
        
        # Internal state tracking for serial angles
        self.curr_tilt_angle = 0
        self.curr_pan_angle = 0

    def interpolate(self, x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1) 

    def pixel_to_angle(self, offset_x, offset_y, distance_mm):
        if distance_mm == 0:
            distance_mm = 2000 # default distance

        max_opposite_pan = distance_mm * math.tan(math.radians(self.camera_fov['width'] / 2))
        opposite_pan = self.interpolate(offset_x, 0, 0, self.frame_center['x'], max_opposite_pan)
        pan_angle = math.degrees(math.atan(opposite_pan / distance_mm))
        
        max_opposite_tilt = distance_mm * math.tan(math.radians(self.camera_fov['height'] / 2))
        opposite_tilt = self.interpolate(offset_y, 0, 0, self.frame_center['y'], max_opposite_tilt)
        tilt_angle = math.degrees(math.atan(opposite_tilt / distance_mm))

        return pan_angle, tilt_angle

    def pid_ctrl(self, offset_x, offset_y):
        P_pan = offset_x * self.Kp['pan']
        I_pan = self.I_term_prev['pan'] + (offset_x * self.Ki['pan'])
        D_pan = (offset_x - self.offset_prev['pan']) * self.Kd['pan']
        self.I_term_prev['pan'] = I_pan
        self.offset_prev['pan'] = offset_x
        out_pan = P_pan + I_pan + D_pan

        P_tilt = offset_y * self.Kp['tilt']
        I_tilt = self.I_term_prev['tilt'] + (offset_y * self.Ki['tilt'])
        D_tilt = (offset_y - self.offset_prev['tilt']) * self.Kd['tilt']
        self.I_term_prev['tilt'] = I_tilt
        self.offset_prev['tilt'] = offset_y
        out_tilt = P_tilt + I_tilt + D_tilt

        return out_pan, out_tilt

    def reset_pid(self):
        self.I_term_prev = {'pan': 0, 'tilt': 0}
        self.offset_prev = {'pan': 0, 'tilt': 0}

    def update(self, frame):
        ret, pan, tilt = orcas_serial.get_pan_tilt_curr_angle()
        if ret == 0:
            self.curr_pan_angle = pan
            self.curr_tilt_angle = tilt

        if self.state == 'standby':
            self.standby_timer += self.dt
            if self.standby_timer > self.standby_timeout:
                self.state = 'tilt_homing'
                self.standby_timer = 0
                print("Transition to tilt_homing")

        elif self.state == 'tilt_homing':
            # Level the barrel
            orcas_serial.set_pan_tilt_rotate_angle(0, -self.curr_tilt_angle)
            if abs(self.curr_tilt_angle) < 1.0:
                self.state = 'searching'
                self.searching_timer = 0
                self.reset_pid()
                print("Transition to searching")

        elif self.state == 'searching':
            cx, cy = self.detector.detect_target(frame)
            if cx >= 0 and cy >= 0:
                self.searching_timer = 0 # reset because we found a target
                
                offset_x = cx - self.frame_center['x']
                offset_y = self.frame_center['y'] - cy # typical image coords have inverted Y

                # Convert pixel offset to motor steps via PID (which outputs pixel correction)
                # and then from pixel correction to angles. Or apply PID directly to angle errors.
                # Following handoff PID wrapper:
                pid_x, pid_y = self.pid_ctrl(offset_x, offset_y)
                
                # Fetch aiming distance
                ret, dist = orcas_serial.get_aiming_distance()
                if ret != 0 or dist == 0:
                    dist = 2000
                
                # Using the handoff suggested pixel_to_angle:
                pan_delta, tilt_delta = self.pixel_to_angle(pid_x, pid_y, dist)
                
                # The returned pan_delta will be positive if to the right, negative if to the left.
                if offset_x < 0: pan_delta = -pan_delta
                if offset_y < 0: tilt_delta = -tilt_delta
                
                orcas_serial.set_pan_tilt_rotate_angle(pan_delta, tilt_delta)

                if abs(offset_x) < self.aim_icon_size and abs(offset_y) < self.aim_icon_size:
                    # Target acquired in the center crosshair
                    print(f"Target Acquired! Offsets: ({offset_x}, {offset_y})")
                    self.laser.fire(0.15)
            else:
                # No target found
                self.searching_timer += self.dt
                self.reset_pid()
                if self.searching_timer > self.searching_timeout:
                    self.state = 'standby'
                    self.searching_timer = 0
                    print("Lost target. Transition to standby")
