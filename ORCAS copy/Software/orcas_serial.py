import time
import math
import serial
import orcas_attributes 
import gpiozero

# Setup GPIO Laser (BCM 18)
LASER_GPIO_PIN = 18 
try:
    laser = gpiozero.DigitalOutputDevice(LASER_GPIO_PIN, active_high=True, initial_value=False)
except Exception as e:
    print(f"Failed to init laser on GPIO {LASER_GPIO_PIN}: {e}")
    laser = None

def fire_laser(duration_s=0.15):
    if laser:
        laser.on()
        time.sleep(duration_s)
        laser.off()
    else:
        print("Laser fired (dummy, GPIO failed)")


cmd_header = 0x5A
cmd_code = {'GET_FW_VER':                   0x01,
            # Removed FIRE_CTRL cmds
            'GET_PAN_TILT_STEP_ANGLE':      0x08,
            'GET_PAN_TILT_CURR_ANGLE':      0x09,
            'SET_PAN_TILT_ROTATE_ANGLE':    0x0A,
            # Removed Radar cmds
            'GET_AIMING_DISTANCE':          0x0F,
            'SET_SEARCHLIGHT_PWM':          0x10}            
cmd_params_length = {'GET_FW_VER':                  15,
                     'GET_PAN_TILT_STEP_ANGLE':     4, 
                     'GET_PAN_TILT_CURR_ANGLE':   	4,
                     'SET_PAN_TILT_ROTATE_ANGLE':   4,
                     'GET_AIMING_DISTANCE':         1,
                     'SET_SEARCHLIGHT_PWM':         1}
                     
ack_header = 0x5A                
ack_header_index = 0
ack_code_index = 1                     
ack_code = {0x00: 'SUCCESS',
            0x01: 'ERR__INVALID_UART_CMD_CODE',
            0x02: 'ERR__INVALID_UART_CMD_CHECKSUM',
            0x10: 'ERR__MMA845X_COMM_FAIL',
            0x11: 'ERR__TILT_MOTOR_STEP_INIT_FAIL',
            0x12: 'ERR__TILT_ENDSTOP_REACHED',
            0x13: 'ERR__PAN_MOTOR_STEP_INIT_FAIL'}              
ack_data_base_index = 2
ack_header_code_checksum_length = 3
              
orcas_serial = None

def init():
    global orcas_serial
    
    orcas_serial = serial.Serial(
        port='/dev/ttyS0',      
        baudrate=9600,        
        bytesize=serial.EIGHTBITS, 
        parity=serial.PARITY_NONE, 
        stopbits=serial.STOPBITS_ONE, 
        timeout=1            
    )

    if orcas_serial.is_open:
        data_to_send = bytearray([cmd_header, cmd_code['GET_FW_VER']])
        checksum = sum(data_to_send) & 0xFF
        data_to_send.append(checksum)
        orcas_serial.write(data_to_send)   
        
        received_data = orcas_serial.read(cmd_params_length['GET_FW_VER'] + ack_header_code_checksum_length)                
        if len(received_data) == cmd_params_length['GET_FW_VER'] + ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:
                        data_str = ''.join(chr(b) for b in received_data[ack_data_base_index:ack_data_base_index + cmd_params_length['GET_FW_VER']])
                        print(f"orcas_serial.init(): ORCAS firmware version {data_str}")
                        return 0
                    else:
                        print(f"orcas_serial.init(): {ack_code[received_data[ack_code_index]]}")
                else:
                    print("orcas_serial.init(): Invalid received ack header")
            else:
                print("orcas_serial.init(): Invalid received ack checksum")
        else:
            print("orcas_serial.init(): Invalid received ack length")
        orcas_serial.close()
        return -1
    
def get_pan_tilt_step_angle():
    global orcas_serial

    if orcas_serial.is_open:
        data_to_send = bytearray([cmd_header, cmd_code['GET_PAN_TILT_STEP_ANGLE']])
        checksum = sum(data_to_send) & 0xFF
        data_to_send.append(checksum)
        orcas_serial.write(data_to_send)
        
        received_data = orcas_serial.read(cmd_params_length['GET_PAN_TILT_STEP_ANGLE'] + ack_header_code_checksum_length)                
        if len(received_data) == cmd_params_length['GET_PAN_TILT_STEP_ANGLE'] + ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:
                        angle = ((received_data[ack_data_base_index] << 8) | (received_data[ack_data_base_index + 1])) / 1000
                        print(f"orcas_serial.get_pan_tilt_step_angle(): Pan step angle = {angle}")
                        angle = ((received_data[ack_data_base_index + 2] << 8) | (received_data[ack_data_base_index + 3])) / 1000
                        print(f"orcas_serial.get_pan_tilt_step_angle(): Tilt step angle = {angle}")
                        return 0
                    else:
                        print(f"orcas_serial.get_pan_tilt_step_angle(): {ack_code[received_data[ack_code_index]]}")
                else:
                    print("orcas_serial.get_pan_tilt_step_angle(): Invalid received ack header")
            else:
                print("orcas_serial.get_pan_tilt_step_angle(): Invalid received ack checksum")
        else:
            print("orcas_serial.get_pan_tilt_step_angle(): Invalid received ack length")
    return -1
    
def get_pan_tilt_curr_angle():
    global orcas_serial

    if orcas_serial.is_open:
        data_to_send = bytearray([cmd_header, cmd_code['GET_PAN_TILT_CURR_ANGLE']])
        checksum = sum(data_to_send) & 0xFF
        data_to_send.append(checksum)
        orcas_serial.write(data_to_send)
        
        received_data = orcas_serial.read(cmd_params_length['GET_PAN_TILT_CURR_ANGLE'] + ack_header_code_checksum_length)                
        if len(received_data) == cmd_params_length['GET_PAN_TILT_CURR_ANGLE'] + ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:
                        pan_angle = (received_data[ack_data_base_index] << 8) | (received_data[ack_data_base_index + 1])
                        if(pan_angle > 0x8000):
                            pan_angle -= 0x10000
                        pan_angle = pan_angle / 10.0    
                        
                        tilt_angle = (received_data[ack_data_base_index + 2] << 8) | (received_data[ack_data_base_index + 3])
                        if(tilt_angle > 0x8000):
                            tilt_angle -= 0x10000
                        tilt_angle = tilt_angle / 10.0                          
                        return 0, pan_angle, tilt_angle
                    else:
                        print(f"orcas_serial.get_pan_tilt_curr_angle(): {ack_code[received_data[ack_code_index]]}")
                else:
                    print("orcas_serial.get_pan_tilt_curr_angle(): Invalid received ack header")
            else:
                print("orcas_serial.get_pan_tilt_curr_angle(): Invalid received ack checksum")
        else:
            print("orcas_serial.get_pan_tilt_curr_angle(): Invalid received ack length")
    return -1, 0, 0
    
def set_pan_tilt_rotate_angle(pan_angle, tilt_angle):
    global orcas_serial
            
    if orcas_serial.is_open:
        pan = int(pan_angle * 10)
        tilt = int(tilt_angle * 10)
        data_to_send = bytearray([cmd_header, 
                                  cmd_code['SET_PAN_TILT_ROTATE_ANGLE'], 
                                  (pan >> 8) & 0xFF,
                                  pan & 0xFF,
                                  (tilt >> 8) & 0xFF,
                                  tilt & 0xFF])      
        checksum = sum(data_to_send) & 0xFF
        data_to_send.append(checksum)
        orcas_serial.write(data_to_send)
        #print(f"data_to_send: {data_to_send.hex().upper()}")
        
        received_data = orcas_serial.read(ack_header_code_checksum_length)                
        if len(received_data) == ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:                         
                        return 0
                    else:
                        print(f"orcas_serial.set_pan_tilt_rotate_angle(): {ack_code[received_data[ack_code_index]]}")
                else:
                    print("orcas_serial.set_pan_tilt_rotate_angle(): Invalid received ack header")
            else:
                print("orcas_serial.set_pan_tilt_rotate_angle(): Invalid received ack checksum")
        else:
            print("orcas_serial.set_pan_tilt_rotate_angle(): Invalid received ack length")
    return -1

def set_searchlight_pwm(duty):
    global orcas_serial
            
    if orcas_serial.is_open:
        data_to_send = bytearray([cmd_header, 
                                  cmd_code['SET_SEARCHLIGHT_PWM'], 
                                  duty])      
        checksum = sum(data_to_send) & 0xFF
        data_to_send.append(checksum)
        orcas_serial.write(data_to_send)
        #print(f"data_to_send: {data_to_send.hex().upper()}")
        
        received_data = orcas_serial.read(ack_header_code_checksum_length)                
        if len(received_data) == ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:                         
                        return 0
                    else:
                        print(f"orcas_serial.set_searchlight_pwm(): {ack_code[received_data[ack_code_index]]}")
                else:
                    print("orcas_serial.set_searchlight_pwm(): Invalid received ack header")
            else:
                print("orcas_serial.set_searchlight_pwm(): Invalid received ack checksum")
        else:
            print("orcas_serial.set_searchlight_pwm(): Invalid received ack length")
    return -1    

def get_aiming_distance():
    global orcas_serial

    if orcas_serial.is_open:
        data_to_send = bytearray([cmd_header, cmd_code['GET_AIMING_DISTANCE']])
        checksum = sum(data_to_send) & 0xFF
        data_to_send.append(checksum)
        orcas_serial.write(data_to_send)
        
        received_data = orcas_serial.read(cmd_params_length['GET_AIMING_DISTANCE'] + ack_header_code_checksum_length)                
        if len(received_data) == cmd_params_length['GET_AIMING_DISTANCE'] + ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:                        
                        target_range = received_data[ack_data_base_index] * 4
                        return 0, target_range
                    else:
                        print(f"orcas_serial.get_aiming_distance(): {ack_code.get(received_data[ack_code_index], 'Unknown Error')}")
                else:
                    print("orcas_serial.get_aiming_distance(): Invalid received ack header")
            else:
                print("orcas_serial.get_aiming_distance(): Invalid received ack checksum")
        else:
            print("orcas_serial.get_aiming_distance(): Invalid received ack length")
    return -1, 0
    
def interpolate(x, x1, y1, x2, y2):
	return y1 + (x - x1) * (y2 - y1) / (x2 - x1)    
    
def communicate():
    while True:
        ret, pan_angle, tilt_angle = get_pan_tilt_curr_angle()
        if ret == 0:
            if (abs(orcas_attributes.status['aeg_pan_angle'] - pan_angle) > 0.5):
                orcas_attributes.status['aeg_pan_angle'] = pan_angle
                orcas_attributes.status['aim_icon_coor_update_req'] = 1
            if (abs(orcas_attributes.status['aeg_tilt_angle'] - tilt_angle) > 0.5):
                orcas_attributes.status['aeg_tilt_angle'] = tilt_angle
                orcas_attributes.status['aim_icon_coor_update_req'] = 1
                
        ret, target_range = get_aiming_distance()
        if ((ret == 0) and (target_range != 0)):            
            if (orcas_attributes.status['aiming_distance'] != target_range):
                orcas_attributes.status['aiming_distance'] = target_range
                orcas_attributes.status['aim_icon_coor_update_req'] = 1
        
        if ((orcas_attributes.cmds['pan_angle'] != 0) or orcas_attributes.cmds['tilt_angle'] != 0):            
            set_pan_tilt_rotate_angle(orcas_attributes.cmds['pan_angle'], orcas_attributes.cmds['tilt_angle'])
            orcas_attributes.cmds['touch_coor']['x'] = 0
            orcas_attributes.cmds['touch_coor']['y'] = 0
            orcas_attributes.cmds['pan_angle'] = 0
            orcas_attributes.cmds['tilt_angle'] = 0
            
        if ((orcas_attributes.cmds['touch_coor']['x'] != 0) or (orcas_attributes.cmds['touch_coor']['y'] != 0)):
            if ((orcas_attributes.status['aiming_distance'] != 0) and 
                (orcas_attributes.status['received_frame_size']['width'] != 0) and 
                (orcas_attributes.status['received_frame_size']['height'] != 0)):
                
                # Set pan/tilt rotation angle (touch handling)
                touch_coor_x = orcas_attributes.cmds['touch_coor']['x'] + orcas_attributes.status['displayed_frame_origin_coor']['x']
                touch_coor_y = orcas_attributes.cmds['touch_coor']['y'] + orcas_attributes.status['displayed_frame_origin_coor']['y']
                if ((orcas_attributes.cmds['touch_holding'] == 0) or (orcas_attributes.cmds['touch_holding'] > 5)):                    
                    if ((abs(touch_coor_x - orcas_attributes.status['aim_icon_coor']['x']) > orcas_attributes.status['static_icon_size'] // 2)):                
                        max_opposite_length = orcas_attributes.status['aiming_distance'] * math.tan(math.radians(orcas_attributes.calibration['camera_fov']['width'] / 2))                                            
                        opposite_length = interpolate(touch_coor_x - orcas_attributes.status['aim_icon_coor']['x'], 
                                                      0, 0,
                                                      orcas_attributes.status['received_frame_size']['width'] / 2, max_opposite_length)
                        pan_angle = math.degrees(math.atan(opposite_length / orcas_attributes.status['aiming_distance']))                    
                    else:
                        pan_angle = 0                    
                    if (abs(touch_coor_y - orcas_attributes.status['aim_icon_coor']['y']) > orcas_attributes.status['static_icon_size'] // 2):
                        max_opposite_length = orcas_attributes.status['aiming_distance'] * math.tan(math.radians(orcas_attributes.calibration['camera_fov']['height'] / 2))                                             
                        opposite_length = interpolate(orcas_attributes.status['aim_icon_coor']['y'] - touch_coor_y,
                                                      0, 0,
                                                      orcas_attributes.status['received_frame_size']['height'] / 2, max_opposite_length)
                        tilt_angle = math.degrees(math.atan(opposite_length / orcas_attributes.status['aiming_distance']))
                    else:
                        tilt_angle = 0                    
                    if((pan_angle != 0) or (tilt_angle != 0)):
                        set_pan_tilt_rotate_angle(pan_angle, tilt_angle)
                 
                # Trigger firing by touching inside the aim icon in the interacting scenario
                if ((orcas_attributes.status['scenario'] == 'interacting') and
                    (abs(touch_coor_x - orcas_attributes.status['aim_icon_coor']['x']) < orcas_attributes.status['aim_icon_size'] // 2) and 
                    (abs(touch_coor_y - orcas_attributes.status['aim_icon_coor']['y']) < orcas_attributes.status['aim_icon_size'] // 2)):
                    if ((orcas_attributes.status['fire_mode'] == 'SAFE') or (orcas_attributes.status['fire_mode'] == 'AUTO')):
                        fire_laser(0.3)
                    else:
                        if (orcas_attributes.cmds['cease_fire'] == 0):
                            if(orcas_attributes.status['fire_mode'] == 'SEMI'):
                                fire_laser(0.15)
                            if(orcas_attributes.status['fire_mode'] == 'BURST'):
                                fire_laser(0.4)
                            orcas_attributes.cmds['cease_fire'] = 1                            
                                        
                if (orcas_attributes.cmds['clear_coor'] == 1):
                    orcas_attributes.cmds['touch_coor']['x'] = 0
                    orcas_attributes.cmds['touch_coor']['y'] = 0            
                    orcas_attributes.cmds['cease_fire'] = 0
                    orcas_attributes.cmds['touch_holding'] = 0
                    orcas_attributes.cmds['clear_coor'] = 0                     
                else:
                    orcas_attributes.cmds['touch_holding'] += 1
                    
        if (orcas_attributes.cmds['direct_fire'] == 1):
            if (orcas_attributes.status['fire_mode'] == 'SEMI'):
                fire_laser(0.15)
            else:
                fire_laser(0.4)                           
            orcas_attributes.cmds['direct_fire'] = 0                    
                
        # Removed safety and ammo since done in STM32 hardware in v1, now redundant. Can add logic back if UI relies on it.

        if (orcas_attributes.cmds['enable_searchlight'] == 1):
            if (set_searchlight_pwm(orcas_attributes.status['searchlight_duty']) == 0):
                orcas_attributes.status['searchlight_en'] = 1
                orcas_attributes.cmds['enable_searchlight'] = 0
        if (orcas_attributes.cmds['disable_searchlight'] == 1):        
            if (set_searchlight_pwm(0) == 0):
                orcas_attributes.status['searchlight_en'] = 0
                orcas_attributes.cmds['disable_searchlight'] = 0   

        time.sleep(0.2)
