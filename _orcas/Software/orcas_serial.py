import time
import serial

cmd_header = 0x5A
cmd_code = {'GET_FW_VER':                   0x01,
            'GET_PAN_TILT_STEP_ANGLE':      0x08,
            'GET_PAN_TILT_CURR_ANGLE':      0x09,
            'SET_PAN_TILT_ROTATE_ANGLE':    0x0A,
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
                        print(f"orcas_serial.init(): {ack_code.get(received_data[ack_code_index], 'Unknown Error')}")
                else:
                    print("orcas_serial.init(): Invalid received ack header")
            else:
                print("orcas_serial.init(): Invalid received ack checksum")
        else:
            print("orcas_serial.init(): Invalid received ack length")
        orcas_serial.close()
        return -1
    else:
        print("orcas_serial.init(): Serial port not open")
        return -1
    
def get_pan_tilt_step_angle():
    global orcas_serial

    if orcas_serial and orcas_serial.is_open:
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
                        angle_pan = ((received_data[ack_data_base_index] << 8) | (received_data[ack_data_base_index + 1])) / 1000
                        angle_tilt = ((received_data[ack_data_base_index + 2] << 8) | (received_data[ack_data_base_index + 3])) / 1000
                        return 0, angle_pan, angle_tilt
                    else:
                        print(f"orcas_serial.get_pan_tilt_step_angle(): {ack_code.get(received_data[ack_code_index], 'Unknown Error')}")
                else:
                    print("orcas_serial.get_pan_tilt_step_angle(): Invalid received ack header")
            else:
                print("orcas_serial.get_pan_tilt_step_angle(): Invalid received ack checksum")
        else:
            print("orcas_serial.get_pan_tilt_step_angle(): Invalid received ack length")
    return -1, 0, 0
    
def get_pan_tilt_curr_angle():
    global orcas_serial

    if orcas_serial and orcas_serial.is_open:
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
                        print(f"orcas_serial.get_pan_tilt_curr_angle(): {ack_code.get(received_data[ack_code_index], 'Unknown Error')}")
                else:
                    print("orcas_serial.get_pan_tilt_curr_angle(): Invalid received ack header")
            else:
                print("orcas_serial.get_pan_tilt_curr_angle(): Invalid received ack checksum")
        else:
            print("orcas_serial.get_pan_tilt_curr_angle(): Invalid received ack length")
    return -1, 0, 0
    
def set_pan_tilt_rotate_angle(pan_angle, tilt_angle):
    global orcas_serial
            
    if orcas_serial and orcas_serial.is_open:
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
        
        received_data = orcas_serial.read(ack_header_code_checksum_length)                
        if len(received_data) == ack_header_code_checksum_length:
            checksum = sum(received_data[:-1]) & 0xFF
            if checksum == received_data[-1]:
                if received_data[ack_header_index] == ack_header:                       
                    if received_data[ack_code_index] == 0x00:                         
                        return 0
                    else:
                        print(f"orcas_serial.set_pan_tilt_rotate_angle(): {ack_code.get(received_data[ack_code_index], 'Unknown Error')}")
                else:
                    print("orcas_serial.set_pan_tilt_rotate_angle(): Invalid received ack header")
            else:
                print("orcas_serial.set_pan_tilt_rotate_angle(): Invalid received ack checksum")
        else:
            print("orcas_serial.set_pan_tilt_rotate_angle(): Invalid received ack length")
    return -1

def get_aiming_distance():
    global orcas_serial

    if orcas_serial and orcas_serial.is_open:
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

def kill():
    global orcas_serial
    if orcas_serial and orcas_serial.is_open:
        orcas_serial.close()
