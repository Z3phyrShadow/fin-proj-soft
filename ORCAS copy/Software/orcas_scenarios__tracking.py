import cv2
import numpy as np
import math
import orcas_attributes

from ultralytics import YOLO

# Export your model to int8 ncnn or use .pt. This assumes .pt.
model = YOLO('yolo11n.pt')

status = {'state': 'standby', # 'standby', 'tilt_homing', 'searching', 'analyzing'
          'standby_timeout': 5,
          'searching_period': 0.4, # Adjusted loop rates are usually constrained by YOLO FPS
          'searching_timeout': 2.0,          
          'firing_period': 1.6,
          'saved_fire_mode': 'SAFE',
          'saved_aim_icon_size': 0,
          'saved_static_icon_size': 0}
                
PID_params = {'Kp': [0.4, 0.4],
              'Ki': [0.02, 0.02],
              'Kd': [0.08, 0.08],
              'I_term_prev': [0, 0],
              'offset_prev': [0, 0]}
              
timer1, timer2, timer3 = 0, 0, 0

def init():
    global timer1
       
    status['saved_fire_mode'] = orcas_attributes.status['fire_mode']
    if (status['saved_fire_mode'] == 'SAFE'):
        fire_mode_button_img = "FireMode_Safe.bmp"
    elif (status['saved_fire_mode'] == 'SEMI'):
        fire_mode_button_img = "FireMode_Semi.bmp"
    else:
        fire_mode_button_img = "FireMode_Burst.bmp"
    orcas_attributes.cmds['enable_safety'] = 1
    orcas_attributes.status['fire_mode'] = 'SAFE'        
    
    if(orcas_attributes.status['camera_zoom_level'] == 1): 
        zoom_button_img = "Zoom_x1.bmp"
    elif(orcas_attributes.status['camera_zoom_level'] == 2): 
        zoom_button_img = "Zoom_x2.bmp"
       
    orcas_attributes.cmds['disable_searchlight'] = 1
    searchlight_button_img = "Searchlight_disable.bmp"    
    
    radar_button_img = "Radar_disable.bmp"
    
    status['saved_aim_icon_size'] = orcas_attributes.status['aim_icon_size']
    status['saved_static_icon_size'] = orcas_attributes.status['static_icon_size']
    orcas_attributes.status['aim_icon_size'] = 40
    orcas_attributes.status['static_icon_size'] = 20            
    status['state'] = 'standby'
    timer1 = 0
    
    return fire_mode_button_img, zoom_button_img, searchlight_button_img, radar_button_img
        
def de_init():
    orcas_attributes.cmds['disable_searchlight'] = 1
    orcas_attributes.status['fire_mode'] = status['saved_fire_mode']
    if (orcas_attributes.status['fire_mode'] == 'SAFE'):
        orcas_attributes.cmds['enable_safety'] = 1
    else:
        orcas_attributes.cmds['disable_safety'] = 1
    orcas_attributes.status['aim_icon_size'] = status['saved_aim_icon_size']
    orcas_attributes.status['static_icon_size'] = status['saved_static_icon_size']
    
def toggle_analysis():
    if (status['state'] == 'analyzing'):
        init()
    else:
        orcas_attributes.cmds['enable_safety'] = 1
        orcas_attributes.cmds['enable_searchlight'] = 1
        status['state'] = 'analyzing'

def analyze_target(frame):
    # Analyzing state is stubbed since HSV color thresholding is not required for YOLO
    pass

def detect_target(frame):
    results = model(frame, imgsz=320, conf=0.4, verbose=False)
    for r in results:
        boxes = r.boxes
        if len(boxes):
            b = boxes[0] # take highest confidence box
            x1,y1,x2,y2 = map(int, b.xyxy[0])
            cx = (x1+x2)//2
            cy = (y1+y2)//2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            return frame, cx, cy
    return frame, -1, -1
    
def PID_ctrl(offset_x, offset_y):
    offset = [offset_x, offset_y]
    P_term = [0, 0]
    I_term = [0, 0]
    D_term = [0, 0]
    output = [0, 0]
    
    for i in range(0, 2, 1):
        P_term[i] = offset[i] * PID_params['Kp'][i]
        I_term[i] = PID_params['I_term_prev'][i] + offset[i] * PID_params['Ki'][i]
        PID_params['I_term_prev'][i] = I_term[i]
        D_term[i] = (offset[i] - PID_params['offset_prev'][i]) * PID_params['Kd'][i]
        PID_params['offset_prev'][i] = offset[i]        
        output[i] = P_term[i] + I_term[i] + D_term[i]

    return output[0], output[1]
    
def attach_info(frame):
    global timer1, timer2, timer3
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 4
    color = (0, 255, 0) 
    thickness = 4 
    
    if (status['state'] == 'standby'):
        timer1 = timer1 + 1
        if (timer1 > status['standby_timeout'] // orcas_attributes.status['frame_receiving_period']):
            status['state'] = 'tilt_homing'
            orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']
        else:
            text_val = status['standby_timeout'] - int(timer1 * orcas_attributes.status['frame_receiving_period'])
            text = f"{text_val}"
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            text_x_coor = frame.shape[1] // 2 - text_size[0] // 2 
            text_y_coor = frame.shape[0] // 2 + text_size[1] // 2 
            cv2.putText(frame, text, (text_x_coor, text_y_coor), font, font_scale, color, thickness)                
    elif (status['state'] == 'tilt_homing'):
        if (abs(orcas_attributes.status['aeg_tilt_angle']) < 1):
            status['state'] = 'searching' 
            orcas_attributes.cmds['enable_searchlight'] = 1         
            timer1, timer2, timer3 = 0, 0, 0
            PID_params['I_term_prev'] = [0, 0]
            PID_params['offset_prev'] = [0, 0]
        else:
            orcas_attributes.cmds['tilt_homing'] = 1
    elif (status['state'] == 'searching'):                
        timer1 = timer1 + 1
        if (timer1 >= status['searching_period'] // orcas_attributes.status['frame_receiving_period']):
            timer1 = 0
            frame, target_coor_x, target_coor_y = detect_target(frame)                       
            if ((target_coor_x >= 0) and (target_coor_y >= 0)):               
                timer2 = 0
                if (status['saved_fire_mode'] != 'SAFE'):
                    orcas_attributes.status['fire_mode'] = status['saved_fire_mode'] 
                    orcas_attributes.cmds['disable_safety'] = 1
                
                aim_icon_x = orcas_attributes.status['aim_icon_coor']['x'] - orcas_attributes.status['displayed_frame_origin_coor']['x']
                aim_icon_y = orcas_attributes.status['aim_icon_coor']['y'] - orcas_attributes.status['displayed_frame_origin_coor']['y']

                exec_x, exec_y = PID_ctrl(target_coor_x - aim_icon_x, target_coor_y - aim_icon_y)
                
                touch_x = int(exec_x) + aim_icon_x
                if (touch_x < 0):
                    touch_x = 0
                if (touch_x > orcas_attributes.status['displayed_frame_size']['width']):
                    touch_x = orcas_attributes.status['displayed_frame_size']['width']
                
                touch_y = int(exec_y) + aim_icon_y
                if (touch_y < 0):
                    touch_y = 0
                if (touch_y > orcas_attributes.status['displayed_frame_size']['height']):
                    touch_y = orcas_attributes.status['displayed_frame_size']['height']                                   
                
                orcas_attributes.cmds['touch_coor']['x'] = touch_x
                orcas_attributes.cmds['touch_coor']['y'] = touch_y
                orcas_attributes.cmds['clear_coor'] = 1 
                
                if ((abs(target_coor_x - aim_icon_x) < orcas_attributes.status['aim_icon_size']) and 
                    (abs(target_coor_y - aim_icon_y) < orcas_attributes.status['aim_icon_size'])):
                    if (timer3 == status['firing_period'] // status['searching_period']):
                        orcas_attributes.cmds['direct_fire'] = 1
                        timer3 = 0
            else:                
                timer2 = timer2 + 1
                PID_params['I_term_prev'][0] = 0
                PID_params['I_term_prev'][1] = 0                
                if (timer2 > status['searching_timeout'] // status['searching_period']):
                    status['state'] = 'tilt_homing'
                    orcas_attributes.status['fire_mode'] = 'SAFE' 
                    orcas_attributes.cmds['enable_safety'] = 1   
                    orcas_attributes.cmds['tilt_angle'] = -orcas_attributes.status['aeg_tilt_angle']   
                    orcas_attributes.cmds['disable_searchlight'] = 1
            
            if (timer3 < status['firing_period'] // status['searching_period']):
                timer3 = timer3 + 1            
    elif (status['state'] == 'analyzing'):
        analyze_target(frame)
        
    return frame