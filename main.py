#============================================================================
#############################################################################
#============================================================================

from djitellopy import tello
from utils import AirBrake, FrameFreezeDetector, KeyboardController
import at
import cv2
import numpy as np
import time
import logging

#============================================================================
################################## CONFIG ###################################
#============================================================================

# [l r f b],CM
# h, index of height
# c, +/-angle
# TAGS = [[7,['f',50],['r',80],['h',0],['c',180]],[8],[0],[4]]
TAGS = [[7,['h',0],['c',30]],[2],[8,['f',35],['h',2],['c',-90]],[9,['f',60],['h',0],['c',-130]],[5,['c',-90]],[3]]

# 0 or 1 or 2 as index
FIRST_GATE = 2
# heights as CM
HEIGHTS = [85 , 130 , 150]

# through as CM, for low, mid and high gates
THROUGH = [60, 60, 60] #LOW, MID, HIGH

#============================================================================
################################### SETUP ###################################
#============================================================================

DEBUG = True
FLY = True

SETPOINT = {'x':0, 'y':-6, 'z':45, 'r':0}
KP = {'x':1.5, 'y':3.0, 'z':2.0, 'r':2.0}
MARGIN = {'x':{'+':8,'-':-8} , 'y':{'+':5,'-':-8} , 'z':{'+':5,'-':-8} , 'r':{'+':5,'-':-5}}
CLIP = {'x':{'+':30,'-':-30} , 'y':{'+':30,'-':-60} , 'z':{'+':25,'-':-60} , 'r':{'+':30,'-':-30}}

ROT_PAUSE = 0.7
SEARCH = [["r",[20,-40,0]],["h"],["h"],["h"],["r",[30,60,90,135,225,270,300,330,360]],["h"],["r",[30,60,90,135,225,270,300,330,360]],["h"],["r",[30,60,90,135,225,270,300,330,360]],["h"]]

#============================================================================
################################# VARIABLES #################################
#============================================================================

img_debug = np.full((720,960,3), (255,255,255), np.uint8)
tag_now = 0
lost_mode = False
lost_x = 0
lost_y = 0
lost_time = 0
search_dir = 1
search_nobat = 0
search_time = 0
alakiY = 0
final_nextY = 0
final_nextR = 0
tempDir = 0
time_tu = 0
tu_en = False
outX_save = 0
search_yaw = 0
bala_paiin = True
yaw_alaki = 0
search_h = 0
nextY = 0
rot_ptr = 0
cx, cy = 0,0
lastYaw = 0

#============================================================================
################################### FUNC ####################################
#============================================================================

def move(r, dir, dist):

    while True:
        t = time.time()

        if dir.strip().lower().startswith('l'):
            r.move('left' , dist)
        elif dir.strip().lower().startswith('r'):
            r.move('right' , dist)
        elif dir.strip().lower().startswith('f'):
            r.move('forward' , dist)
        elif dir.strip().lower().startswith('b'):
            r.move('back' , dist)
        elif dir.strip().lower().startswith('u'):
            r.move('up' , dist)
        elif dir.strip().lower().startswith('d'):
            r.move('down' , dist)

        if (time.time() - t) > 0.6:
            break

#============================================================================

def angle(d):
    if d > 180 :
        d -= 360
    elif d < -180 :
        d += 360
    return d

#============================================================================

def read_video_frame(frame_reader, detector, rc_command, telemetry):
    """Read, check, and convert the latest DJITelloPy frame."""
    raw_frame = frame_reader.frame
    frame_frozen = detector.is_frozen(raw_frame, rc_command, telemetry)

    if (
        not isinstance(raw_frame, np.ndarray)
        or raw_frame.ndim != 3
        or raw_frame.shape[2] < 3
    ):
        return None, frame_frozen

    try:
        video_frame = cv2.cvtColor(raw_frame, cv2.COLOR_RGB2BGR)
        return cv2.resize(video_frame, (960, 720)), frame_frozen
    except (cv2.error, TypeError, ValueError):
        return None, frame_frozen


#============================================================================
################################### INIT ####################################
#============================================================================

robot = tello.Tello()
robot.LOGGER.setLevel(logging.ERROR)
robot.connect() 
robot.streamoff()
robot.streamon()
robot_frame = robot.get_frame_read(with_queue=False)
if FLY:
    robot.takeoff()
while True:
    img = robot_frame.frame
    if img is not None:
        break
last_time = time.time()
while (time.time()-last_time) < 3.0:
    img = robot_frame.frame
    if img is not None:
        cv2.imshow('img',img)
        cv2.waitKey(1)
    robot.send_rc_control(0,0,0,0)

#============================================================================
################################### FUNC ####################################
#============================================================================

state = 'FIRST'
outX_save = 0
gate_start = False
c = {'x':0,'y':0}
axis = ["x","y","z","r"]
err = {'x':0,'y':0,'z':0,'r':0}
last = {'x':0,'y':0,'z':0,'r':0}
out = {'x':0,'y':0,'z':0,'r':0}
lock = {'x':False,'y':False,'z':False,'r':False}
lastYaw = 0
time_through = 0
step = 0
step2 = 0
state_sub = 'CHECK'
air_brake_is_finished = True
last_rc_out = {'x':0,'y':0,'z':0,'r':0}
freeze_detector = FrameFreezeDetector()
air_brake = AirBrake()
keyboard = KeyboardController()

while True:
    telemetry = robot.get_current_state().copy()
    battery = robot.get_battery()
    height = robot.get_distance_tof()
    yaw = robot.get_yaw()
    temperature = robot.get_temperature()
    video_frame, frame_frozen = read_video_frame(robot_frame, freeze_detector, last_rc_out, telemetry)

    #----------------------------------------------
    seen = False
    vision_processing_failed = False
    err = {'x':0,'y':0,'z':0,'r':0}
    out = {'x':0,'y':0,'z':0,'r':0}
    lock = {'x':False,'y':False,'z':False,'r':False}
    c['x'] = 0
    c['y'] = 0
    #----------------------------------------------

   
    if video_frame is not None and not frame_frozen:
        try:
            tags = at.get_tags(video_frame)
            for tag in tags:
                if tag[0] == TAGS[tag_now][0]:
                    seen = True
                    lastYaw = yaw
                    err['x'] = tag[1]
                    err['y'] = tag[2]
                    err['z'] = tag[3]
                    err['r'] = tag[4]
                    c['x'] = tag[5]
                    c['y'] = tag[6]
                    break
        except Exception as tag_error:
            vision_processing_failed = True
            print(f"******* APRILTAG BUG: {type(tag_error).__name__} *******")


    #****************************************** FIRST HEIGHT

    if state == 'FIRST':
        e = HEIGHTS[FIRST_GATE] - height
        out['y'] = e * 2.0
        out['y'] = np.clip(out['y'],-40,40)
        if abs(e) < 10 or seen:
            out = {'x':0,'y':0,'z':0,'r':0}
            last_time = time.time()
            state = 'FOLLOW'

    #****************************************** GOTO TAG

    elif state == 'FOLLOW':
        if seen:
            air_brake_is_finished = air_brake.is_finished(telemetry, yaw, stop_requested=False)
            for key in axis:
                err[key] = err[key] - SETPOINT[key]
                last[key] = err[key]
                if err[key] > MARGIN[key]['-'] and err[key] < MARGIN[key]['+']:
                    out[key] = 0
                    lock[key] = True
                else:
                    out[key] = int(err[key] * KP[key])
                    out[key] = np.clip(out[key], CLIP[key]['-'], CLIP[key]['+'])
            
            if all(lock.values()):
                out = {'x':0,'y':0,'z':0,'r':0}
                time_through = last['z'] + SETPOINT['z'] + THROUGH[HEIGHTS.index(min((HEIGHTS[0], HEIGHTS[-1]), key=lambda n: abs(n - height)))]
                state = 'THROUGH'
        
        else:
            air_brake_is_finished = air_brake.is_finished(telemetry, yaw)
            if air_brake_is_finished:
                state = 'LOST'
                pass

        last_time = time.time()
            
    #****************************************** LOST MODE

    elif state == 'LOST':

        if seen == True:
            out = {'x':0,'y':0,'z':0,'r':0}
            state = 'FOLLOW'
        
        elif (time.time() - last_time) > 2.0:
            out = {'x':0,'y':0,'z':0,'r':0}
            last_time = time.time()      
            state = 'SEARCH'
    
        else:
            if last['x'] > MARGIN['x']['+']:
                out['r'] = 15
            elif last['x'] < MARGIN['x']['-']:
                out['r'] = -15
            #......
            if last['y'] < 0:
                out['y'] = -30
            #......
            #out['z'] = -20

    #****************************************** THROUGH

    elif state == 'THROUGH':
            out = {'x':0,'y':0,'z':0,'r':0}
            if (time.time() - last_time) > 0.5:
                air_brake_is_finished = air_brake.is_finished(telemetry, yaw)
                if air_brake_is_finished:
                    move(robot, 'forward', time_through)
                    tag_now += 1
                    if tag_now >= len(TAGS):
                        robot.land()
                        break
                    last_time = time.time()
                    state = 'OVERRIDE'

    #****************************************** OVERRIDE

    elif state == 'OVERRIDE':

        if state_sub == 'CHECK':
            step += 1
            if step < len(TAGS[tag_now-1]):
                try:
                    act = TAGS[tag_now-1][step][0].lower()
                    val = TAGS[tag_now-1][step][1]

                    if act in ["l","r","f","b"]:
                        move(robot, act, val)

                    elif act == 'h':
                        state_sub = 'HEIGHT'

                    elif act == 'c':
                        lastYaw = angle(yaw + val)
                        state_sub = 'ROTATE'
                except:
                    pass
            else:
                out = {'x':0,'y':0,'z':0,'r':0}
                step = 0
                state_sub = 'CHECK'
                state = 'SEARCH'


        elif state_sub == 'HEIGHT':
            try:
                e = HEIGHTS[TAGS[tag_now-1][step][1]] - height
                out['y'] = e * 2.0
                out['y'] = np.clip(out['y'],-40,40)
                if abs(e) < 10:
                    out['y'] = 0
                    state_sub = 'CHECK'
            except:
                state_sub = 'CHECK'


        elif state_sub == 'ROTATE':
            e = angle(lastYaw - yaw)
            out['r'] = e * 1.5
            out['r'] = np.clip(out['r'],-30,30)
            if abs(e) < 5:
                out['r'] = 0
                lastYaw = yaw
                state_sub = 'CHECK'

        
        if seen:
            out = {'x':0,'y':0,'z':0,'r':0}
            step = 0
            state_sub = 'CHECK'
            state = 'FOLLOW'


    #****************************************** SEARCH

 
    elif state == 'SEARCH':
        if state_sub == 'CHECK':
            if SEARCH[step][0] == "r":
                state_sub = 'ROTATE'

            elif SEARCH[step][0] == "h":
                step2 = (min(range(len(HEIGHTS)), key=lambda i: abs(HEIGHTS[i] - height)) + 1) % len(HEIGHTS)
                state_sub = 'HEIGHT'    

        #........................

        elif state_sub == 'ROTATE':
            e = angle(angle(lastYaw + SEARCH[step][1][step2]) - yaw)
            out['r'] = e * 4.0
            out['r'] = np.clip(out['r'],-100,100)
            if abs(e) < 5:
                out['r'] = 0
                last_time = time.time()
                state_sub = 'PAUSE'

        #.........................

        elif state_sub == 'HEIGHT':
            e = HEIGHTS[step2] - height
            out['y'] = e * 2.0
            out['y'] = np.clip(out['y'],-40,40)
            if abs(e) < 10:
                out['y'] = 0
                last_time = time.time()
                state_sub = 'PAUSE'

        #........................

        elif state_sub == 'PAUSE':
            if (time.time() - last_time) > ROT_PAUSE:
                if len(SEARCH[step]) > 1:
                    step2 += 1
                    if step2 >= len(SEARCH[step][1]):
                        step2 = 0
                        step += 1
                else:
                    step2 = 0
                    step += 1

                if step >= len(SEARCH):
                    step = 0
                state_sub = 'CHECK'

        #........................

        if seen:
            out = {'x':0,'y':0,'z':0,'r':0}
            step = 0
            step2 = 0
            state_sub = 'CHECK'
            state = 'FOLLOW'


    #****************************************** LANDING


    elif state == 'LAND':
        #*********************** avazi *********************
        #move(robot, 'left', 50)
        #***************************************************
        break

        
    #------------------------------------------------------------------------ KEYBOARD

    key = -1
    manual_command = None
    if DEBUG:
        key, manual_command = keyboard.poll()

        if key == ord('e') or key == ord('E'):
            state = 0
            keyboard.reset()
            manual_command = None
        elif key == ord('r') or key == ord('R'):
            state = 'FOLLOW'
            state_sub = 'CHECK'
            step = 0
            step2 = 0
            last_time = time.time()
            keyboard.reset()
            manual_command = None

    if key == ord('q') or key == ord('Q'):
        break
    
    if DEBUG and manual_command is not None:
        out = manual_command

    #------------------------------------------------------------------------ RC

    out = air_brake.apply(out, telemetry, yaw)
    robot.send_rc_control(int(out['x']),int(out['z']),int(out['y']),int(out['r']))
    last_rc_out = {key:int(out[key]) for key in axis}

    #------------------------------------------------------------------------ DEBUG

    if video_frame is not None:
        img_debug = video_frame.copy()

    if DEBUG:
        cv2.putText(img_debug, f"S:{state} {state_sub} {step} {step2}", (20,620), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), 3)
        cv2.putText(img_debug, f"X LAST:{int(last['x'])} | ERR:{int(err['x'])} | OUT:{last_rc_out['x']}", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1.15, (0, 255, 0) if lock['x'] else (0, 0, 255), 3)
        cv2.putText(img_debug, f"Y LAST:{int(last['y'])} | ERR:{int(err['y'])} | OUT:{last_rc_out['y']}", (20,100), cv2.FONT_HERSHEY_SIMPLEX, 1.15, (0, 255, 0) if lock['y'] else (0, 0, 255), 3)
        cv2.putText(img_debug, f"Z LAST:{int(last['z'])} | ERR:{int(err['z'])} | OUT:{last_rc_out['z']}", (20,160), cv2.FONT_HERSHEY_SIMPLEX, 1.15, (0, 255, 0) if lock['z'] else (0, 0, 255), 3)
        cv2.putText(img_debug, f"R LAST:{int(last['r'])} | ERR:{int(err['r'])} | OUT:{last_rc_out['r']}", (20,220), cv2.FONT_HERSHEY_SIMPLEX, 1.15, (0, 255, 0) if lock['r'] else (0, 0, 255), 3)
        cv2.putText(img_debug, f"H:{int(height)} Y:{int(yaw)} LY:{int(lastYaw)}", (20,280), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
        cv2.putText(img_debug, f"TIME THROUGH:{time_through:.2f}", (20,340), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,255), 3)
        cv2.putText(img_debug, f"BRAKE FINISHED:{air_brake_is_finished}", (500,280), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,200,0) if air_brake_is_finished else (0,165,255), 2)
        cv2.putText(img_debug, f"FRAME FROZEN: {frame_frozen}", (500,230), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,0,255) if frame_frozen else (0,200,0), 2)
    
    cv2.circle(img_debug, (c['x'],c['y']), 20, (0, 255, 0) if all(lock.values()) else (0, 0, 255), 3)
    cv2.putText(img_debug, f"B {battery}% T {temperature}C", (20,680), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 3)
    cv2.imshow('img', img_debug)

robot_frame.stop()
robot.land()
