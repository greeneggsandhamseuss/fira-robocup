# configure during competition

#T1
HMAX = 130 
HMID = 115
HMIN = 70
forwardDist = 60 # distance for forward manual
throughDist = 55 # distance for going through gate
MANU_ROT_TIME = 6.5  # time for rotate manual
SIDE_HIGH = 3.8 # time for side manual
SIDE_LOW = 4.9
MANU_TIMEOUT = 20 # override tello's de fault timeouts 

TAGS =[[0,'E','R'],[1,'D','W'],[2,'F','E','U','R'],[3,'D','E'],[4,'L'],[6,'U','R'],[7,'D','R'],[8,'W','U'],[9,'F','L'],[10]]

startH = 'up'

#====================================================

from djitellopy import tello
import time
import cv2 as c
import numpy as np
import at

img_debug = np.full((720,960,3), (255,255,255), np.uint8)

#============================================ intialization part / intial setup

tello.Tello.RESPONSE_TIMEOUT = MANU_TIMEOUT
tello.Tello.TAKEOFF_TIMEOUT = MANU_TIMEOUT

robot = tello.Tello()

robot.connect()
robot.streamoff()
robot.streamon()
robot_frame = robot.get_frame_read(with_queue = False)
#robot.takeoff()

while True:
    img = robot_frame.frame
    if img is not None:
        break

last_time = time.time()
while(time.time() - last_time) < 3.0:
    img = robot_frame.frame
    if img is not None:
        c.imshow('dronecam', img)
        c.waitKey(1)
    robot.send_rc_control(0,0,0,0)

# ============================================ main loop / superloop

state = 'search'
state_search = 'set'
state_manual = 'forward'
aPtr = 0
orgYaw = 0
dAng = 0
rotTime = 0
hDest = 0
tagPtr = 0
lostX = 0
throughZ = 0 
tries = 0
lostY = 0
lostZ = 0
last_time = 0
lostTime = 0

ANG = [60,120,180,240,300,360]  

def angleCorr(angle):
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle  

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

battery = robot.get_battery()
if battery < 40:
    print(f"LOW BATTERY: {battery}%")
    while True:
        None    

while True:
    battery = robot.get_battery()
    temperature = robot.get_temperature()  
    yaw = robot.get_yaw()
    height = robot.get_distance_tof()

    errX, errY, errZ, errR = 0,0,0,0  
    outX, outY, outZ, outR = 0,0,0,0
    seen = False                      

    img = robot_frame.frame  
    if img is not None:
        img = c.cvtColor(img, c.COLOR_RGB2BGR)
        img = c.resize(img, [960, 720])
        img_debug = img.copy()            

        try:
            tags = at.get_tags(img)
            ids = None   
            for tag in tags:   
                if tag[0] == TAGS[tagPtr][0]:  
                    seen = True
                    ids = tag[0]
                    errX = tag[1]
                    errY = tag[2]          
                    errZ = tag[3]
                    throughZ = tag[3]        
                    errR = tag[4]                
                    c.circle(img_debug, (tag[5], tag[6]), 20, (0,0,255), 3)
        except:
            pass  

#===============================================================================================================================

    if state == 'search':
        if state_search == "set":
                if startH == 'up':
                    outX, outY, outZ, outR = 0,50,0,0
                    if height >= HMAX:
                        outX, outY, outZ, outR = 0,0,0,0
                        state_search = 'initial'
                elif startH == 'down':
                    outX, outY, outZ, outR = 0,-50,0,0
                    if height <= HMIN:
                        outX, outY, outZ, outR = 0,0,0,0
                        state_search = 'initial'

        if state_search == 'initial':  
            aPtr = 0
            orgYaw = yaw            
            state_search = 'pick'

        elif state_search == 'pick':
            dAng = ANG[aPtr]
            dAng = dAng + orgYaw
            dAng = angleCorr(dAng)  
            state_search = 'rotate'

        elif state_search == 'rotate':  
            errR = yaw - dAng
            errR = angleCorr(errR)
            outR = errR * -12  
            outR = np.clip(outR, -80, 80)
            if abs(errR) < 5:            
                errR, outR = 0,0
                rotTime = time.time()  
                state_search = 'wait'

        elif state_search == 'wait':  
            if(time.time() - rotTime) > 0.2:
                aPtr += 1            
                if aPtr >= len(ANG):        
                    aPtr = 0
                    state_search = 'heightDecision'
                else:    
                    state_search = 'pick'
       
        elif state_search == 'heightDecision':
            if abs(height - HMAX) < abs(height - HMIN):
                tries += 1
                hDest = HMIN
            else:
                tries += 1
                hDest = HMAX
            if tries >= 2:
                hDest = HMID
                tries -= 3
            state_search = 'heightReach'

        elif state_search == 'heightReach':  
            errY = hDest - height
            outY = errY * 1.7      
            outY = np.clip(outY, -35, 35)
            if abs(errY) < 4:    
                errY, outY = 0,0
                aPtr = 0
                state_search = 'pick'          

        if seen:
            tries = 0
            aPtr = 0
            state_search = 'initial'  
            outX, outR, outY, outZ = 0,0,0,0
            state = 'alignment'

#===============================================================================================================================

    elif state == 'alignment':
        if seen:
            lostX = errX        
            lostY = errY - 10
            lostZ = errZ - 45

            errY = errY - 14            
            outY = errY * -1.6            
            outY = np.clip(outY, -30, 25)
            if errY < 5 and errY > -5:      
                errY, outY = 0,0

            errX = errX - 0            
            outX = errX * 0.7
            outX = np.clip(outX, -28, 28)
            if errX < 5 and errX > -5:
                errX, outX = 0,0

            errZ = errZ - 45        
            outZ = errZ * 1.8              
            outZ = np.clip(outZ, -20, 20)    
            if errZ < 5 and errZ > -5:
                errZ, outZ = 0,0

            errR = errR - 0  
            outR = errR * 1.2            
            outR = np.clip(outR, -35, 35)
            if errR < 5 and errR > -5:
                errR, outR = 0,0

            if errX<8 and errX>-8 and errR<2 and errR>-2 and errY<5 and errY>-5 and errZ<6 and errZ>-6:
                outX, outR, outY, outZ = 0,0,0,0
                state = 'goThrough'

        else:
            state = 'lost'
            lostTime = time.time()

#===============================================================================================================================

    elif state == 'manual':
        if state_manual == 'forward':
            if (time.time() - last_time) < 0.5:
                outX, outY, outZ, outR = 0,0,0,0
            else:
                if 'F' in TAGS[tagPtr-1]:
                    robot.move_forward(forwardDist)
                elif 'B' in TAGS[tagPtr-1]:
                    robot.move_back(20) 
                else:
                    outX, outY, outZ, outR = 0,0,0,0
                    last_time = time.time()
                    state_manual = 'side'
                outX, outY, outZ, outR = 0,0,0,0
                if abs(height - HMAX) < abs(height - HMIN):
                    MANU_SIDE_TIME = SIDE_HIGH
                else:
                    MANU_SIDE_TIME = SIDE_LOW
                last_time = time.time()
                state_manual = 'side'

        elif state_manual == 'side':
            if 'W' in TAGS[tagPtr-1]:
                    outX, outY, outZ, outR = -30,0,0,0  
            elif 'E' in TAGS[tagPtr-1]:
                    outX, outY, outZ, outR = 30,0,0,0 
            else: 
                last_time -= MANU_SIDE_TIME
            if (time.time() - last_time) > MANU_SIDE_TIME:
                outX, outY, outZ, outR = 0,0,0,0
                state_manual = 'height'

        elif state_manual == 'height':
            last_time = time.time()
            if 'U' in TAGS[tagPtr-1]:
                if (height >= HMAX):
                    outX, outY, outZ, outR = 0,0,0,0
                    state_manual = 'rotate'
                else:
                    outX, outY, outZ, outR = 0,35,0,0
            elif 'D' in TAGS[tagPtr-1]:
                if (height <= HMIN):
                    outX, outY, outZ, outR = 0,0,0,0
                    state_manual = 'rotate'
                else:
                    outX, outY, outZ, outR = 0,-35,0,0
                    
            else:
                state_manual = 'rotate'
       
        elif state_manual == 'rotate':
            if 'L' in TAGS[tagPtr-1]:
                outX, outY, outZ, outR = 0,0,0,-40
            elif 'R' in TAGS[tagPtr-1]:
                outX, outY, outZ, outR = 0,0,0,40
            else:
                last_time -= MANU_ROT_TIME

            if(time.time() - last_time) > MANU_ROT_TIME:
                outX, outY, outZ, outR = 0,0,0,0
                state = 'search'
                state_manual = 'forward'    

        if seen:
            state = 'alignment'
            state_manual = 'forward'
            outX, outY, outZ, outR = 0,0,0,0

#===============================================================================================================================

    elif state == 'lost':
        if seen == True:
            outX, outR, outY, outZ = 0,0,0,0
            state = 'alignment'  
                                                                                                               
        elif (time.time() - lostTime) > 10.0:
            outY, outX, outR, outZ = 0,0,0,0
            state = 'search'
        else:
            if lostX < -7:
                outR = -20  
            elif lostX > 7:
                outR = 20          
            else:
                outR = 0

            if lostY > 0:
                outY = -20    
            elif lostY < -10:
                outY = 0  

            if lostZ < 0:
                outZ = -15        
            elif lostZ > 10:
                outZ = 0

#===============================================================================================================================

    elif state == 'goThrough':  
        move(robot, 'forward', throughZ + throughDist)
        outX, outR, outY, outZ = 0,0,0,0
        tagPtr += 1                    
        if tagPtr >= len(TAGS):
            break        
        else:
            state = 'manual'
            last_time = time.time()

    robot.send_rc_control(int(outX),int(outZ),int(outY),int(outR))

    c.putText(img_debug, f'state: {state}', (20,30), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'seen: {seen}', (20,70), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'outY: {outY}', (20,110), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errY: {errY}', (250,110), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'outX: {outX}', (20,150), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errX: {errX}', (250,150), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'outZ: {outZ}', (20,190), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errZ: {errZ}', (250,190), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'outR: {outR}', (20,230), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errR: {errR}', (250,230), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'throughZ: {throughZ}', (20,270), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'temperature: {temperature}', (20,310), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'height: {height}', (20,350), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'battery: {battery}', (20,390), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'state_manual: {state_manual}', (20,430), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'last_time: {last_time}', (20,470), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'time.time(): {time.time()}', (20,510), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)

    c.imshow('dronecam', img_debug)  
    key = c.waitKey(1)
    if key == 27:              
        break

robot.land()  
