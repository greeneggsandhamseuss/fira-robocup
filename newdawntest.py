# -----List of Equipment-----

# ==(BACKPACK)==

# Laptop
# 2 c-port power adapter
# Phone
# Earbuds
# C-port / a-port wire 
# C-port / c-port wire
# Lunchbag
# Computer charger (c-port)

# ==(DRONE BOX)==

# Drone
# 4 batteries 
# 3 slot battery charger
# C-port / d-port wire

# ==(TEAM BOX)==

# Fan box (Fan &&& inverted-USB / outverted-USB wire)
# Antennae
# 1 slot battery charger
# 4 slot battery charger 
# 4 slot battery charger's wire, 2 circles on one end, wall outlet charger type on other end

#====================================================================

# List of problems and concerns / notes

# Want to somehow shorten time when the robot first takes off and has to wait a while to "load"
# Tune PID and margin of error for the start goThrough state (needs heavy tuning I think cuz it's pretty bad)
# Randomly started going up and not stopping, had to do emergency ESC, only happened one single time (very rare)
# Need three levels for search
# Sometimes goes inconsistnetly through gates, sometimes goes sort of close but I think that's a tuning problem 

# Overall, works pretty well and consistently completes the course

#============================================================

# configure during competition if necessary:

HMAX = 150
HMIN = 90 

TAGS = [[9,'D','L'], [2,'U','L'], [0,'L'], [5]]    

MANU_ROT_TIME = 5.0

#=================================================

from djitellopy import tello
import time         
import cv2 as c         
import numpy as np        
import at                    

img_debug = np.full((720,960,3), (255,255,255), np.uint8)

#============================================ intialization part / intial setup

robot = tello.Tello()    
robot.connect()    
robot.streamoff() 
robot.streamon()      
robot_frame = robot.get_frame_read(with_queue = False) 
robot.takeoff()    

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
state_search = 'initial'
state_gT = 'stabalize'
state_manual = 'height'
aPtr = 0
orgYaw = 0
dAng = 0
rotTime = 0
hDest = 0
tagPtr = 0
lostX = 0
lostY = 0
lostZ = 0
last_time = 0 
lostTime = 0 
pauseTime = 0

ANG = [60,120,180,240,300,360]   

def angleCorr(angle):
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle

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

        tags = at.get_tags(img)
        ids = None
        for tag in tags:
            try: 
                if tag[0] == TAGS[tagPtr][0]:  
                    seen = True 
                    ids = tag[0]
                    errX = tag[1]
                    errY = tag[2]           
                    errZ = tag[3]              
                    errR = tag[4]                
                    c.circle(img_debug, (tag[5], tag[6]), 20, (0,0,255), 3)
            except:
                pass  

    if state == 'search':

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
            outR = errR * -10    
            outR = np.clip(outR, -80, 80) 
            if abs(errR) < 5:             
                errR, outR = 0,0
                rotTime = time.time()  
                state_search = 'wait'

        elif state_search == 'wait':   
            if(time.time() - rotTime) > 0.3:
                aPtr += 1            
                if aPtr >= len(ANG):        
                    aPtr = 0
                    state_search = 'heightDecision'
                else:    
                    state_search = 'pick'
        
        elif state_search == 'heightDecision':
            if abs(height - HMAX) < abs(height - HMIN):
                hDest = HMIN
            else: 
                hDest = HMAX
            state_search = 'heightReach'

        elif state_search == 'heightReach':  
            errY = hDest - height
            outY = errY * 1.2      
            outY = np.clip(outY, -35, 35)     
            if abs(errY) < 8:    
                errY, outY = 0,0
                aPtr = 0
                state_search = 'pick'           

        if seen:
            aPtr = 0
            state_search = 'initial'  
            outX, outR, outY, outZ = 0,0,0,0
            state = 'alignment'

    elif state == 'alignment':
        if seen:
            lostX = errX        
            lostY = errY - 10
            lostZ = errZ - 40

            errY = errY - 15            
            outY = errY * -1.7            
            outY = np.clip(outY, -30, 25) 
            if errY < 5 and errY > -5:      
                errY, outY = 0,0 

            errX = errX - 0            
            outX = errX *  0.8    
            outX = np.clip(outX, -35, 35)
            if errX < 5 and errX > -5:
                errX, outX = 0,0 

            errZ = errZ - 40          
            outZ = errZ * 1.5               
            outZ = np.clip(outZ, -20, 15)     
            if errZ < 5 and errZ > -5:
                errZ, outZ = 0,0 

            errR = errR - 0  
            outR = errR * 1            
            outR = np.clip(outR, -35, 35)
            if errR < 5 and errR > -5:
                errR, outR = 0,0 

            if errX<6 and errX>-7 and errR<2 and errR>-2 and errY<5 and errY>-5 and errZ<7 and errZ>-5:
                outX, outR, outY, outZ = 0,0,0,0
                state = 'goThrough'
                last_time = time.time()     

        else:
            state = 'lost'
            lostTime = time.time()

    elif state == 'manual':

        if state_manual == 'height':
            last_time = time.time()
            if 'U' in TAGS[tagPtr-1]:
                if (height >= HMAX):
                    outX, outY, outZ, outR = 0,0,0,0
                    state_manual = 'rotate'
                else: 
                    outX, outY, outZ, outR = 0,30,0,0
            elif 'D' in TAGS[tagPtr-1]:
                if (height <= HMIN):
                    outX, outY, outZ, outR = 0,0,0,0
                    state_manual = 'rotate'
                else:
                    outX, outY, outZ, outR = 0,-30,0,0
            else:
                state_manual = 'rotate'
        
        elif state_manual == 'rotate':
            if 'L' in TAGS[tagPtr-1]:
                outX, outY, outZ, outR = 0,0,0,-33
            elif 'R' in TAGS[tagPtr-1]:
                outX, outY, outZ, outR = 0,0,0,33
            else:
                last_time += MANU_ROT_TIME

            if(time.time() - last_time) > MANU_ROT_TIME:
                outX, outY, outZ, outR = 0,0,0,0
                state = 'search'    
                state_manual = 'height'

        if seen:
            state = 'alignment'
            state_manual = 'height'
            outX, outY, outZ, outR = 0,0,0,0

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

    elif state == "goThrough": 

        if state_gT == 'stabalize': 

            '''New state that adds a brief period of 
            time for the robot to recover from moving / activity during alignment; added 
            because without this state, it tries to go through immediately 
            but is still wobbly so it goes in at inconsistent times each gate + the timer for 
            gothrough has already started (which either leaves the robot too much 
            time or too little time and will easily hit the gate (also had to interfere
            a couple of times before figuring out the problem))'''

            if (time.time() - last_time) < 0.8: 
                outX, outR, outY, outZ = 0,0,0,0  
            else:
                state_gT = 'through'
                last_time = time.time()     

        elif state_gT == 'through':
            if (time.time() - last_time) < 3.5: 
                outX, outR, outY, outZ = 0,0,0,30  
            else:
                outX, outR, outY, outZ = 0,0,0,0
                tagPtr += 1                    
                if tagPtr >= len(TAGS):
                    break        
                else:
                    pauseTime = time.time()
                    state_gT = 'pause'

        elif state_gT == 'pause':
            if (time.time() - pauseTime) < 1.0: 
                outX, outR, outY, outZ = 0,0,0,0      
            else:
                state = 'manual'                      
                state_gT = 'stabalize'           

    robot.send_rc_control(int(outX),int(outZ),int(outY),int(outR))

    c.putText(img_debug, f'state: {state}', (20,30), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    #c.putText(img_debug, f'state: {state_search}', (300,30), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'seen: {seen}', (20,70), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'tagPtr: {tagPtr}', (20,110), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'outX: {outX}', (200,150), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'outY: {outY}', (200,190), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'outZ: {outZ}', (200,230), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'outR: {outR}', (200,270), c.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2)
    c.putText(img_debug, f'errX: {errX}', (20,150), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errY: {errY}', (20,190), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errZ: {errZ}', (20,230), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errR: {errR}', (20,270), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'battery: {battery}', (20,310), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'temperature: {temperature}', (20,350), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)

    c.imshow('dronecam', img_debug)  
    key = c.waitKey(1)
    if key == 27:              
        break

robot.land()  