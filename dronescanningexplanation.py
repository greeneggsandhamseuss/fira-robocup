# configure during competition if necessary:

HMAX = 160  # upper height bound (cm) for search sweeps
HMIN = 100  # lower height bound (cm) for search sweeps

from djitellopy import tello   # library to control the Tello drone
import time                    # for timestamps / delays
import cv2 as c                 # OpenCV, used for image processing + display
import numpy as np              # math / array operations (clipping, etc)
import at                       # custom module that detects AprilTags in a frame

img_debug = np.full((720,960,3), (255,255,255), np.uint8)
# creates a blank white placeholder image (720 tall x 960 wide, 3 color channels)
# used before the camera has produced a real frame yet

#============================================ intialization part / intial setup

robot = tello.Tello()      # create the Tello object
robot.connect()             # connect to the drone over wifi
robot.streamoff()           # turn video stream off first...
robot.streamon()                 # ...then back on (avoids a common Tello stream bug)
robot_frame = robot.get_frame_read(with_queue = False)  # background frame reader
robot.takeoff()              # take off

while True: 
    img = robot_frame.frame       # grab the latest camera frame
    if img is not None:           # keep looping until a real frame arrives
        break                     # (right after connecting, frame can be None briefly)

last_time = time.time()                       # mark start time
while(time.time() - last_time) < 3.0:          # for 3 seconds after takeoff...
    img = robot_frame.frame
    if img is not None: 
        c.imshow('dronecam', img)              # show raw camera feed
        c.waitKey(1)                           # required for imshow to actually render
    robot.send_rc_control(0,0,0,0)              # hold perfectly still (no movement) to stabilize

# ============================================ main loop / superloop

state = 'search'          # top-level state: search / alignment / manual / lost / goThrough
state_search = 'initial'  # sub-state used only within the 'search' state
state_gT = 'through'       # sub-state used only within the 'goThrough' state
state_manual = 'height'    # sub-state used only within the 'manual' state (currently unused/dead code)

aPtr = 0        # index into ANG list -> which rotation step of the search sweep we're on
orgYaw = 0      # yaw recorded at the start of a search sweep (reference point)
dAng = 0        # destination/target yaw for the current rotation step
rotTime = 0     # timestamp used for the settle-pause after finishing a rotation
hDest = 0       # target height chosen during search (either HMIN or HMAX)
tagPtr = 0      # index into TAGS -> which gate/tag we're currently hunting for
lostX = 0       # last known X error before losing sight of the tag
lostY = 0       # last known Y error before losing sight of the tag
lostZ = 0       # last known Z (distance) error before losing sight of the tag
last_time = 0   # generic reusable timestamp variable (used in multiple states)
lostTime = 0    # timestamp for when we entered the 'lost' state
pauseTime = 0   # timestamp for the pause after flying through a gate
MANU_ROT_TIME = 5.0   # how long (seconds) to rotate for in manual mode

ANG = [60,120,180,240,280,320,360]   # sequence of relative yaw offsets (deg) for the search sweep
TAGS = [[9], [4], [0], [5]]      # sequence of target tag IDs, one per gate, in order

def angleCorr(angle):
    # normalizes an angle into the range (-180, 180]
    # needed because yaw wraps around at +/-180 degrees
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle

while True: 
    battery = robot.get_battery()             # current battery %
    temperature = robot.get_temperature()      # current drone temperature
    yaw = robot.get_yaw()                       # current yaw / compass heading
    height = robot.get_distance_tof()            # current height from downward time-of-flight sensor

    errX, errY, errZ, errR = 0,0,0,0    # reset positional/rotational errors each frame
    outX, outY, outZ, outR = 0,0,0,0    # reset RC output commands each frame
    seen = False                          # whether the target tag is visible this frame

    img = robot_frame.frame               # grab the latest camera frame
    if img is not None:
        img = c.cvtColor(img, c.COLOR_RGB2BGR)   # Tello gives RGB, OpenCV wants BGR
        img = c.resize(img, [960, 720])            # force a fixed resolution
        img_debug = img.copy()                     # copy for drawing debug overlays on later

        tags = at.get_tags(img)     # detect all AprilTags visible in this frame
        ids = None
        for tag in tags:
            try: 
                if tag[0] == TAGS[tagPtr][0]:   # check if this tag matches our CURRENT target ID
                    seen = True                  # yes! mark target as seen this frame
                    ids = tag[0]
                    errX = tag[1]                # horizontal offset from tag
                    errY = tag[2]                # vertical offset from tag
                    errZ = tag[3]                # distance offset from tag
                    errR = tag[4]                # rotational offset from tag
                    c.circle(img_debug, (tag[5], tag[6]), 20, (0,0,255), 3)
                    # draws a red circle at the tag's pixel location on the debug image
            except:
                pass   # silently skip malformed tag entries

    if state == "search":
        # sweeps the drone through a set of angles at two alternating heights, looking for the target tag

        if state_search == 'initial':   # reset variables at the start of a sweep
            aPtr = 0
            orgYaw = yaw                 # remember current yaw as the reference
            state_search = 'pick'

        elif state_search == 'pick':   # pick the next target yaw
            dAng = ANG[aPtr]              # get the next relative angle offset
            dAng = dAng + orgYaw          # convert to an absolute target heading
            dAng = angleCorr(dAng)        # normalize into (-180, 180]
            state_search = 'rotate'

        elif state_search == 'rotate':   # rotate until we reach the destination yaw
            errR = yaw - dAng
            errR = angleCorr(errR)
            outR = errR * -12              # proportional controller (gain = -12)
            outR = np.clip(outR, -99, 99)  # clip to max rotation speed
            if abs(errR) < 5:               # close enough to target angle
                errR, outR = 0,0
                rotTime = time.time()       # record time we finished rotating
                state_search = 'wait'

        elif state_search == 'wait':   # brief pause to let drone/camera settle after rotating
            if(time.time() - rotTime) > 0.2:
                aPtr += 1                     # move to next angle in the sweep
                if aPtr >= len(ANG):           # finished a full sweep of all angles
                    aPtr = 0
                    state_search = 'heightDecision'
                else:    
                    state_search = 'pick'
        
        elif state_search == 'heightDecision':
            # after a full rotation with no luck, switch to the FARTHER height bound
            if abs(height - HMAX) < abs(height - HMIN):
                hDest = HMIN
            else: 
                hDest = HMAX
            state_search = 'heightReach'

        elif state_search == 'heightReach':   # move to the new target height
            errY = hDest - height
            outY = errY * 1.2                  # proportional controller (gain = 1.2)
            outY = np.clip(outY, -35, 35)       # clip to max climb/descend speed
            if abs(errY) < 8:                   # close enough to target height
                errY, outY = 0,0
                aPtr = 0
                state_search = 'pick'            # start another rotation sweep at new height

        if seen:
            # this check runs no matter which search sub-state we're in --
            # if the target tag becomes visible at ANY point, immediately stop and switch states
            aPtr = 0
            state_search = 'initial'    # reset for next time we search
            outX, outR, outY, outZ = 0,0,0,0
            state = 'alignment'

    elif state == 'alignment':
        # fine-tunes position on all 4 axes to center on the gate before flying through
        if seen:
            lostX = errX          # cache last-seen errors in case we lose the tag
            lostY = errY - 10
            lostZ = errZ - 40

            errY = errY - 15                 # target is offset -15 from raw tag Y (camera mount offset?)
            outY = errY * -1.7                # proportional controller (gain = -1.7)
            outY = np.clip(outY, -30, 25) 
            if errY < 5 and errY > -5:         # within deadband -> stop moving on this axis
                errY, outY = 0,0 

            errX = errX - 0                  # no offset applied for horizontal
            outX = errX *  0.8                # proportional controller (gain = 0.8)
            outX = np.clip(outX, -35, 35)
            if errX < 5 and errX > -5:
                errX, outX = 0,0 

            errZ = errZ - 40                 # target distance offset by 40 (hover in front of gate, not at it)
            outZ = errZ * 1.3                 # proportional controller (gain = 1.3)
            outZ = np.clip(outZ, -20, 15)      # asymmetric clip: backs up slower than it advances
            if errZ < 5 and errZ > -5:
                errZ, outZ = 0,0 

            errR = errR - 0                  # no offset applied for rotation
            outR = errR * 1                   # proportional controller (gain = 1)
            outR = np.clip(outR, -35, 35)
            if errR < 5 and errR > -5:
                errR, outR = 0,0 

            if errX<5 and errX>-8 and errR<2 and errR>-2 and errY<5 and errY>-5 and errZ<5 and errZ>-5:
                # ALL four axes are within tolerance at the same time -> we're aligned!
                outX, outR, outY, outZ = 0,0,0,0
                state = 'goThrough'
                last_time = time.time()       # record time so goThrough knows when to start counting

        else:
            # lost sight of the tag while trying to align
            state = 'lost'
            lostTime = time.time()

    elif state == 'manual':
        # NOTE: nothing in this script ever sets state = 'manual', so this code never actually runs.
        # meant as a manual-override mode using 'U'/'D'/'L'/'R' flags stored in the TAGS list.

        if state_manual == 'height':
            last_time = time.time()
            if 'U' in TAGS[tagPtr]:              # climb if 'U' flag present for this gate
                outX, outY, outZ, outR = 0,30,0,0
            elif 'D' in TAGS[tagPtr]:             # descend if 'D' flag present
                outX, outY, outZ, outR = 0,-30,0,0
            else:
                state_manual = 'rotate'            # no height flag -> skip to rotate

            if(outY > 0 and height >= HMAX) or (outY < 0 and height <= HMIN):
                # safety clamp: stop once we hit height bounds
                outX, outY, outZ, outR = 0,0,0,0
                state_manual = 'rotate'
        
        elif state_manual == 'rotate':
            if 'L' in TAGS[tagPtr]:               # rotate left if 'L' flag present
                outX, outY, outZ, outR = 0,0,0,-25
            elif 'R' in TAGS[tagPtr]:
                # BUG: should be TAGS[tagPtr], not TAGS(tagPtr) -- this would crash if reached
                outX, outY, outZ, outR = 0,0,0,25
            else:
                last_time += MANU_ROT_TIME         # no flags -> fast-forward the timer so it exits immediately

            if(time.time() - last_time) > MANU_ROT_TIME:
                outX, outY, outZ, outR = 0,0,0,0
                state = 'search'                    # done -> go back to searching
                state_manual = 'height'              # reset for next time

        if seen:
            # if the tag appears mid-maneuver, abandon manual mode and go straight to alignment
            state = 'alignment'
            outX, outY, outZ, outR = 0,0,0,0

    elif state == 'lost':
        # handles briefly losing the tag -- tries to keep drifting toward its last known position
        # instead of immediately restarting a full search
        if seen == True:
            outX, outR, outY, outZ = 0,0,0,0
            state = 'alignment'      # tag reappeared -> go back to fine alignment
                                                                                                                
        elif (time.time() - lostTime) > 10.0:
            # been lost too long (>10s) -> give up and do a full search instead
            outY, outX, outR, outZ = 0,0,0,0
            state = 'search'
        else:
            # still within the grace period -> nudge based on LAST KNOWN offsets

            if lostX < -7:
                outR = -20                # rotate one way
            elif lostX > 7: 
                outR = 20                 # rotate the other way
            else: 
                outR = 0

            if lostY > 0:
                outY = -20                # descend if it was last seen low
            elif lostY < -10: 
                outY = 0                  # stop descending if it was last seen high enough
                # NOTE: no branch here to actively climb -- possible gap in logic

            if lostZ < 0:
                outZ = -15                # back up if it was too close
            elif lostZ > 10:
                outZ = 0                  # stop moving forward once far enough
                # NOTE: no branch here to actively move forward -- possible gap in logic

    elif state == "goThrough": 
        # flies straight through the gate, then pauses before the next search

        if state_gT == 'through':
            if (time.time() - last_time) < 3.0: 
                outX, outR, outY, outZ = 0,0,0,30   # fly straight forward for 3 seconds
            else:
                outX, outR, outY, outZ = 0,0,0,0
                tagPtr += 1                           # move on to the next target tag
                if tagPtr >= len(TAGS):
                    break                              # that was the last gate -> end the mission
                else:
                    pauseTime = time.time()
                    state_gT = 'pause'

        elif state_gT == 'pause':
            if (time.time() - pauseTime) < 1.0: 
                outX, outR, outY, outZ = 0,0,0,0        # hover still for 1 second to let things settle
            else:
                state = 'search'                          # go hunt for the next gate's tag
                state_gT = 'through'                        # reset for next time

    robot.send_rc_control(int(outX),int(outZ),int(outY),int(outR))
    # sends the actual RC command -- Tello's signature is (left_right, forward_back, up_down, yaw)
    # so: outX = left/right, outZ = forward/back, outY = up/down, outR = yaw

    # ---- draw live telemetry text onto the debug frame ----
    c.putText(img_debug, f'state: {state}', (20,30), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'seen: {seen}', (20,70), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'tagPtr: {tagPtr}', (20,110), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errX: {errX}', (20,150), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errY: {errY}', (20,190), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errZ: {errZ}', (20,230), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'errR: {errR}', (20,270), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)
    c.putText(img_debug, f'battery: {battery}', (20,310), c.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 2)

    c.imshow('dronecam', img_debug)   # show the debug view
    key = c.waitKey(1)
    if key == 27:                      # Esc key -> manually abort
        break

robot.land()   # land the drone once the loop ends (all gates done, or Esc pressed)