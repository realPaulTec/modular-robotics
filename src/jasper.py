import atexit
import numpy as np
from drivers.BNO055 import BNO055
from drivers.PySabertooth import Sabertooth
from tracking import Tracking
import control_utils as utils

import subprocess
import threading
import time
import stream
import os
import sys


####
####    SETUP
####

### SPEECH

voice_control = subprocess.Popen(["../speech.sh"])


# Wait for speech server to start
time.sleep(3)
print('JASPER: initiated speech server...')

### GYROSCOPE NOTE

# Generating new BNO 
bno = BNO055()

if not bno.begin(mode=BNO055.OPERATION_MODE_NDOF):
    print("Error initializing BNO055")
    exit()

time.sleep(1)
bno.setExternalCrystalUse(True)
print('Initiated BNO055 sensor...')


### RECEIVE SPEECH

# Speech events & thread
terminate_speech, engage, disengage, forward, reverse, left, right, stop = [threading.Event() for _ in range(8)]
speech = threading.Thread(target=stream.receive_speech, daemon=True, args=(terminate_speech, engage, disengage, forward, reverse, left, right, stop,))
speech.start()

### MOTOR DRIVER

saber = Sabertooth("/dev/ttyTHS1", baudrate=9600, address=128, timeout=0.4)
saber.sendCommand(16, 14)

### TRACKING

tracking = Tracking()
tracking.override = True
print('JASPER: Initiated tracking...')

### STREAMING

stop_control = threading.Event()
streamer = threading.Thread(target=stream.streamer, daemon=True, args=(tracking, stop_control,))
streamer.start()


### EXITHANDLER

def exit_handler():
    print('\nJASPER: Terminating...')

    # Stop motors
    saber.driveBoth(0, 0)

    # Terminate speech client
    terminate_speech.set()
    speech.join()

    # Terminate speech server
    try     : voice_control.terminate()
    except  : pass

    # Turn off printing errors
    sys.stderr = open(os.devnull, 'w')

    # Trigger exit handlers for GPIO and LiDAR
    tracking.lidar.exit_handler()

    # Exit program
    os._exit(0)

# Register exit handler
atexit.register(exit_handler)

####
####    CONSTANTS
####

### DISTANCE

B_OFFSET            = 0.0 #.3 

FORWARD_THRESHOLD   = 1.50 + B_OFFSET
REVERSE_THRESHOLD   = 1.12 + B_OFFSET

FWD_STOP            = 1.40 + B_OFFSET
REV_STOP            = 1.15 + B_OFFSET

ANGLE_THRESHOLD     = 15 
ANGLE_STOP          = 12

ANGLE_HARD          = 30

### ACCELERATION

ACCEL_FRONT         = 1.5
ACCEL_TURN          = 2.0


### MAX SPEED & MAX SPEED DIFFERENCE

MAX_SPEED           =  80
MIN_SPEED           =  40
MAX_REVERSE         = -65
TURN_SPEED          = 55
MIN_TURN_SPEED      = 32

### TUNING DIFFERENCE (what speed difference is considererd turning?)

TURN_DIFF           = 20


### SAFETY MARGINS

MARGIN_FRONT        = 1.40 + B_OFFSET
MARGIN_REAR         = 0.65
MARGIN_SIDES        = 0.6


####
####    MAIN
####

### VARIABLES

sspeed = 50
rspeed = 20
tspeed = 35
hard_turn = False

# Set speed of each motor
speed_left, speed_right = 0, 0

# Precision mode with lower margins
precision_mode = False
hard_turn = False


### OBSTACLE DETECTION

def detect_obstacles(speed_left, speed_right, fac=1.0):
    # Check sides if turning
    if np.abs(speed_right - speed_left) > TURN_DIFF:
        return utils.obstacle_detection(tracking, 30, 150, MARGIN_SIDES*fac, label="LEFT") or\
            utils.obstacle_detection(tracking, 210, 350, MARGIN_SIDES*fac, label="RIGHT")

    # Check front if going forwards
    if 0 < speed_left and 0 < speed_right:
        return utils.obstacle_detection(tracking, 150, 210, MARGIN_FRONT*fac, label="FRONT")

    # Check rear if going in reverse
    if 0 > speed_left and 0 > speed_right:
        return utils.obstacle_detection(tracking, 250, 130, MARGIN_REAR*fac, label="REAR")
    
    return False


### MAIN LOOP

def speech_client():
    # Set override
    if engage.is_set()      : tracking.override = False
    elif disengage.is_set() : tracking.override = True

    # Clear both
    engage.clear(); disengage.clear()


def main_loop():
    global speed_left, speed_right, hard_turn

    ### DELTA TIME 1

    t1 = time.time()


    ### FETCHING SENSOR HEADING

    heading = utils.get_heading(bno)


    ### TRACKING USER

    tracking.track_cycle(heading=heading)

    # print(f"DT {time.time() - t1}")


    ### MOTOR CONTROL

    if tracking.tracked_point:

        # Get relative distance and angle to user
        distance, angle = tracking.tracked_point[0], utils.correct_angle(np.rad2deg(tracking.tracked_point[1]), heading)

        # Turning control
        if np.abs(angle) > ANGLE_HARD:
            hard_turn = True

            if angle < 0:
                speed_left  = -TURN_SPEED
                speed_right = TURN_SPEED
            
            elif angle > 0:
                speed_left  = TURN_SPEED
                speed_right = -TURN_SPEED

        else:
            if hard_turn == True:
                speed_left, speed_right = 0, 0
                hard_turn = False
            
            if angle < -ANGLE_THRESHOLD:
                if speed_right < MAX_SPEED  : speed_right += ACCEL_TURN
                if speed_left > MAX_REVERSE : speed_left  -= ACCEL_TURN

                speed_right = utils.get_speed(speed_right, thresh = MIN_TURN_SPEED)
            
            elif angle > ANGLE_THRESHOLD:
                if speed_right > MAX_REVERSE    : speed_right  -= ACCEL_TURN
                if speed_left < MAX_SPEED       : speed_left += ACCEL_TURN

                speed_left = utils.get_speed(speed_left, thresh = MIN_TURN_SPEED)

        # Stop at specified angle
        if np.abs(angle) < ANGLE_STOP:
            if speed_right > speed_left : speed_left = speed_right
            if speed_left > speed_right : speed_right = speed_left   

        # Distance control
        if not hard_turn:
            if distance > FORWARD_THRESHOLD\
                and speed_right < MAX_SPEED and speed_left < MAX_SPEED:
                speed_right += ACCEL_FRONT
                speed_left  += ACCEL_FRONT

            elif distance < REVERSE_THRESHOLD:
                speed_right = MAX_REVERSE
                speed_left  = MAX_REVERSE

            # Enforce reverse and forward stops
            if distance > REV_STOP and\
                speed_left < 0 and speed_right < 0:
                
                speed_right = 0
                speed_left = 0

            if distance < FWD_STOP and\
                speed_left > 0 and speed_right > 0:
                
                speed_right = 0
                speed_left = 0
 
    else:
        speed_left, speed_right = 0, 0


    ### VOICE CONTROL 

    speech_client()

    # Voice command directions state machine
    if forward.is_set()     : speed_left, speed_right = sspeed  , sspeed    #; print('FWD')
    elif reverse.is_set()   : speed_left, speed_right = -rspeed , -rspeed   #; print('REV')
    elif left.is_set()      : speed_left, speed_right = tspeed , -tspeed    #; print('LEF')
    elif right.is_set()     : speed_left, speed_right = -tspeed  , tspeed   #; print('RGT')
    elif stop.is_set()      : speed_left, speed_right = 0       , 0         #; print('STP')


    ### OBSTACLE DETECTION

    if detect_obstacles(speed_left, speed_right, fac=1.0):
        # print("JASPER: Obstacle detected!")
        speed_left, speed_right = 0, 0


    ### DRIVE MOTORS
    
    # print(speed_left, speed_right)
    saber.driveBoth(round(utils.get_speed(speed_left)), -round(utils.get_speed(speed_right)))


    ### PRINT DELTA TIME

    # print(f"DTF {time.time() - t1}")

if __name__ == "__main__":
    while True:
        try:
            main_loop()
            
        except KeyboardInterrupt:
            exit_handler()