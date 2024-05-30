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
saber.sendCommand(16, 15) # 


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


####
####    CONSTANTS
####

### DISTANCE

TRACK_DISTANCE      = 1.3
DISTANCE_THRESHOLD  = 0.1
ANGLE_THRESHOLD     = 10


### ACCELERATION

ACCEL_LINEAR        = 1.2
ACCEL_TURN          = 1.5


### MAX SPEED & MAX SPEED DIFFERENCE

MAX_SPEED           = 40
MAX_REVERSE         = -30
MAX_SPEED_DIFF      = 100


### TUNING DIFFERENCE (what speed difference is considererd turning?)

TURN_DIFF           = 20


### SAFETY MARGINS

MARGIN_FRONT        = 1.25
MARGIN_REAR         = 0.6
MARGIN_SIDES        = 0.6


####
####    MAIN
####

### VARIABLES

sspeed = 35
tspeed = 25

# Set speed of each motor
speed_left, speed_right = 0, 0

# Precision mode with lower margins
precision_mode = False


### OBSTACLE DETECTION

def detect_obstacles(speed_left, speed_right, heading, fac=1.0):
    print(f"2 {heading}")

    # Check sides if turning
    if np.abs(speed_right - speed_left) > TURN_DIFF:
        return utils.obstacle_detection(tracking.clusters, 30, 150, MARGIN_SIDES*fac, heading) or\
            utils.obstacle_detection(tracking.clusters, 210, 330, MARGIN_SIDES*fac, heading)

    # Check front if going forwards
    if 0 < speed_left and 0 < speed_right:
        return utils.obstacle_detection(tracking.clusters, 330, 30, MARGIN_FRONT*fac, heading)

    # Check rear if going in reverse
    if 0 > speed_left and 0 > speed_right:
        return utils.obstacle_detection(tracking.clusters, 150, 210, MARGIN_REAR*fac, heading)
    
    return False


### MAIN LOOP

def speech_client():
    # Set override
    if engage.is_set()      : tracking.override = False
    elif disengage.is_set() : tracking.override = True

    # Clear both
    engage.clear(); disengage.clear()


def main_loop():
    global speed_left, speed_right

    ### DELTA TIME 1

    t1 = time.time()


    ### FETCHING SENSOR HEADING

    heading = utils.get_heading(bno)
    print(f"1 {heading}")
    
    ### TRACKING USER

    tracking.track_cycle(heading=heading)


    ### MOTOR CONTROL

    if tracking.tracked_point:

        # Get relative distance and angle to user
        distance, angle = tracking.tracked_point[0], utils.correct_angle(np.rad2deg(tracking.tracked_point[1]), heading)

        # Turning control
        if angle < -ANGLE_THRESHOLD:
            speed_right += ACCEL_TURN
            speed_left  -= ACCEL_TURN
        
        elif angle > ANGLE_THRESHOLD:
            speed_right -= ACCEL_TURN
            speed_left  += ACCEL_TURN

        # Distance control
        if distance - TRACK_DISTANCE > -DISTANCE_THRESHOLD:
            speed_right += ACCEL_LINEAR
            speed_left  += ACCEL_LINEAR

        elif distance - TRACK_DISTANCE < DISTANCE_THRESHOLD:
            speed_right -= ACCEL_LINEAR
            speed_left  -= ACCEL_LINEAR

        # Enforce speed caps
        if speed_right > MAX_SPEED  : speed_right = MAX_SPEED
        if speed_left > MAX_SPEED   : speed_left = MAX_SPEED

        if speed_right < MAX_REVERSE    : speed_right = MAX_REVERSE
        if speed_left < MAX_REVERSE     : speed_left = MAX_REVERSE

        # Enforce max. difference
        if np.abs(speed_right - speed_left) > MAX_SPEED_DIFF:
            if speed_left < speed_right     : speed_left = speed_right - MAX_SPEED_DIFF
            elif speed_right < speed_left   : speed_right = speed_left - MAX_SPEED_DIFF
    
    else:
        speed_left, speed_right = 0, 0


    ### VOICE CONTROL 

    speech_client()

    # Voice command directions state machine
    if forward.is_set()     : speed_left, speed_right = sspeed  , sspeed    #; print('FWD')
    elif reverse.is_set()   : speed_left, speed_right = -sspeed , -sspeed   #; print('REV')
    elif left.is_set()      : speed_left, speed_right = -tspeed , tspeed    #; print('LEF')
    elif right.is_set()     : speed_left, speed_right = tspeed  , -tspeed   #; print('RGT')
    elif stop.is_set()      : speed_left, speed_right = 0       , 0         #; print('STP')


    ### OBSTACLE DETECTION

    if detect_obstacles(speed_left, speed_right, heading, fac=1.0):
        print("JASPER: Obstacle detected!")
        speed_left, speed_right = 0, 0


    ### DRIVE MOTORS

    saber.driveBoth(round(speed_left), -round(speed_right))


    ### PRINT DELTA TIME

    # print(time.time() - t1)

if __name__ == "__main__":
    while True:
        try:
            main_loop()
            
        except KeyboardInterrupt:
            exit_handler()