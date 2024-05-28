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

voice_control = subprocess.Popen(["../speech.sh"], stdout = subprocess.PIPE)

# Print output

def print_voice_control(voice_control):
    for l in iter(voice_control.stdout.readline, b""):
        sys.stdout.write(f'VOICE: {l.decode("utf-8", "backslashreplace")}\n')

# Voice control printing
voice_control_thread = threading.Thread(target=print_voice_control, daemon=True, args=(voice_control,))
voice_control_thread.start()

# Wait for speech server to start
time.sleep(3)
print('Initiated speech server...')

### GYROSCOPE NOTE

# # Generating new BNO 
# bno = BNO055()

# if not bno.begin(mode=BNO055.OPERATION_MODE_NDOF):
#     print("Error initializing BNO055")
#     exit()

# time.sleep(1)
# bno.setExternalCrystalUse(True)
# print('Initiated BNO055 sensor...')


### RECEIVE SPEECH

# Speech events & thread
terminate_speech, engage, disengage, forward, reverse, left, right, stop = [threading.Event() for _ in range(8)]
speech = threading.Thread(target=stream.receive_speech, daemon=True, args=(terminate_speech, engage, disengage, forward, reverse, left, right, stop,))
speech.start()

### MOTOR DRIVER

saber = Sabertooth("/dev/ttyTHS1", baudrate=9600, address=128, timeout=0.4)
# saber.sendCommand(16, 18)


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

    # Wait for speech server to end
    # voice_control_thread.join()

    # Exit program
    os._exit(0)


####
####    CONSTANTS
####

### DISTANCE

TRACK_DISTANCE      = 0.6
DISTANCE_THRESHOLD  = 0.1
ANGLE_THRESHOLD     = 10


### ACCELERATION

ACCEL_LINEAR        = 2.5
ACCEL_TURN          = 0.5


### MAX SPEED & MAX SPEED DIFFERENCE

MAX_SPEED           = 60
MAX_REVERSE         = -30
MAX_SPEED_DIFF      = 30


### TUNING DIFFERENCE (what speed difference is considererd turning?)

TURN_DIFF           = 5


### SAFETY MARGINS

MARGIN_FRONT        = 0.4
MARGIN_REAR         = 0.6
MARGIN_SIDES        = 0.5


####
####    MAIN
####

### VARIABLES

# Set speed of each motor
speed_left, speed_right = 0, 0

# Precision mode with lower margins
precision_mode = False


### OBSTACLE DETECTION

def detect_obstacles(speed_left, speed_right, heading, fac=1.0):
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

    heading = 0 # utils.get_heading()

    
    ### TRACKING USER

    tracking.track_cycle() # heading=heading


    ### MOTOR CONTROL

    if tracking.tracked_point:

        # Get relative distance and angle to user
        distance, angle = tracking.tracked_point[0], utils.correct_angle(np.rad2deg(tracking.tracked_point[1]), heading)

        # Distance control

        if distance - TRACK_DISTANCE > -DISTANCE_THRESHOLD:
            speed_right += ACCEL_LINEAR
            speed_left  += ACCEL_LINEAR

        elif distance - TRACK_DISTANCE > -DISTANCE_THRESHOLD:
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


        # Turning control

        if angle < -ANGLE_THRESHOLD:
            speed_right += ACCEL_TURN
            speed_left  -= ACCEL_TURN
        
        elif angle > ANGLE_THRESHOLD:
            speed_right -= ACCEL_TURN
            speed_left  += ACCEL_TURN
    
    else:
        speed_left, speed_right = 0, 0


    ### VOICE CONTROL 

    speech_client()

    # Voice command directions state machine
    if forward.is_set()     : speed_left, speed_right = 20  , 20    #; print('FWD')
    elif reverse.is_set()   : speed_left, speed_right = -20 , -20   #; print('REV')
    elif left.is_set()      : speed_left, speed_right = -20 , 20    #; print('LEF')
    elif right.is_set()     : speed_left, speed_right = 20  , -20   #; print('RGT')
    elif stop.is_set()      : speed_left, speed_right = 0   , 0     #; print('STP')


    ### OBSTACLE DETECTION

    if detect_obstacles(speed_left, speed_right, heading, fac=1.0):
        print("JASPER: Obstacle detected!")
        speed_left, speed_right = 0, 0


    ### DRIVE MOTORS

    saber.driveBoth(speed_left, speed_right)


    ### PRINT DELTA TIME

    # print(time.time() - t1)

if __name__ == "__main__":
    while True:
        try:
            main_loop()
            
        except KeyboardInterrupt:
            exit_handler()