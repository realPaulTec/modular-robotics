import math
import time
from drivers import MotorDriver, MotorInterface
import threading
from tracking import Tracking
from queue import Queue
from threading import Thread
import os
import sys
import numpy as np
import stream
from BNO055 import BNO055
import subprocess
import signal

# Stop event for hall feedback
stop_feedback = threading.Event()

# Trackng distance in meters
tracking_distance = 0.55

# Setting wheel radius and wheelbase in meters
wheel_radius = 0.025
wheelbase = 0.17

# Manuvering treshold
thresh_degrees = 20
thresh_meters = 0.05

# Startign speech process
def run_speech_server():
    # Start speech process
    global proc; proc = subprocess.Popen(["../speech.sh"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

# Run the speech server thread
speech_server = threading.Thread(target=run_speech_server())
speech_server.start()

# Wait for speech server to start
time.sleep(2)
print('Initiated speech server...')

# Generating new BNO
bno = BNO055()

if not bno.begin(mode=BNO055.OPERATION_MODE_NDOF):
    print("Error initializing BNO055")
    exit()

time.sleep(1)
bno.setExternalCrystalUse(True)
print('Initiated BNO055 sensor...')

# Speech events & thread
terminate_speech, engage, disengage, forward, reverse, left, right, stop = [threading.Event() for _ in range(8)]
speech = threading.Thread(target=stream.receive_speech, args=(terminate_speech, engage, disengage, forward, reverse, left, right, stop,))
speech.daemon = True
speech.start()

# generating new MotorDriver class driver with motor pins
driver = MotorDriver(33, 36, 35, 32, 38, 40, 12, 16, 18, 22)
print('Initiated motor driver...')

# generating new lidar class "scanner"
tracking = Tracking()
tracking.override = True
print('Initiated tracking...')

# Generating new motor interface class based on motor driver
interface = MotorInterface(driver, wheel_radius, wheelbase)

# Stop tracking event
stop_control = threading.Event()

# Streamer thread
streamer = threading.Thread(target=stream.streamer, args=(tracking, stop_control,))
streamer.daemon = True
streamer.start()

def speech_client():
    # Set override
    if engage.is_set()      : tracking.override = False
    elif disengage.is_set() : tracking.override = True

    # Clear both
    engage.clear(); disengage.clear()

def terminate():
    print('\nTerminating...')

    # Terminate speech client
    terminate_speech.set()
    speech.join()

    # Terminating speech server
    try:
        # Send SIGTERM & timeout after 10 seconds
        proc.terminate()
        proc.wait(timeout=10)
    except ProcessLookupError:
        print(f'Speech server process-{proc.pid} not found...')
    except subprocess.TimeoutExpired:
        # Send SIGKILL if termination fails
        proc.kill()

    # Turn off printing errors
    sys.stderr = open(os.devnull, 'w')
      
    # Trigger exit handlers for GPIO and LiDAR
    tracking.lidar.exit_handler()
    driver.exit_handler()
    
    # Wait for speech server to end
    speech_server.join()
    
    # Exit program
    os._exit(0)

def start_hall():
    # Create queue for results
    results_queue = Queue()

    # Get offset for tracking 
    hall_thread = Thread(target=interface.get_tracking_offset, args=(results_queue, stop_feedback,))
    hall_thread.deamon = True
    hall_thread.start()

    return hall_thread, results_queue

# TODO: don't just look at the center
def obstacle_detection(clusters, min_angle, max_angle, max_distance, heading):
    # Return false if there are no clusters
    if len(clusters) == 0: return False
    
    for label, cluster_data in clusters.items():
        # Get angle and radius from cluster center
        radius, angle = cluster_data['central_position']
        
        # Skip if radius not in range
        if radius > max_distance: continue

        # Normalize angle
        angle = correct_angle(np.rad2deg(-angle), heading)
        angle += 360 if angle < 0 else 0

        if min_angle > max_angle and\
            (angle > min_angle or angle < max_angle)    : return True
        elif min_angle < max_angle and\
            angle > min_angle and angle < max_angle     : return True

    # Return false is there is no obstacle in the area
    return False

# Get the PWM for the motors
def get_control(distance, direction):
    # print(direction, distance, tracking.heading)

    # Direction adjustments
    if direction > thresh_degrees       : return -100, 100
    elif direction < -thresh_degrees    : return 100, -100

    # Distance adjustments
    if tracking_distance - thresh_meters < distance < tracking_distance + thresh_meters : return 0, 0
    elif distance < tracking_distance - thresh_meters                                   : return -100, -100

    return 100, 100

def get_heading():
    # Read compass data (Heading, Roll, Pitch)
    heading, roll, pitch = bno.getVector(BNO055.VECTOR_EULER)

    return heading

def correct_angle(angle, heading):
    # Adjust object_degree by magnetic_heading
    adjusted_angle = angle + heading - 360
    
    # Normalize the result to be within -180 to 180 degrees
    adjusted_angle = (adjusted_angle + 180) % 360 - 180
    
    return adjusted_angle

while True:
    try:
        t1 = time.time()
        # Run speech client
        speech_client()

        # Get sensor heading
        heading = get_heading()

        # Tracking system cycle
        tracking.track_cycle(heading=heading)

        # Get distance and direction to user if currently tracking & Getting PWM for motor control
        if tracking.tracked_point   : pwm_A, pwm_B = get_control(tracking.tracked_point[0], correct_angle(np.rad2deg(tracking.tracked_point[1]), heading))
        else                        : pwm_A, pwm_B = 0, 0 

        # Voice command directions state machine
        if forward.is_set()     : pwm_A, pwm_B = 100, 100       #; print('FWD')
        elif reverse.is_set()   : pwm_A, pwm_B = -100, -100     #; print('REV')
        elif left.is_set()      : pwm_A, pwm_B = -100, 100      #; print('LEF')
        elif right.is_set()     : pwm_A, pwm_B = 100, -100      #; print('RGT')
        elif stop.is_set()      : pwm_A, pwm_B = 0, 0           #; print('STP')

        # Obstacle detection
        # if      obstacle_detection(tracking.clusters, 315, 45, 0.2, heading)    : print('FWRD')
        # if      obstacle_detection(tracking.clusters, 45, 135, 0.2, heading)    : print('RGHT')
        # if      obstacle_detection(tracking.clusters, 135, 225, 0.2, heading)   : print('REAR')
        # if      obstacle_detection(tracking.clusters, 225, 315, 0.2, heading)   : print('LEFT')

        # Control the motors with set PWM values
        interface.control(-pwm_A, -pwm_B)

        print(time.time() - t1)
   
    # Exiting program after keyboardinterrupt
    except KeyboardInterrupt:
        terminate()
