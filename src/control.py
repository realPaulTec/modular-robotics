import math
from drivers import MotorDriver, MotorInterface
import threading
from tracking import Tracking
from queue import Queue
from threading import Thread
import os
import sys
import stream

# sudo /home/paultec/archiconda3/bin/python3 tracking_interface.py

# Speech events & thread
terminate_speech, engage, disengage, forward, reverse, left, right, stop = [threading.Event() for _ in range(8)]
speech = threading.Thread(target=stream.receive_speech, args=(terminate_speech, engage, disengage, forward, reverse, left, right, stop,))
speech.daemon = True
speech.start()

# generating new MotorDriver class driver with motor pins
driver = MotorDriver(33, 36, 35, 32, 38, 40, 12, 16, 18, 22)

# generating new lidar class "scanner"
tracking = Tracking()
tracking.override = True

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

# Generating new motor interface class based on motor driver
interface = MotorInterface(driver, wheel_radius, wheelbase)

# Stop tracking event
stop_control = threading.Event()

# Streamer thread
streamer = threading.Thread(target=stream.streamer, args=(tracking, stop_control,))
streamer.daemon = True
streamer.start()

def speech_client():
    if engage.is_set()      : tracking.override = False
    elif disengage.is_set() : tracking.override = True

def terminate():
    print('\nTerminating...')
    
    # Terminate speech
    terminate_speech.set()
    speech.join()
    
    # Turn off printing errors
    sys.stderr = open(os.devnull, 'w')
    
    # Trigger exit handlers for GPIO and LiDAR
    tracking.lidar.exit_handler()
    driver.exit_handler()
    
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

# Checking for obscales in specified ranges.
def check_for_obstacles(clusters, min_angle, max_angle, min_distance, max_distance):
    for label, cluster_data in clusters.items():
        # Extract the central position in polar coordinates (r, theta)
        r, theta = cluster_data['central_position']

        # Return true if cluster is in the way
        if min_angle <= theta <= max_angle and min_distance <= r <= max_distance:
            return True

    # Return false if no cluster is in the way
    return False

# Get the PWM for the motors
def get_control(distance, direction):
    # Direction adjustments
    if direction > thresh_degrees       : return 0, 100
    elif direction < -thresh_degrees    : return 100, 0

    # Distance adjustments
    if tracking_distance - thresh_meters < distance < tracking_distance + thresh_meters : return 0, 0
    elif distance < tracking_distance - thresh_meters                                   : return -100, -100

    return 100, 100

while True:
    try: 
        # Run speech client
        speech_client()

        # Tracking system cycle
        tracking.track_cycle()

        # Get distance and direction to user if currently tracking & Getting PWM for motor control
        if tracking.tracked_point   : pwm_A, pwm_B = get_control(tracking.tracked_point[0], math.degrees(tracking.tracked_point[1]))
        else                        : pwm_A, pwm_B = 0, 0 

        # Voice command directions state machine
        if forward.is_set()     : pwm_A, pwm_B = 100, 100       #; print('FWD')
        elif reverse.is_set()   : pwm_A, pwm_B = -100, -100     #; print('REV')
        elif left.is_set()      : pwm_A, pwm_B = -100, 100      #; print('LEF')
        elif right.is_set()     : pwm_A, pwm_B = 100, -100      #; print('RGT')
        elif stop.is_set()      : pwm_A, pwm_B = 0, 0           #; print('STP')

        # Control the motors with set PWM values
        interface.control(-pwm_A, -pwm_B)
   
    # Exiting program after keyboardinterrupt
    except KeyboardInterrupt:
        terminate()
