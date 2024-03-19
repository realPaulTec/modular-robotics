import math
from drivers import MotorDriver, MotorInterface
import threading
from tracking import Tracking
from queue import Queue
from threading import Thread
import time
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
    print('\nTerminating ...')
    
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

while True:
    try: 
        # if stop_control.is_set():   terminate()

        # Get initial time
        time_1 = time.time()

        # Run speech client
        speech_client()

        # Tracking system cycle
        tracking.track_cycle()

        # Getting the direction in degrees and distance to person
        if tracking.tracked_point:
            distance, direction = tracking.tracked_point[0], math.degrees(tracking.tracked_point[1])
        else:
            # Set PWM for both motors to Zero if not tracking
            pwm_A = 0
            pwm_B = 0

            # Interface with the motors
            interface.control(pwm_A, pwm_B)

            continue

        # Left - Right PWM adjustments TODO: FIX PWM!!!
        if direction > thresh_degrees:
            pwm_A = 0
            pwm_B = 100
        elif direction < -thresh_degrees:
            pwm_A = 100
            pwm_B = 0
        else:
            pwm_A = 100
            pwm_B = 100
        
        # Distance adjustments
        if pwm_A == 100 and pwm_B == 100 and tracking_distance - thresh_meters < distance < tracking_distance + thresh_meters:
            pwm_A = 0
            pwm_B = 0
        elif pwm_A == 100 and pwm_B == 100 and distance < tracking_distance - thresh_meters:
            pwm_A = -100
            pwm_B = -100

        # Voice command directions state machine
        if forward.is_set()     : pwm_A, pwm_B = 100, 100       ; print('FWD')
        elif reverse.is_set()   : pwm_A, pwm_B = -100, -100     ; print('REV')
        elif left.is_set()      : pwm_A, pwm_B = -100, 100      ; print('LEF')
        elif right.is_set()     : pwm_A, pwm_B = 100, -100      ; print('RGT')
        elif stop.is_set()      : pwm_A, pwm_B = 0, 0           ; print('STP')

        # Object avoidance
        # if tracking.tracking_failiure and check_for_obstacles(tracking.clusters, -thresh_degrees, thresh_degrees, 0, tracking_distance + thresh_meters):
        #     pwm_A = -100
        #     pwm_B = -100
        # elif tracking.tracking_failiure:
        #     tracking.tracking_failiure = False

        # print(
        #     check_for_obstacles(tracking.clusters, 0, thresh_degrees, 0, tracking_distance + thresh_meters)
        # )

        # Control the motors with set PWM values
        interface.control(-pwm_A, -pwm_B)
   
    # Exiting program after keyboardinterrupt
    except KeyboardInterrupt:
        terminate()
