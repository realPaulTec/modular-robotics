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

def terminate():
    print('\nTerminating ...')
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

while True:
    try: 
        if stop_control.is_set():   terminate()

        # Get initial time
        time_1 = time.time()

        # Tracking system cycle
        tracking.track_cycle() #hall_thread=hall_thread, hall_queue=results_queue, stop_feedback=stop_feedback)

        # # Clear stop feedback signal
        # stop_feedback.clear()

        # Getting the direction in degrees and distance to person
        if tracking.tracked_point:
            distance, direction = tracking.tracked_point[0], math.degrees(tracking.tracked_point[1])
        else:
            # Set PWM for both motors to Zero if not tracking
            pwm_A = 0
            pwm_B = 0

            # print(f"\r\033[K {pwm_A} | {pwm_B} | NONE | NONE  ", end="")

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
        if pwm_A == 100 and pwm_B == 100 and tracking_distance - thresh_meters < distance < tracking_distance +  thresh_meters:
            pwm_A = 0
            pwm_B = 0
        elif pwm_A == 100 and pwm_B == 100 and distance < tracking_distance - thresh_meters:
            pwm_A = -100
            pwm_B = -100

        interface.control(-pwm_A, -pwm_B)
   
    # Exiting program after keyboardinterrupt
    except KeyboardInterrupt:
        terminate()
