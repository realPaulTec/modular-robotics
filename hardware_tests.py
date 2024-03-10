
import os
from queue import Queue
import sys
import time
from drivers import MotorDriver, MotorInterface
from tracking import Tracking
import threading

# generating new MotorDriver class driver with motor pins
driver = MotorDriver(33, 36, 35, 32, 38, 40, 12, 16, 18, 22)

# Stop event for hall feedback
stop_feedback = threading.Event()

# Setting wheel radius and wheelbase in meters
wheel_radius = 0.025
wheelbase = 0.45 # 17

# Generating new motor interface class based on motor driver
interface = MotorInterface(driver, wheel_radius, wheelbase)

while True:
    try:    
        pwm_1, pwm_2, stime = input("PWM1, PWM2, TIME: ").replace(" ", "").split(",")

        # Create queue for results
        results_queue = Queue()

        # Get offset for tracking 
        hall_thread = threading.Thread(target=interface.get_tracking_offset, args=(results_queue, stop_feedback,))
        hall_thread.deamon = True
        hall_thread.start()

        # Interface with the motors
        interface.control(-int(pwm_1), -int(pwm_2))

        # Sleep for stime seconds
        time.sleep(float(stime))

        # Stop the motors
        interface.control(0, 0)

        # Stop the thread
        stop_feedback.set()
        
        # Waiting for thread to finish and getting information
        hall_thread.join()
        linear_displacement, angular_displacement = results_queue.get()

        print(f"{round(linear_displacement, 4)} | {round(angular_displacement, 4)}")

        # Clear the stopping signal
        stop_feedback.clear()
    
    # Exiting program after keyboardinterrupt
    except KeyboardInterrupt:
        print('\nTerminating ...')
        sys.stderr = open(os.devnull, 'w')
                
        # Trigger exit handlers for GPIO and LiDAR
        driver.exit_handler()
        
        # Exit program
        os._exit(0)