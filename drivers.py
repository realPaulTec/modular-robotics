import math
import Jetson.GPIO as GPIO
import atexit
import time

class MotorDriver:
    def __init__(self, ENA, IN1, IN2, ENB, IN3, IN4, HALL_A1, HALL_A2, HALL_B1, HALL_B2):
        # Motor Right
        self.ENA = ENA
        self.IN1 = IN1
        self.IN2 = IN2

        # Motor Left
        self.ENB = ENB
        self.IN3 = IN3
        self.IN4 = IN4

        # Hall Wires
        self.HALL_A1 = HALL_A1
        self.HALL_A2 = HALL_A2

        self.HALL_B1 = HALL_B1
        self.HALL_B2 = HALL_B2

        # Pin setup
        self.setup_pins()

        # Debounce time for HALL feedbacl
        self.debounce_time = 0.01

        # PWM setup
        # self.setup_pwm()

        # Register the exit handler
        atexit.register(self.exit_handler)

    def setup_pins(self):
        # Set the mode of numbering the pins
        GPIO.setmode(GPIO.BOARD)

        # Set up the motor pins
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)
       
        # Set up HALL pins
        GPIO.setup(self.HALL_A1, GPIO.IN)
        GPIO.setup(self.HALL_A2, GPIO.IN)
        GPIO.setup(self.HALL_B1, GPIO.IN)
        GPIO.setup(self.HALL_B2, GPIO.IN)

    def setup_pwm(self, frequency = 100):
        # Set PWM frequency in Hz
        self.PWM_A = GPIO.PWM(self.ENA, frequency)  
        self.PWM_B = GPIO.PWM(self.ENB, frequency)

        self.PWM_A.start(0)
        self.PWM_B.start(0)

    def control(self, motor, speed, reverse, stop=False):
        if motor == 'A':
            # pwm = self.PWM_A
            IN1 = self.IN1
            IN2 = self.IN2
        elif motor == 'B':
            # pwm = self.PWM_B
            IN1 = self.IN3
            IN2 = self.IN4
        else:
            print(f'Unknown Motor: {motor}')
            return

        # pwm.ChangeDutyCycle(speed)
        
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

        if stop == True:
            return

        if reverse == False:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)

    def hall_feedback(self, stop_feedback):
        # Initialize counters for each hall sensor wire
        count_A1 = 0
        count_A2 = 0
        count_B1 = 0
        count_B2 = 0

        # Callback functions to increment counters for each hall sensor wire
        def callback_A1(channel):
            nonlocal count_A1
            time.sleep(self.debounce_time)
            if GPIO.input(channel):
                count_A1 += 1

        def callback_A2(channel):
            nonlocal count_A2
            time.sleep(self.debounce_time)
            if GPIO.input(channel):
                count_A2 += 1

        def callback_B1(channel):
            nonlocal count_B1
            time.sleep(self.debounce_time)
            if GPIO.input(channel):
                count_B1 += 1

        def callback_B2(channel):
            nonlocal count_B2
            time.sleep(self.debounce_time)
            if GPIO.input(channel):
                count_B2 += 1

        # Attach callbacks to GPIO inputs
        GPIO.add_event_detect(self.HALL_A1, GPIO.RISING, callback=callback_A1)
        GPIO.add_event_detect(self.HALL_A2, GPIO.RISING, callback=callback_A2)
        GPIO.add_event_detect(self.HALL_B1, GPIO.RISING, callback=callback_B1)
        GPIO.add_event_detect(self.HALL_B2, GPIO.RISING, callback=callback_B2)

        # Wait until stop_feedback is called usually after scan completion
        stop_feedback.wait()

        # Detach callbacks to stop counting
        GPIO.remove_event_detect(self.HALL_A1)
        GPIO.remove_event_detect(self.HALL_A2)
        GPIO.remove_event_detect(self.HALL_B1)
        GPIO.remove_event_detect(self.HALL_B2)

        # Calculate total counts for each motor by summing the counts from both sensors
        total_count_A = count_A1 + count_A2
        total_count_B = count_B1 + count_B2

        # Calculate rotations for each motor
        rotations_A = (total_count_A / 24) # Divide by PPR (2 pulses per revolution)
        rotations_B = (total_count_B / 24) # Divide by PPR (2 pulses per revolution)

        return round(rotations_A, 4), round(rotations_B, 4)
    
    def exit_handler(self):
        # Stop motors
        # self.PWM_A.stop()
        # self.PWM_B.stop()
        GPIO.cleanup()

class MotorInterface:
    def __init__(self, driver, wheel_radius, wheelbase, ):
        self.driver = driver
        self.wheel_radius = wheel_radius
        self.wheelbase = wheelbase

    def control(self, pwm_A, pwm_B):
        if pwm_A < 0:
            self.driver.control("A", -pwm_A, False)
        elif pwm_A > 0:
            self.driver.control("A", pwm_A, True)
        else:
            self.driver.control("A", 0, False, stop=True)

        if pwm_B < 0:
            self.driver.control("B", -pwm_B, False)
        elif pwm_B > 9:
            self.driver.control("B", pwm_B, True)
        else:
            self.driver.control("B", 0, False, stop=True)

    def get_tracking_offset(self, results_queue, scan_complete):
        # Get RPM of each motor
        t1 = time.time()
        rot_A, rot_B = self.driver.hall_feedback(scan_complete)

        print(f"{rot_A} | {rot_B} | {round(time.time()-t1, 3)}")

        # Calculate the circumference of the wheel
        wheel_circumference = 2 * math.pi * self.wheel_radius

        # Calculate distance traveled by each wheel
        distance_A = rot_A * wheel_circumference
        distance_B = rot_B * wheel_circumference

        # Calculate total distance traveled by the robot
        total_distance = (distance_A + distance_B) / 2

        # Calculate the actual angular change
        # The difference in distance divided by the wheelbase gives the angular change in radians
        angular_change_rad = (distance_B - distance_A) / self.wheelbase
        angular_change_deg = math.degrees(angular_change_rad)

        # Put results in queue
        results_queue.put(( total_distance, angular_change_deg))
    