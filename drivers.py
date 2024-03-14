import math
import Jetson.GPIO as GPIO
import atexit
import time
import numpy as np
import smbus

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

        # print(f"{rot_A} | {rot_B} | {round(time.time()-t1, 3)}")

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

class GyroInterface:
    def __init__(self, motor_interface, bus=1, address=0x68, mag_address=0x0C):
        # Interface with MPU9250
        self.motor_interface = motor_interface
        self.bus = smbus.SMBus(bus)
        self.address = address
        self.mag_address = mag_address

        # Setup offsets and scale factors for calibration
        self.offsets = np.zeros(3)
        self.scale_factors = np.zeros(3)

        # Enable bypass mode to access magnetometer
        self.bus.write_byte_data(self.address, 0x37, 0x02)
        
        # Set magnetometer to continuous measurement mode
        self.bus.write_byte_data(self.mag_address, 0x0A, 0x16)
        
        # Short delay
        time.sleep(0.1)

    def calibrate(self, angular_threshold=5, sample_rate=100, sample_threshold=60):
        print("Calibrating... Ensure the robot moves in a circle.")

        # Start moving in a circle
        self.motor_interface.control(-100, 100)

        initial_readings = self.read_mag_data()
        initial_heading = round(self.calculate_heading(initial_readings))
        readings = [initial_readings]

        samples = 0
        while True:
            samples += 1
            
            # Stop motor for readings
            self.motor_interface.control(0,0)
            
            current_readings = []
            current_headings = []
            for i in range(sample_rate):
                # Get readings and heading
                data = np.array(self.read_mag_data())
                
                # Update headings and readings
                current_readings.append(data)
                current_headings.append(round(self.calculate_heading(data)))
                
                # Delay for sensor
                time.sleep(0.001)

            self.motor_interface.control(-100, 100)

            # Get mean of current readings and headings
            current_reading = np.median(np.array(current_readings), axis=0)
            current_heading = np.median(current_headings)
            
            # Update headings for calibration
            readings.append(current_reading)

            # The robot turns ~64 times, with 5,625 degrees 
            time.sleep(0.1)

            print(current_heading)

            # Break if the robot turned in a circle once 
            if np.abs(current_heading - initial_heading) < angular_threshold and samples > sample_threshold:
                self.motor_interface.control(0, 0)
                break

        # Convert to numpy array for easier manipulation
        hall_array = np.array(readings)

        # Normalize
        hall_mean = np.mean(hall_array, axis=0)
        hall_std = np.std(hall_array, axis=0)

        normalized_hall = (hall_array - hall_mean) / hall_std
        
        # Find min and max for each axis
        min_vals = np.min(normalized_hall, axis=0)
        max_vals = np.max(normalized_hall, axis=0)

        # Calculate offsets
        self.offsets = (max_vals + min_vals) / 2

        # Calculate scale factors
        self.scale_factors = 2 / (max_vals - min_vals)

        print(f"Calibration complete... I{initial_heading} E{current_heading}")

    def read_mag_data(self):
        # Read magnetometer data
        data = self.bus.read_i2c_block_data(self.mag_address, 0x03, 7)
        x = data[0] * 256 + data[1]
        if x > 32767:
            x -= 65536
        y = data[2] * 256 + data[3]
        if y > 32767:
            y -= 65536
        z = data[4] * 256 + data[5]
        if z > 32767:
            z -= 65536
        return np.array([x, y, z])

    def calculate_heading(self, hall_reading):
        return math.degrees(math.atan2(hall_reading[1], hall_reading[0])) % 360
    
    def get_calibrated_heading(self, samples=20):
        readings = []
        for i in range(samples):
            readings.append(self.read_mag_data())

        data = np.median(np.array(readings), axis=0)

        return self.calculate_heading(self.apply_calibration(data))

    def apply_calibration(self, hall_reading):
        return (hall_reading - self.offsets) * self.scale_factors