import atexit
import time
import pyrplidar
import math
import numpy as np
import asyncio
import threading
import queue

class Lidar:
    SCAN_MODE = 2 # 3
    MOTOR_PWM = 800

    def __init__(self, SAMPLE_RATE, MAX_DISTANCE_METERS):
        # LiDAR constants
        self.SAMPLE_RATE = round(SAMPLE_RATE)
        self.MAX_DISTANCE_METERS = MAX_DISTANCE_METERS

        # Connecting to lidar hardware
        self.lidar = pyrplidar.PyRPlidar()
        self.lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        
        # Getting lidar status
        health = self.lidar.get_health()
        print(f"status: {health.status}, error code: {health.error_code}")

        # Exit handler to disable lidar scanning and motor
        atexit.register(self.exit_handler)

        # Spinning up lidar motor
        self.lidar.set_motor_pwm(self.MOTOR_PWM)
        time.sleep(2)

        # Setting up scan handler
        self.handler = self.lidar.start_scan_express(self.SCAN_MODE)

        # Setup the asyncio event loop for the instance
        self.loop = asyncio.get_event_loop()
        self.data_queue = queue.Queue()
        
        # Ensure continuous_scanning and process_data coroutines are scheduled
        asyncio.ensure_future(self.continuous_scanning())
        self.thread = threading.Thread(target=self.start_background_loop)
        self.thread.daemon = True
        self.thread.start()

    def exit_handler(self):
        # Stop and disconnect LiDAR on program termination
        self.lidar.set_motor_pwm(0)
        self.lidar.disconnect()
    
    def start_background_loop(self):
        self.loop.run_forever()

    async def continuous_scanning(self):
        data = []
        for count, scan in enumerate(self.handler()):
            if 0 < scan.distance < self.MAX_DISTANCE_METERS * 1000:
                data.append((scan.distance / 1000, -np.deg2rad(scan.angle)))
            if count % (self.SAMPLE_RATE-1) == 0:
                self.data_queue.queue.clear()
                self.data_queue.put(np.array(data))
                data = []
    
    # TODO Fetch and process only the last SAMPLE_RATE samples    
    def fetch_scan_data(self):
        return self.data_queue.get()
