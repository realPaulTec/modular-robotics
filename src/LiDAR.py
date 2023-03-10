#!../.venv/bin/python3.11

import pyrplidar
import time
import atexit
import threading

import matplotlib.pyplot as plt
import numpy as np

# sudo chmod 666 /dev/ttyUSB0

lidar = pyrplidar.PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)

health = lidar.get_health()
print('info: ', health)

# Disconnecting on exit
def exit_handler():
    lidar.set_motor_pwm(0)
    lidar.disconnect()

atexit.register(exit_handler)

# Spinning the motor up
lidar.set_motor_pwm(600)
time.sleep(2)

distance, angle = [], []
distanceGraph, angleGraph = [], []

fig = plt.figure()
axis = fig.add_subplot(projection='polar')

distance_lock = threading.Lock()
angle_lock = threading.Lock()

def scan_thread():
    global distanceGraph, angleGraph

    # Starting the LiDAR data handler
    handler = lidar.start_scan_express(2)
    deadScans = 0

    while True:
        # Processing the data
        for count, scan in enumerate(handler()):
            with distance_lock: 
                distance.append(scan.distance / 1000)
            with angle_lock:
                angle.append(np.deg2rad(scan.angle))

            if count == 2000: break

        with distance_lock, angle_lock:
            distanceGraph, angleGraph = distance, angle
            distance.clear(), angle.clear()

            print('Dead Scans: %s' %deadScans)
            deadScans = 0

def graphing():
    with distance_lock, angle_lock:
        axis.clear()
        axis.scatter(angleGraph, distanceGraph, s = 1)
        axis.set_ybound(0, 4)

    plt.pause(0.01)

scanThread = threading.Thread(target = scan_thread)
scanThread.daemon = True

scanThread.start()

while True:
    graphing()
