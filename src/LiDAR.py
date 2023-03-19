#!../.venv/bin/python3.11

import pyrplidar
import time
import atexit
import threading
import cv2

import matplotlib.pyplot as plt
import numpy as np

# Setting constants || Should be an uneven number
RESOLUTION_X = 55
RESOLUTION_Y = 55

MATRIX_VISUALISATION_DIALATION = 0
DISPLAY_THRESHOLD = 1

MAX_DISTANCE_METERS = 2

SAMPLE_RATE = 4000

COLOR_MAP = 'plasma'

# Matrix visualisation chars
ONES_CHAR = "\u25A0"  # Unicode character for a square
ZEROS_CHAR = "\u25CB"  # Unicode character for a circle

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
lidar.set_motor_pwm(800)
time.sleep(2)

# Setting up LiDAR output and graphing lists
distance, angle = [], []
distanceGraph, angleGraph = [], []

graphingMatrix = np.zeros((RESOLUTION_Y, RESOLUTION_X))

# Setting up the plot for the LiDAR visualisation
fig = plt.figure()
axis = fig.add_subplot(121, projection = 'polar')
matrixVisualisation = fig.add_subplot(122)

# Setting up thread locks for thread synchronysation
distance_lock = threading.Lock()
angle_lock = threading.Lock()
matrix_lock = threading.Lock()

def scan_thread():
    global distanceGraph, angleGraph

    # Starting the LiDAR data handler
    handler = lidar.start_scan_express(2)

    while True:
        # Processing the data from the LiDAR
        for count, scan in enumerate(handler()):
            with distance_lock: 
                distance.append(scan.distance / 1000)
            with angle_lock:
                angle.append(np.deg2rad(scan.angle))

            if count == SAMPLE_RATE: break

        with distance_lock, angle_lock:
            distanceGraph, angleGraph = distance, angle
            
            data_processing_lidar(angle, distance)
            distance.clear(), angle.clear()

def data_processing_lidar(angleData, distanceData):
    global graphingMatrix

    # Setting up numpy matrix for data processing
    matrix = np.zeros((RESOLUTION_Y, RESOLUTION_X))
    
    for i, angle in enumerate(angleData):
        primary_x = np.cos(angle) * distanceData[i]
        primary_y = np.sin(angle) * distanceData[i]

        if abs(primary_x) < MAX_DISTANCE_METERS and abs(primary_y) < MAX_DISTANCE_METERS and distanceData[i] != 0:
            x = round(primary_x / MAX_DISTANCE_METERS * (RESOLUTION_X / 2))
            y = round(primary_y / MAX_DISTANCE_METERS * (RESOLUTION_Y / 2))
        
            index_x = min(round(RESOLUTION_X / 2) + x, RESOLUTION_X - 1)
            index_y = min(round(RESOLUTION_Y / 2) + y, RESOLUTION_Y - 1)
            
            # print('x: %s || y: %s' %(x, y))

            matrix[index_x, index_y] += 1
    
    with matrix_lock:
        matrix[matrix < DISPLAY_THRESHOLD] = 0
        graphingMatrix = cv2.dilate(matrix, None, iterations = MATRIX_VISUALISATION_DIALATION)

    # printMatrix = np.where(matrix == 1, ONES_CHAR, ZEROS_CHAR)
    # print(matrix)

def graphing():
    # Graphing the LiDAR output

    with distance_lock, angle_lock:
        axis.clear()
        axis.scatter(angleGraph, distanceGraph, s = 1)
        axis.set_ybound(0, MAX_DISTANCE_METERS)

    with matrix_lock:
        matrixVisualisation.clear()
        matrixVisualisation.imshow(graphingMatrix, cmap = COLOR_MAP)

    plt.pause(0.01)

scanThread = threading.Thread(target = scan_thread)
scanThread.daemon = True

scanThread.start()

while True:
    graphing()
