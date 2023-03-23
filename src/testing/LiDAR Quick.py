#!../.venv/bin/python3.11

import pyrplidar
import time
import atexit
import threading
import cv2

import matplotlib.pyplot as plt
import numpy as np

# Setting constants || Resolution should be an uneven number
RESOLUTION_X = 155
RESOLUTION_Y = 155

DISPLAY_THRESHOLD = 1
MAX_DISTANCE_METERS = 2

# MAX: 65536 due to int16 matrix || Should be between 600 - 7200 || 1200 samples is ideal / good enough
SAMPLE_RATE = round(3600 / 3)

COLOR_MAP = 'plasma'

# Matrix visualization chars
ONES_CHAR = "\u25A0"  # Unicode character for a square
ZEROS_CHAR = "\u25CB"  # Unicode character for a circle

# Constants for processing
THRESHOLD_SCANS_PROCESSING = 10
DILATION_STEPS = 2
MORPHOLOGY_STEPS = 4

AREA_THRESHOLD = 10

DILATION_KERNEL = np.ones((2, 2), np.uint8) 
MORPH_KERNEL = np.ones((3, 3), np.uint8)

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

# Setting up LiDAR output and graphing lists
distance, angle = [], []
distanceGraph, angleGraph = [], []

graphingMatrix = np.zeros((RESOLUTION_Y, RESOLUTION_X))

# Setting up the plot for the LiDAR visualization
fig = plt.figure()
matrixVisualization = fig.add_subplot()

# Setting up thread locks for thread synchronization
matrix_lock = threading.Lock()

def scan_thread():
    global graphingMatrix

    # Starting the LiDAR data handlerMATRIX_DILATION
    handler = lidar.start_scan_express(2)

    max_x = MAX_DISTANCE_METERS / np.cos(np.pi / 4)
    max_y = MAX_DISTANCE_METERS / np.sin(np.pi / 4)
    max_x_y = max(max_x, max_y) * 1000

    while True: 
        matrix = np.zeros((RESOLUTION_Y, RESOLUTION_X)).astype(np.int16)
        primaryTime = time.time()
        
        # Processing the data from the LiDAR
        for count, scan in enumerate(handler()):
            if scan.distance != 0 and scan.distance < max_x_y:
                primary_x = np.cos(np.deg2rad(scan.angle)) * (scan.distance / 1000)
                primary_y = np.sin(np.deg2rad(scan.angle)) * (scan.distance / 1000)

                # print('x: %s || y: %s' %(primary_x, primary_y))

                if np.abs(primary_x) < MAX_DISTANCE_METERS and np.abs(primary_y) < MAX_DISTANCE_METERS:
                    x = np.round(primary_x / MAX_DISTANCE_METERS * ((RESOLUTION_X - 1) / 2)).astype(np.int16)
                    y = np.round(primary_y / MAX_DISTANCE_METERS * ((RESOLUTION_Y - 1) / 2)).astype(np.int16)
                
                    index_x = np.round(RESOLUTION_X / 2).astype(np.int16) + x - 1
                    index_y = np.round(RESOLUTION_Y / 2).astype(np.int16) + y - 1

                    matrix[index_x, index_y] += 1

            if count == SAMPLE_RATE: break

        with matrix_lock:
            print('==NEXT======================================NEXT==')

            LiDARcomponents = []
            secondaryTime = time.time()
            
            # Dilating and thresholding matrix for user interface || Helpful with large resolutions
            graphingMatrix[matrix < DISPLAY_THRESHOLD] = 0
            
            # Making matrix binary and getting individual components
            processingMatrix = (matrix > 0).astype(np.uint8)
            processingMatrix = cv2.dilate(processingMatrix, DILATION_KERNEL, iterations=DILATION_STEPS)
            processingMatrix = cv2.morphologyEx(processingMatrix, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=MORPHOLOGY_STEPS)

            # Generating connected components with OpenCV spaghetti algorithm
            connectedLiDAR = cv2.connectedComponentsWithStats(processingMatrix, 1, cv2.CV_32S)

            # Getting individual components from LiDAR scan        
            components = connectedLiDAR[2]
            totalComponents = connectedLiDAR[0]

            graphingMatrix = connectedLiDAR[1] # (connectedLiDAR[1] == 2).astype(np.uint8)

            for i in range(totalComponents - 1):
                if components[i + 1, cv2.CC_STAT_AREA] >= AREA_THRESHOLD:
                    component = {
                        'index'     : i,
                        'x'         : components[i + 1, cv2.CC_STAT_LEFT],
                        'y'         : components[i + 1, cv2.CC_STAT_TOP],
                        'width'     : components[i + 1, cv2.CC_STAT_WIDTH],
                        'height'    : components[i + 1, cv2.CC_STAT_HEIGHT],
                        'area'      : components[i + 1, cv2.CC_STAT_AREA],
                        'matrix'    : (connectedLiDAR[1] == i).astype(np.uint8)
                    }
                    
                    LiDARcomponents.append(component)

                    print('I: %s || x: %s || y: %s || a: %s' %(component['index'], component['x'], component['y'], component['area']))

            graphingMatrix = processingMatrix

            # Printing delta time
            print('Delta Time: %ss' %(format((secondaryTime - primaryTime), '.2f')))

def graphing():
    global graphingMatrix

    # Graphing the LiDAR output
    with matrix_lock:
        matrixVisualization.clear()
        matrixVisualization.imshow(graphingMatrix, cmap = COLOR_MAP)

    plt.pause(0.01)

scanThread = threading.Thread(target = scan_thread)
scanThread.daemon = True

scanThread.start()

while True:
    graphing()
