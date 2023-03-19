#!../.venv/bin/python3.11

import pyrplidar
import time
import atexit
import threading
import cv2

import matplotlib.pyplot as plt
import numpy as np

class LiDAR:
    def __init__(self) -> None:
        # ===== LiDAR constants ===================================
    
        # MAX: 65536 due to int16 matrix || Should be between 600 - 7200 || 1200 samples is ideal / good enough
        self.SAMPLE_RATE = round(3600 / 2)

        # ===== Processing constants ==============================
        
        # Resolution should be an uneven number
        self.RESOLUTION_X = 155
        self.RESOLUTION_Y = 155

        # Setting the max distance
        self.MAX_DISTANCE_METERS = 2

        # Steps of primary dilation and secondary morphology close
        self.DILATION_STEPS = 1
        self.MORPHOLOGY_STEPS = 2

        # Dilation and morphology kernels
        self.DILATION_KERNEL = np.ones((3, 3), np.uint8) 
        self.MORPH_KERNEL = np.ones((7, 7), np.uint8)

        # Minimal area for processing
        self.AREA_THRESHOLD = 4

        # Minimal amount of scans in matrix grid square
        self.THRESHOLD_SCANS_PROCESSING = 1

        # How similar do two matrices have to be to be considered one?
        # NOTE: Implement WEIGHT!
        self.MAX_MATRIX_DIFFERENCE = 2

        # ===== UI constants ======================================

        # The color map (which displays the amount of scans in a grid square) in the UI
        self.COLOR_MAP = 'plasma'

        # Minimal amount of scans in matrix grid square for the UI
        self.DISPLAY_THRESHOLD = 3

        
        # ===== LiDAR setup =======================================
        
        # Setting the exit handler to disconnect from the LiDAR on exit
        atexit.register(self.on_exit)

        # sudo chmod 666 /dev/ttyUSB0
        self.lidar = pyrplidar.PyRPlidar()
        self.lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        
        print(self.lidar.get_health())

        # Spinning up the LiDAR
        self.lidar.set_motor_pwm(600)
        time.sleep(2)

        # Setting up thread locks for thread synchronization
        self.matrix_lock = threading.Lock()

        # Setting up and starting the scan thread
        scanThread = threading.Thread(target = self.scan)
        scanThread.daemon = True
        scanThread.start()


        # ===== Graphing setup ====================================

        # Setting up the graphing matrix
        self.graphingMatrix = np.zeros((self.RESOLUTION_Y, self.RESOLUTION_X))

        # Setting up the plot for the LiDAR visualization
        self.fig = plt.figure()
        self.matrixVisualization = self.fig.add_subplot()


        # ===== General setup =====================================

        self.history = []


    def update(self):
        self.graphing()

    def on_exit(self):
        self.lidar.set_motor_pwm(0)
        self.lidar.disconnect()

    def graphing(self):
        # Graphing the LiDAR output
        with self.matrix_lock:
            self.matrixVisualization.clear()
            self.matrixVisualization.imshow(self.graphingMatrix, cmap = self.COLOR_MAP)

        plt.pause(0.01)

    def scan(self):
        global graphingMatrix

        # Starting the LiDAR data handler MATRIX_DILATION
        handler = self.lidar.start_scan_express(2)

        max_x = self.MAX_DISTANCE_METERS / np.cos(np.pi / 4)
        max_y = self.MAX_DISTANCE_METERS / np.sin(np.pi / 4)
        max_x_y = max(max_x, max_y) * 1000

        while True: 
            matrix = np.zeros((self.RESOLUTION_Y, self.RESOLUTION_X)).astype(np.int16)
            primaryTime = time.time()
            
            # Processing the data from the LiDAR
            for count, scan in enumerate(handler()):
                if scan.distance != 0 and scan.distance < max_x_y:
                    primary_x = np.cos(np.deg2rad(scan.angle)) * (scan.distance / 1000)
                    primary_y = np.sin(np.deg2rad(scan.angle)) * (scan.distance / 1000)

                    # print('x: %s || y: %s' %(primary_x, primary_y))

                    if np.abs(primary_x) < self.MAX_DISTANCE_METERS and np.abs(primary_y) < self.MAX_DISTANCE_METERS:
                        x = np.round(primary_x / self.MAX_DISTANCE_METERS * ((self.RESOLUTION_X - 1) / 2)).astype(np.int16)
                        y = np.round(primary_y / self.MAX_DISTANCE_METERS * ((self.RESOLUTION_Y - 1) / 2)).astype(np.int16)
                    
                        index_x = np.round(self.RESOLUTION_X / 2).astype(np.int16) + x - 1
                        index_y = np.round(self.RESOLUTION_Y / 2).astype(np.int16) + y - 1

                        matrix[index_x, index_y] += 1

                if count == self.SAMPLE_RATE: break

            print('==NEXT======================================NEXT==')

            LiDARcomponents = []
            
            # Making matrix binary and getting individual components
            processingMatrix = (matrix >= self.THRESHOLD_SCANS_PROCESSING).astype(np.uint8)
            processingMatrix = cv2.morphologyEx(processingMatrix, cv2.MORPH_CLOSE, self.MORPH_KERNEL, iterations=self.MORPHOLOGY_STEPS)
            processingMatrix = cv2.dilate(processingMatrix, self.DILATION_KERNEL, iterations=self.DILATION_STEPS)

            # Generating connected components with OpenCV spaghetti algorithm
            connectedLiDAR = cv2.connectedComponentsWithStats(processingMatrix, 1, cv2.CV_32S)

            # Getting individual components from LiDAR scan        
            components = connectedLiDAR[2]
            totalComponents = connectedLiDAR[0]

            for i in range(totalComponents - 1):
                if components[i + 1, cv2.CC_STAT_AREA] >= self.AREA_THRESHOLD:
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

                    # print('I: %s || x: %s || y: %s || a: %s' %(component['index'], component['x'], component['y'], component['area']))

            self.filter(LiDARcomponents)

            with self.matrix_lock:
                self.graphingMatrix = connectedLiDAR[1]

                # Thresholding matrix for user interface || Helpful with large resolutions
                # self.graphingMatrix[matrix < self.DISPLAY_THRESHOLD] = 0

                # Printing delta time
                secondaryTime = time.time()
                print('Delta Time: %ss' %(format((secondaryTime - primaryTime), '.2f')))

    def filter(self, components):
        stepArray = []
        subStepArray = []

        for component in components:
            componentMatrix = np.array([
                component['index'],
                component['x'],
                component['y'],
                component['width'],
                component['height'],
                component['area']
                ])
            
            stepArray.append(componentMatrix)

            if len(self.history) > 0:
                filterArray = []

                for historicalMatrix in self.history[-1]:
                    # Subtract the historical matrix from the current one; The smaller the value, the more similar they are!
                    # NOTE: Implement WEIGHT for the individual components!!!
                    bufferArray = np.abs(componentMatrix - historicalMatrix)

                    mean = np.fix(np.mean(bufferArray))
                    appendArray = np.array([historicalMatrix[0], mean])

                    filterArray.append(appendArray)

                # Sort the filterArray value, while keeping the index. In this case the sortedArray represents how similar each historicalMatrix is to the current one!
                filterArray = np.array(filterArray)
                sortedIndices = np.argsort(filterArray[:, 1])
                sortedArray = filterArray[sortedIndices]

                componentMatrix[0] = sortedArray[0, 0]
                subStepArray.append(componentMatrix)

        if len(subStepArray) > 0:
            sortedSubStepArray = np.zeros((len(subStepArray), 6))

            subStepArray = np.array(subStepArray)
            sortedIndices = np.argsort(subStepArray[0])

            for i in sortedIndices:
                if i < len(subStepArray):
                    sortedSubStepArray[i] = subStepArray[i]

            print(sortedSubStepArray)

        # Avoid using too much memory, popping history.
        if len(self.history) >= 10:
            self.history.pop()
        
        self.history.append(stepArray)

currentLiDAR = LiDAR()

while True:
    currentLiDAR.update()
