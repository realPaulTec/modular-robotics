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
        self.RESOLUTION_X = 85
        self.RESOLUTION_Y = 85

        # Setting the max distance
        self.MAX_DISTANCE_METERS = 2

        # Steps of primary dilation and secondary morphology close
        self.DILATION_STEPS = 1
        self.MORPHOLOGY_STEPS = 1

        # Dilation and morphology kernels
        self.DILATION_KERNEL = np.ones((3, 3), np.uint8) 
        self.MORPH_KERNEL = np.ones((5, 5), np.uint8)

        # Minimal area for processing
        self.AREA_THRESHOLD = 4

        # Minimal amount of scans in matrix grid square
        self.THRESHOLD_SCANS_PROCESSING = 2

        # How similar do two matrices have to be to be considered one?
        # NOTE: Implement WEIGHT!
        self.MAX_MATRIX_DIFFERENCE = 4

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
    
        max_x = self.MAX_DISTANCE_METERS / np.cos(np.pi / 4)
        max_y = self.MAX_DISTANCE_METERS / np.sin(np.pi / 4)
        self.max_x_y = max(max_x, max_y) * 1000


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

        # Starting the LiDAR data handler.
        handler = self.lidar.start_scan_express(2)

        while True: 
            # Getting first time for Δt in seconds.
            primaryTime = time.time()

            # Resetting matrix for new scan.
            matrix = np.zeros((self.RESOLUTION_Y, self.RESOLUTION_X)).astype(np.int16)
            
            # Processing the data from the LiDAR in a handler from the PyRPlidar library!
            for count, scan in enumerate(handler()):

                # Quick distance approximation for performance reasons. || Don't want to run trigonometric functions if they aren't necessary!
                if scan.distance != 0 and scan.distance < self.max_x_y:
                    matrix = self.get_scan_matrix(matrix, scan)
                
                # Breaking for loop after surpassing set sample rate.
                if count == self.SAMPLE_RATE: break

            print('==NEXT======================================NEXT==')

            # Processing the scan & getting the processed matrix. 
            matrix = self.process_scan(matrix)

            # Printing Δt in seconds.
            secondaryTime = time.time()
            print('Delta Time: %ss' %(format((secondaryTime - primaryTime), '.2f')))


    def get_scan_matrix(self, matrix, scan):
        # Getting hit coordinates relative to LiDAR position in meters!
        primary_x = np.cos(np.deg2rad(scan.angle)) * (scan.distance / 1000)
        primary_y = np.sin(np.deg2rad(scan.angle)) * (scan.distance / 1000)

        # Only calculating matrix indices if they are within the max. bounds.
        if np.abs(primary_x) < self.MAX_DISTANCE_METERS and np.abs(primary_y) < self.MAX_DISTANCE_METERS:
            # Getting indices in matrix for coordinates, relative to the center coordinate.
            x = np.round(primary_x / self.MAX_DISTANCE_METERS * ((self.RESOLUTION_X - 1) / 2)).astype(np.int16)
            y = np.round(primary_y / self.MAX_DISTANCE_METERS * ((self.RESOLUTION_Y - 1) / 2)).astype(np.int16)
           
            # Getting absolute grid coordinate in matrix
            index_x = np.round(self.RESOLUTION_X / 2).astype(np.int16) + x - 1
            index_y = np.round(self.RESOLUTION_Y / 2).astype(np.int16) + y - 1

            # Adding one scan to the matrix grd square.
            matrix[index_x, index_y] += 1

        return matrix


    def process_scan(self, matrix):
        LiDARcomponents = []
        LiDARmatrices = []
            
        # Making matrix binary and getting individual components
        processingMatrix = (matrix >= self.THRESHOLD_SCANS_PROCESSING).astype(np.uint8)
        processingMatrix = cv2.morphologyEx(processingMatrix, cv2.MORPH_CLOSE, self.MORPH_KERNEL, iterations=self.MORPHOLOGY_STEPS)
        processingMatrix = cv2.dilate(processingMatrix, self.DILATION_KERNEL, iterations=self.DILATION_STEPS)

        # Generating connected components with OpenCV spaghetti algorithm
        connectedLiDAR = cv2.connectedComponentsWithStats(processingMatrix, 1, cv2.CV_32S)

        # Getting individual components from LiDAR scan        
        components = connectedLiDAR[2]
        totalComponents = connectedLiDAR[0]

        # Making a dictionary of the components. NOTE: Change to np.array for performance reasons.
        for i in range(totalComponents - 1):
            if components[i + 1, cv2.CC_STAT_AREA] >= self.AREA_THRESHOLD:
                component = np.array([
                    i,                                          # Index | Acts as ID
                    components[i + 1, cv2.CC_STAT_LEFT],        # x coordinate
                    components[i + 1, cv2.CC_STAT_TOP],         # y coordinate
                    components[i + 1, cv2.CC_STAT_WIDTH],       # Width
                    components[i + 1, cv2.CC_STAT_HEIGHT],      # Height
                    components[i + 1, cv2.CC_STAT_AREA],        # Area
                ])
                
                LiDARcomponents.append(component)
                LiDARmatrices.append([
                    i,                                          # Index
                    (connectedLiDAR[1] == i).astype(np.uint8)   # Matrix
                ])

        LiDARcomponents = np.array(LiDARcomponents)

        # Filtering the LiDAR components, compared to their historical counterparts. 
        if len(self.history) > 0:
            self.filter(LiDARcomponents)

        # On the first run, the history will be appended from the process_scan function.
        else:
            self.history.append(np.array(LiDARcomponents))

        # Synchronizing with main thread and graphing the matrix in matplotlib. 
        with self.matrix_lock:
            self.graphingMatrix = connectedLiDAR[1]


    def filter(self, components):
        stepArray = []

        # Checking which new matrix is the closest to a historical matrix.
        for historicalMatrix in self.history[-1]:
            filterArray = []

            for componentMatrix in components:
                # Subtract the historical matrix from the current one; The smaller the value, the more similar they are! || NOTE: Implement WEIGHT for the individual components!!!
                bufferArray = np.abs(historicalMatrix - componentMatrix)
                mean = np.fix(np.mean(bufferArray))

                filterArray.append(np.array([componentMatrix[0], mean]))

            # Sort the filterArray value, while keeping the index. In this case the sortedArray represents how similar each new matrix is to the historical one!
            filterArray = np.array(filterArray)
            sortedIndices = np.argsort(filterArray[:, 1])
            sortedArray = filterArray[sortedIndices]

            # For the arrays to be seen as the same, the median of their differences must be below a certain threshold.
            if sortedArray[0, 1] <= self.MAX_MATRIX_DIFFERENCE:
                stepArray.append(np.array([historicalMatrix[0], sortedArray[0, 0], sortedArray[0, 1]]))
        
        stepArray = np.array(stepArray)

        # Creating a new array which should incorporate the sorted values.
        maxValueHistorical = np.max(stepArray[:, 0])
        maxValueModern = np.max(stepArray[:, 1])
        maxValue = int(max(maxValueHistorical, maxValueModern) + 1)
        
        finalArray = np.zeros((maxValue, len(components[0] + 1)))

        # Incorporating the values. 
        for component in stepArray:
            finalArray[int(component[0])] = components[int(component[1])][:]
        
        # Removing all zero-rows!
        mask_zero = np.any(finalArray != 0, axis=1)
        finalArray = finalArray[mask_zero]

        print(f'Pre-final: \n {finalArray}')

        for component in components:
            if component not in finalArray:
                finalArray = np.append(finalArray, component)

        print(f'Final: \n {finalArray}')

        self.history.append(finalArray)

        if len(self.history) >= 10:
            self.history.pop(0)


class Component:
    def __init__(self, x, y, w, h, a, g):
        # Initializing this component
        self.x = x
        self.y = y
        self.width = w
        self.height = h
        self.area = a
        self.geometry = g

        self.matrix = self.generate_matrix()

        self.history = []

    def generate_matrix(self):
        matrix = np.array([
            self.x,
            self.y,
            self.width,
            self.height,
            self.area
        ])
        
        return matrix
    
    def set_to_other(self, component):
        # Set the core variables of this class to the values of the new component
        self.history.append(self.matrix)

        self.x = component.x
        self.y = component.y
        self.width = component.width
        self.height = component.height
        self.area = component.area
        self.geometry = component.geometry

    def check_similarity_single(self, input_class, max_difference):
        self.matrix = self.generate_matrix()
        inputMatrix = input_class.generate_matrix()
        
        bufferArray = np.abs(inputMatrix - self.matrix)
        mean = np.fix(np.mean(bufferArray)) 

        return mean

    def check_similarity_array(self, input_array, max_difference):
        similarityList = []
        
        # We want to figure out which component in an array is the most similar to this class!
        for i, component in enumerate(input_array):
            mean = self.check_similarity_array(component, max_difference)

            # Check if the mean is below the maximum allowed difference between arrays.
            if mean <= max_difference:
                similarityList.append(np.array[i, mean])

        # Converting the list to a numpy array and sorting it | lower values represent higher similarity | return the most similar value
        similarityArray = np.array(similarityList)
        similarityArray = similarityArray[np.argsort(similarityArray[:, 1])]

        return similarityArray[0]
            


currentLiDAR = LiDAR()

while True:
    try:
        currentLiDAR.update()
    except KeyboardInterrupt:
        break
