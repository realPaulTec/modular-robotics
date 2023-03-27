#!../.venv/bin/python3.11

from component import Component
import matplotlib.pyplot as plt
import numpy as np
import pyrplidar
import time
import atexit
import threading
import cv2


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
        self.MAX_MATRIX_DIFFERENCE = 20

        self.MAX_LIFETIME_SECONDS = 1.0
        self.ACCEPTED_TIME = 0.2
        
        # Weight
        self.wCoordinates = 1.0
        self.wBounds = 0.3
        self.wArea = 0.4


        # ===== UI constants ======================================

        # The color map (which displays the amount of scans in a grid square) in the UI
        self.COLOR_MAP = 'plasma'

        # Minimal amount of scans in matrix grid square for the UI
        self.DISPLAY_THRESHOLD = 3

        # ID color range
        self.MAX_VISUALIZATION = 8
        
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

        # Starting the LiDAR data handler.
        self.handler = self.lidar.start_scan_express(2)


        # ===== Graphing setup ====================================

        # Setting up the graphing matrix
        self.graphingMatrix = np.zeros((self.RESOLUTION_Y, self.RESOLUTION_X))

        if __name__ == "__main__":
            # Setting up the plot for the LiDAR visualization
            self.fig = plt.figure(facecolor='black')
            self.matrixVisualization = self.fig.add_subplot()
            self.matrixVisualization.tick_params(which='both', colors='#55ddffff')


        # ===== General setup =====================================

        self.LiDARcomponents = []
    
        max_x = self.MAX_DISTANCE_METERS / np.cos(np.pi / 4)
        max_y = self.MAX_DISTANCE_METERS / np.sin(np.pi / 4)
        self.max_x_y = max(max_x, max_y) * 1000

        self.ΔTime = 0

    def update(self):
        self.graphing()

    def on_exit(self):
        self.lidar.set_motor_pwm(0)
        self.lidar.disconnect()

    def graphing(self):
        # Graphing the LiDAR output
        with self.matrix_lock:
            self.matrixVisualization.clear()
            self.matrixVisualization.imshow(self.graphingMatrix, cmap = self.COLOR_MAP, vmin = 0, vmax = self.MAX_VISUALIZATION)

        plt.pause(0.01)

    def get_scan(self, handler):
        # Resetting matrix for new scan.
        matrix = np.zeros((self.RESOLUTION_Y, self.RESOLUTION_X)).astype(np.int16)
        
        # Processing the data from the LiDAR in a handler from the PyRPlidar library!
        for count, scan in enumerate(handler()):

            # Quick distance approximation for performance reasons. || Don't want to run trigonometric functions if they aren't necessary!
            if scan.distance != 0 and scan.distance < self.max_x_y:
                matrix = self.get_scan_matrix(matrix, scan)
            
            # Breaking for loop after surpassing set sample rate.
            if count == self.SAMPLE_RATE: break

        return matrix

    def scan_thread(self, render=False):
        global graphingMatrix

        while True:
            try:
                self.scan()
                LiDARimage = self.render_scan(self.LiDARcomponents)

                if render == True:
                    # Synchronizing with main thread and graphing the matrix in matplotlib. 
                    with self.matrix_lock:
                        self.graphingMatrix = LiDARimage
                else:
                    self.graphingMatrix = LiDARimage
            except TypeError:
                break

    def scan(self): 
        # Getting first time for Δt in seconds.
        primaryTime = time.time()

        matrix = self.get_scan(self.handler)

        # Processing the scan & getting the processed matrix. 
        newComponents = self.process_scan(matrix)

        # Filtering the LiDAR components, compared to their historical counterparts. 
        if len(self.LiDARcomponents) > 0:
            self.LiDARcomponents = self.filter_components(newComponents)
        else:
            for component in newComponents:
                component.generate_id()

            self.LiDARcomponents = newComponents

        # Calculating Δt in seconds.
        secondaryTime = time.time()
        self.ΔTime = round(secondaryTime - primaryTime, 2)

        return self.LiDARcomponents

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
        componentsN = []
            
        # Making matrix binary and getting individual components
        processingMatrix = (matrix >= self.THRESHOLD_SCANS_PROCESSING).astype(np.uint8)
        processingMatrix = cv2.morphologyEx(processingMatrix, cv2.MORPH_CLOSE, self.MORPH_KERNEL, iterations=self.MORPHOLOGY_STEPS)
        processingMatrix = cv2.dilate(processingMatrix, self.DILATION_KERNEL, iterations=self.DILATION_STEPS)

        # Generating connected components with OpenCV spaghetti algorithm
        connectedLiDAR = cv2.connectedComponentsWithStats(processingMatrix, 1, cv2.CV_32S)

        # Getting individual components from LiDAR scan        
        components = connectedLiDAR[2]
        totalComponents = connectedLiDAR[0]

        # Making a dictionary of the components.
        for i in range(totalComponents - 1):
            if components[i + 1, cv2.CC_STAT_AREA] >= self.AREA_THRESHOLD:
                component = Component(
                    components[i + 1, cv2.CC_STAT_LEFT],        # x coordinate
                    components[i + 1, cv2.CC_STAT_TOP],         # y coordinate
                    components[i + 1, cv2.CC_STAT_WIDTH],       # Width
                    components[i + 1, cv2.CC_STAT_HEIGHT],      # Height
                    components[i + 1, cv2.CC_STAT_AREA],        # Area
                    np.array((connectedLiDAR[1] == (i + 1)).astype(np.uint8))   # Geometry (matrix)
                )
                
                componentsN.append(component)

        return np.array(componentsN)
        
    def render_scan(self, components):
        # Making a new matrix to drawn on with the filtered components!
        canvasMatrix = np.zeros((self.RESOLUTION_Y, self.RESOLUTION_X)).astype(np.uint8)

        for component in components:
            # Get the component geometry which acts as a "Paintbrush"
            brush = component.geometry
            brush[brush > 0] = component.ID % self.MAX_VISUALIZATION

            # Draw the component
            canvasMatrix += brush

        return canvasMatrix

    def print_output(self):
        print('==NEXT======================================NEXT==')

        for component in self.LiDARcomponents:
            print(f"COMPONENT {component.ID}: \n x: {component.x} | y: {component.y} | width: {component.width} | height: {component.height}")

        print('Δt: %ss' %(format((self.ΔTime), '.2f')))

    def filter_components(self, newComponents):
        newComponents = np.array(newComponents)

        for component in self.LiDARcomponents:
            index, difference = component.check_similarity_array(newComponents, self.wCoordinates, self.wArea, self.wBounds)

            if index != None and difference != None and difference <= self.MAX_MATRIX_DIFFERENCE:
                newComponents[int(index)].update_component(component.ID, difference)

            elif time.time() - component.lastUpdate < self.MAX_LIFETIME_SECONDS:
                newComponents = np.append(newComponents, component)

        # Generate IDs for all new components
        for component in newComponents:
            if component.ID == None:
                component.generate_id()

        return np.array(newComponents)

if __name__ == "__main__":
    font = {'family' : 'URW Gothic',
        'weight' : 'bold',
        'size'   : 14}

    plt.rc('font', **font)

    currentLiDAR = LiDAR()

    # Setting up and starting the scan thread
    scanThread = threading.Thread(target = currentLiDAR.scan_thread, kwargs={'render': True})
    scanThread.daemon = True
    scanThread.start()

    while True:
        try:
            currentLiDAR.update()
            currentLiDAR.print_output()
        except KeyboardInterrupt:
            break
