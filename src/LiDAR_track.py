import pyrplidar
import time
import atexit
import threading
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN

class LidarScanner:
    # scanning constants
    MAX_DISTANCE_METERS = 0.2
    SAMPLE_RATE = 720

    # acquisition constants
    FIXED_DISTANCE = 0.125
    FIXED_ANGLE = np.deg2rad(90)
    RADIUS = 0.05

    # DBSCAN constants
    DBSCAN_EPS = 0.15
    DBSCAN_MIN_SAMPLES = 15

    def __init__(self):
        # lidar and plotting setup
        self.setup_lidar()
        self.setup_plot()
        
        # locks for threading
        self.distance_lock = threading.Lock()
        self.angle_lock = threading.Lock()
        
        # class variables
        self.distance, self.angle = [], []
        self.tracking = False
        self.tracked_point = None
        self.clusters = {}

    def exit_handler(self):
        self.lidar.set_motor_pwm(0)
        self.lidar.disconnect()

    def setup_lidar(self):
        # connecting to lidar hardware
        self.lidar = pyrplidar.PyRPlidar()
        self.lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=3)
        
        # getting lidar status
        health = self.lidar.get_health()
        print(f"status: {health.status}, error code: {health.error_code}")
        
        # exit handler to disable lidar scanning and motor
        atexit.register(self.exit_handler)

        # spinning up lidar motor
        self.lidar.set_motor_pwm(600)
        time.sleep(2)

        # setting up scan handler
        self.handler = self.lidar.start_scan_express(2)

    def setup_plot(self):
        # matplotlib styling for plot
        font = {'family': 'URW Gothic', 'weight': 'bold', 'size': 14}
        plt.rc('font', **font)
        self.fig = plt.figure(facecolor='black')

        # creating polar plot for visualization
        self.axis = self.fig.add_subplot(111, projection='polar')
        self.axis.set_facecolor('black')
        self.axis.tick_params(which='both', colors='#55ddffff')

    def continuous_tracking(self):
        while True:
            # get LiDAR data from scan
            distance, angle = self.perform_scan()

            clusters = self.perform_clustering(distance, angle)
            self.clusters = clusters

            if self.tracking == True:
                self.perform_tracking(clusters)   
            else:
                self.acquire_track(clusters)

            # draw distance and angle for next cycle
            with self.distance_lock, self.angle_lock:
                self.distance.clear(), self.angle.clear()
                self.distance, self.angle = distance, angle

    def perform_clustering(self, distance, angle):
        # DBSCAN clustering
        coordinates = np.array(list(zip(angle, distance)))

        # Use haversine metric for distance calculation
        dbscan = DBSCAN(eps=self.DBSCAN_EPS, min_samples=self.DBSCAN_MIN_SAMPLES, metric='haversine').fit(coordinates)
        labels = dbscan.labels_

        clusters = {}
        for i, label in enumerate(labels):
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(coordinates[i])

        return clusters

    def acquire_track(self, clusters):
        # Logic to acquire track
        pass

    def perform_tracking(self, clusters):
        # Logic to update track
        pass

    def perform_scan(self):
        distance, angle = [], []
    
        for count, scan in enumerate(self.handler()):
            # adding scans below the max range into 
            if scan.distance > 0 and (scan.distance / 1000) < self.MAX_DISTANCE_METERS:
                distance.append(scan.distance / 1000)
                angle.append(np.deg2rad(scan.angle))

            # breaking loop once sample rate is achieved
            if count == self.SAMPLE_RATE:
                return distance, angle

    def graphing(self):
        with self.distance_lock, self.angle_lock:
            # clearing axis and scattering new data
            self.axis.clear()
            self.axis.scatter(self.angle, self.distance, s=5, c="#ff5050")
            
            # Visualizing clusters
            for label, points in self.clusters.items():
                cluster_angles, cluster_distances = zip(*points)
                self.axis.scatter(cluster_angles, cluster_distances, s=20, marker='o')
            
            # Visualizing tracked point
            if self.tracked_point:
                self.axis.scatter([self.tracked_point[0]], [self.tracked_point[1]], s=60, c="#00FF00", marker='x')

            # scaling the axis to the max range
            self.axis.set_ybound(0, self.MAX_DISTANCE_METERS)

            # labeling distance in plot
            self.axis.set_xlabel('distance')
        
        # updating canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == "__main__":
    # generating new lidar class "scanner"
    scanner = LidarScanner()

    # Initialize Matplotlib window
    plt.show(block=False)

    # setting up separate daemon thread for scanning and tracking
    tracking_thread = threading.Thread(target=scanner.continuous_tracking)
    tracking_thread.daemon = True
    tracking_thread.start()
    
    while True:
        scanner.graphing()
        if not plt.get_fignums():
            print("Matplotlib window is closed. Exiting.")
            break
