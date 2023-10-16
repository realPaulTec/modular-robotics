from collections import defaultdict
from matplotlib import patches
import pyrplidar
import time
import atexit
import threading
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN


# old mean angle function (20s): MEAN DELTA TIME: 0.118468
# new mean angle function (20s): MEAN DELTA TIME: 0.11863
# -> no difference

# python list for scanning: 
#
# MEAN DELTA TIME OVER 500 ITERATIONS (6 minutes)
# MAIN: 0.11206554222106933
# SCANNING: 0.10442154598236084
# CLUSTERING: 0.007629129409790039

# np array for scanning: 
#
# MEAN DELTA TIME OVER 500 ITERATIONS (6 minutes)
# MAIN: 0.11229584789276123
# SCANNING: 0.1050695242881775
# CLUSTERING: 0.007212021350860596

# Different Scan Modes (100 iteration mean, 720 samples):
# 0: 0.200039381980896      | bad quality
# 1: 0.1994147777557373     | bad quality
# 2: 0.11259601593017578    |
# 3: 0.11345351457595826    |
# 4: 0.1752121329307556     |

# Different motor speeds (100 iteration mean, 720 samples, mode 2)
# PWM 400   : 0.11333080530166625  | bad quality
# PWM 600   : 0.11225196838378906  |
# PWM 800   : 0.11141704082489014  |
# PWM 1000  : 0.1111382269859314   | bad for hardware

# Different sample rates (100 iteration mean, mode 2)
# 360   : 0.0660561227798462    | bad quality (unusable)
# 720   : 0.11093573808670044   |
# 1080  : 0.15820626258850098   |
#
# >>> sample rate ~ scan time

def calculate_mean_angle(angles):
    # Euler's formula: e^(iθ)=cos(θ)+i*sin(θ)

    # convert angles to unit vectors in the complex plane using NumPy's vectorized operations
    complex_numbers = np.exp(1j * np.array(angles))
    
    # compute the mean complex number
    mean_complex = np.mean(complex_numbers)
    
    # retrieve the angle of the mean complex number
    mean_angle = np.angle(mean_complex)
    
    return mean_angle

def calculate_distance(primary, secondary):
    angle1, distance1 = primary  # primary point (angle, distance)
    angle2, distance2 = secondary  # secondary point (angle, distance)
    
    # Convert the primary and secondary points to complex numbers
    primary_complex = distance1 * np.exp(1j * angle1)
    secondary_complex = distance2 * np.exp(1j * angle2)
    
    # Calculate the distance between the primary and secondary points
    distance = np.abs(primary_complex - secondary_complex)
    
    return distance

class LidarScanner:
    # scanning constants
    MAX_DISTANCE_METERS = 5
    SAMPLE_RATE = 720
    SCAN_MODE = 2
    MOTOR_SPEED = 600

    # acquisition constants
    ACQUISITION_DISTANCE = 0.5
    ACQUISITION_RADIUS = 0.2

    # DBSCAN constants
    DBSCAN_EPS = 0.1
    DBSCAN_MIN_SAMPLES = 8 # 8

    # tracking constants
    MAX_TRACK_DEVIATION = 0.3
    MAX_TRACK_LIFETIME = 2.0

    def __init__(self):
        # lidar and plotting setup
        self.setup_lidar()
        self.setup_plot()
        
        # locks for threading
        self.cluster_lock = threading.Lock()

        # class variables
        self.coordinates = []
        self.tracking = False
        self.tracked_point = None
        self.clusters = {}
        self.last_track = time.time()

        self.long_delta_time = {
            "main" : [],
            "scanning" : [],
            "clustering" : []
        }

        self.SAMPLE_RATE = int(round(self.SAMPLE_RATE))

    def exit_handler(self):
        self.lidar.set_motor_pwm(0)
        self.lidar.disconnect()

        print(f"MEAN DELTA TIME OVER 100 ITERATIONS:\nMAIN: {np.mean(np.array(self.long_delta_time['main']))}\nSCANNING: {np.mean(np.array(self.long_delta_time['scanning']))}\nCLUSTERING: {np.mean(np.array(self.long_delta_time['clustering']))}")

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
        self.lidar.set_motor_pwm(self.MOTOR_SPEED)
        time.sleep(2)

        # setting up scan handler
        self.handler = self.lidar.start_scan_express(self.SCAN_MODE)

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
            start_time = time.time()

            # get LiDAR data from scan
            start_time_scanning = time.time()
            coordinates = self.perform_scan()
            self.long_delta_time["scanning"].append(time.time() - start_time_scanning)

            # restart the loop when distance and angle are empty
            if not coordinates.any():
                with self.cluster_lock:
                    self.clusters.clear()

                continue
            
            start_time_clustering = time.time()
            clusters = self.perform_clustering(coordinates)
            self.long_delta_time["clustering"].append(time.time() - start_time_clustering)

            if self.tracking == True:
                self.perform_tracking(clusters)   
            else:
                self.acquire_track(clusters)

            self.long_delta_time["main"].append(time.time() - start_time)

            if len(self.long_delta_time["main"]) > 100:
                self.long_delta_time["main"].pop(-1)
                self.long_delta_time["scanning"].pop(-1)
                self.long_delta_time["clustering"].pop(-1)

            # draw distance and angle for next cycle
            with self.cluster_lock:
                self.clusters = clusters

    def perform_clustering(self, coordinates):
        # DBSCAN clustering

        # converting polar to euclidean coordinates
        transformed_coordinates = np.array([(np.sin(a), np.cos(a), d) for a, d in coordinates])

        # euclidean metric for distance calculation
        dbscan = DBSCAN(eps=self.DBSCAN_EPS, min_samples=self.DBSCAN_MIN_SAMPLES, metric='euclidean').fit(transformed_coordinates)
        labels = dbscan.labels_

        # Use defaultdict to automatically initialize clusters
        clusters = defaultdict(lambda: {
            'points': [],
            'total_distance': 0,
            'count': 0,
            'central_position': (0, 0)
        })
        
        for i, label in enumerate(labels):
            clusters[label]['points'].append(coordinates[i])
            
            # Update the running totals for distance
            clusters[label]['total_distance'] += coordinates[i][1]
            clusters[label]['count'] += 1
            
        # After the loop, compute the central angle for each cluster using the above method
        for label, cluster_data in clusters.items():
            all_angles = [point[0] for point in cluster_data['points']]
            mean_angle = calculate_mean_angle(all_angles)
            mean_distance = cluster_data['total_distance'] / cluster_data['count']
            cluster_data['central_position'] = (mean_angle, mean_distance)

        return clusters

    def acquire_track(self, clusters):
        
        for label, cluster_data in clusters.items():
            if label == -1:
                continue

            distance = calculate_distance(cluster_data['central_position'], (0, self.ACQUISITION_DISTANCE))

            if distance < self.ACQUISITION_RADIUS:
                self.tracked_point = cluster_data['central_position']
                self.tracking = True
                self.last_track = time.time()
                break

    def perform_tracking(self, clusters):
        new_track = False

        for label, cluster_data in clusters.items():
            if label == -1:
                continue

            distance = calculate_distance(cluster_data['central_position'], (self.tracked_point))

            if distance < self.MAX_TRACK_DEVIATION:
                self.tracked_point = cluster_data['central_position']
                self.last_track = time.time()
                new_track = True
                break
        
        if new_track == False and (self.last_track + self.MAX_TRACK_LIFETIME) < time.time():
            self.tracking = False
            self.tracked_point = (0, self.ACQUISITION_DISTANCE)            

    def perform_scan(self):
        # Pre-allocating arrays
        distance = np.zeros(self.SAMPLE_RATE)
        angle = np.zeros(self.SAMPLE_RATE)

        # engaging with the handler is the most performance intensive step here!
        for count, scan in enumerate(self.handler()):
            # adding scans below the max range into
            if scan.distance > 0 and (scan.distance / 1000) < self.MAX_DISTANCE_METERS:
                distance[count] = scan.distance / 1000
                angle[count] = np.deg2rad(scan.angle)

            # breaking loop once sample rate is achieved
            if count == (self.SAMPLE_RATE - 1):
                # Filter out zeros
                valid_indices = np.nonzero(distance)
                return np.array(list(zip(angle[valid_indices], distance[valid_indices])))
     
    def graphing(self):
        with self.cluster_lock:
            # clearing axis and scattering new data
            self.axis.clear()
            
            # Visualizing clusters
            for label, cluster_data in self.clusters.items():
                # don't render noise
                if label == -1:
                    continue

                cluster_angles, cluster_distances = zip(*cluster_data['points'])
                self.axis.scatter(cluster_angles, cluster_distances, s=40, marker='o')
                
                # Visualizing central position
                central_angle, central_distance = cluster_data['central_position']
                self.axis.scatter(central_angle, central_distance, s=60, c="#FFD700", marker='*')
            
            # Drawing acquisition circle
            if not self.tracking:
                circle = plt.Circle((self.ACQUISITION_DISTANCE, 0.0), self.ACQUISITION_RADIUS, transform=self.axis.transData._b, color="yellow", fill=False)
                self.axis.add_artist(circle)
            
            # Visualizing tracked point
            else:
                self.axis.scatter([self.tracked_point[0]], [self.tracked_point[1]], s=1200, c="#00FF00", marker='x')

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
