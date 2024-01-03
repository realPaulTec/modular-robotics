from collections import defaultdict
from matplotlib import patches
import pyrplidar
import time
import atexit
import threading
import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN
from filterpy.kalman import KalmanFilter

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

def mahalanobis_distance(x, μ, Σ):
    # D² = (x - μ)' Σ⁻¹ (x - μ)
    # 
    # D: Mahalanobis distance
    # x: object state
    # μ: predicted state
    # Σ: covariance matrix

    delta = np.array(x) - np.array(μ)

    inv_Σ = np.linalg.inv(Σ)

    return np.sqrt(np.dot(np.dot(delta, inv_Σ), delta.T))

def cartesian(theta, rho):
    return rho * np.cos(theta), rho * np.sin(theta)

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
    DBSCAN_EPS = 0.15
    DBSCAN_MIN_SAMPLES = 8

    # tracking constants
    MAX_TRACK_DEVIATION = 1.0
    MAX_TRACK_LIFETIME = 2.0
    MAX_TRACK_RUNAWAY = 0.4

    def __init__(self):
        # lidar and plotting setup
        self.setup_lidar()
        self.setup_plot()
        self.setup_kalman()
        
        # locks for threading
        self.cluster_lock = threading.Lock()

        # UI variables
        self.show_hits = True

        # class variables
        self.coordinates = []
        self.tracking = False
        self.tracked_point = []
        self.prediction = []
        self.clusters = {}
        self.last_track = time.time()

        self.SAMPLE_RATE = int(round(self.SAMPLE_RATE))

    def exit_handler(self):
        # Stop and disconnect LiDAR on program termination
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

    def setup_kalman(self):
        self.kalman_filter = KalmanFilter(dim_x=4, dim_z=2)
        
        # Initial State [x_position, x_velocity, y_position, y_velocity]
        self.kalman_filter.x = np.array([self.ACQUISITION_DISTANCE, 0., 0., 0.])
        
        # State Transition Matrix
        self.kalman_filter.F = np.array([[1, 1, 0, 0],
                                         [0, 1, 0, 0],
                                         [0, 0, 1, 1],
                                         [0, 0, 0, 1]])
        
        # Measurement Matrix
        self.kalman_filter.H = np.array([[1, 0, 0, 0],
                                         [0, 0, 1, 0]])
        
        # Initial Uncertainty
        self.kalman_filter.P *= 1.0
        
        # Process Uncertainty
        self.kalman_filter.Q = np.array([[1, 1, 0, 0],
                                         [1, 1, 0, 0],
                                         [0, 0, 1, 1],
                                         [0, 0, 1, 1]]) * 0.01
        
        # Measurement Uncertainty
        self.kalman_filter.R = np.array([[1, 0],
                                         [0, 1]]) * 0.5

    def continuous_tracking(self):
        while True:
            start_time = time.time()

            # get LiDAR data from scan
            start_time_scanning = time.time()
            coordinates = self.perform_scan()

            # restart the loop when distance and angle are empty
            if not coordinates.any():
                with self.cluster_lock:
                    self.clusters.clear()

                continue
            
            clusters = self.perform_clustering(coordinates)

            if self.tracking == True:
                self.perform_tracking(clusters)   
            else:
                self.acquire_track(clusters)

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
            'central_position': (0, 0),
            'mahalanobis_distance': 0
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
                self.kalman_filter.update(np.array([self.tracked_point[1] * np.cos(self.tracked_point[0]), self.tracked_point[1] * np.sin(self.tracked_point[0])]))
                self.tracking = True
                self.last_track = time.time()
                break

    def perform_tracking(self, clusters):
        # make a prediction for the next position
        self.kalman_filter.predict()
        current_prediction = np.array([self.kalman_filter.x[0], self.kalman_filter.x[2]])
        
        # extract position parts of the covariance matrix Σ
        covariance_matrix = self.kalman_filter.P[np.ix_([0, 2], [0, 2])]
                
        for label, cluster_data in clusters.items():
            # skip noise
            if label == -1:
                continue

            # set Mahalanobis distance for each cluster | FIXME: cluster_data['central_position'] are POLAR that's WRONG. 
            cluster_data['mahalanobis_distance'] = np.round(
                mahalanobis_distance(cartesian(*cluster_data['central_position']), current_prediction, covariance_matrix),
                decimals=3
                )
        
        # converting prediction to polar coordinates
        current_prediction_polar = np.array([np.arctan2(current_prediction[1], current_prediction[0]), np.sqrt(current_prediction[0]**2 + current_prediction[1]**2)])

        # filter the keys by distance threshold FIXME
        filtered_keys = [k for k in clusters.keys() if k != -1 and 
                         calculate_distance(clusters[k]['central_position'], self.tracked_point) < self.MAX_TRACK_RUNAWAY and
                         calculate_distance(clusters[k]['central_position'], current_prediction_polar) < self.MAX_TRACK_DEVIATION
                         ]
        
        # find cluster with lowest Mahalanobis distance
        closest_cluster_label = min(filtered_keys, key=lambda k: clusters[k]['mahalanobis_distance'], default=None)

        if closest_cluster_label:
            # set tracked position
            self.tracked_point = clusters[closest_cluster_label]['central_position']
            self.last_track = time.time()

            # update Kalman filter
            self.kalman_filter.update(np.array([self.tracked_point[1] * np.cos(self.tracked_point[0]), self.tracked_point[1] * np.sin(self.tracked_point[0])]))
        
        elif (self.last_track + self.MAX_TRACK_LIFETIME) < time.time():
            # reset lost track after lifetime exceeded
            self.tracked_point = []
            self.tracking = False
        
        # make a prediction for the next position, this is the end point of the red arrow
        self.kalman_filter.predict()
        self.prediction = self.kalman_filter.x[0], self.kalman_filter.x[2]

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
            
            # visualizing clusters
            for label, cluster_data in self.clusters.items():
                # skip noise
                if label == -1:
                    continue
                
                alpha = 1

                if len(self.tracked_point) > 0 and calculate_distance(self.tracked_point, cluster_data['central_position']) > self.MAX_TRACK_DEVIATION:
                    alpha = 0.15

                if self.show_hits == True:
                    cluster_angles, cluster_distances = zip(*cluster_data['points'])
                    self.axis.scatter(cluster_angles, cluster_distances, s=40, marker='o', alpha=alpha)
                
                # visualizing central position
                central_angle, central_distance = cluster_data['central_position']
                self.axis.scatter(central_angle, central_distance, s=60, c="#FFD700", marker='*')

                # annotating Mahalanobis distance for each cluster
                self.axis.annotate(cluster_data['mahalanobis_distance'], cluster_data['central_position'], color='white')
            
            # visualizing tracked point
            if len(self.tracked_point) > 0:
                self.axis.scatter([self.tracked_point[0]], [self.tracked_point[1]], s=1500, c="#00FF00", marker='x')

            # drawing acquisition circle
            else:
                circle = plt.Circle((self.ACQUISITION_DISTANCE, 0.0), self.ACQUISITION_RADIUS, transform=self.axis.transData._b, color="yellow", fill=False)
                self.axis.add_artist(circle)

            # drawing vector from track to prediction
            if len(self.prediction) > 0 and len(self.tracked_point) > 0:
                x_track, y_track = self.tracked_point[1] * np.cos(self.tracked_point[0]), self.tracked_point[1] * np.sin(self.tracked_point[0])             
                arrow = plt.arrow(x_track, y_track, (self.prediction[0]-x_track), (self.prediction[1]-y_track), width=0.002, length_includes_head=True, transform=self.axis.transData._b, color='red')
                self.axis.add_artist(arrow)

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
