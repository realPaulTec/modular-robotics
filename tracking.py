import os
import sys
import numpy as np
import time
from collections import defaultdict
import threading
from sklearn.cluster import DBSCAN
from filterpy.kalman import KalmanFilter
import utils
from lidar import Lidar
import stream

class Tracking:
    # scanning constants
    MAX_DISTANCE_METERS = 2.5
    SAMPLE_RATE = 720 * 2 #441

    # acquisition constants
    ACQUISITION_DISTANCE = 0.5
    ACQUISITION_ANGLE = 0
    ACQUISITION_RADIUS = 0.2

    # DBSCAN constants
    DBSCAN_EPS = 0.1
    DBSCAN_MIN_SAMPLES = 8

    # tracking constants
    MAX_TRACK_DEVIATION = 0.4 * 1.4
    MAX_TRACK_LIFETIME = 1.0
    MAX_TRACK_RUNAWAY = 1.5

    def __init__(self):
        # lidar and kalman setup
        self.lidar = Lidar(self.SAMPLE_RATE, self.MAX_DISTANCE_METERS)   
        self.setup_kalman()

        # Class variables
        self.coordinates = []
        self.tracking = False
        self.tracked_point = []
        self.prediction = []
        self.clusters = {}
        self.last_track = time.time()

    def setup_kalman(self):
        self.kalman_filter = KalmanFilter(dim_x=7, dim_z=5)
        
        # Initial State [x_position, x_velocity, y_position, y_velocity, var(x), var(y)]
        self.kalman_filter.x = np.array([self.ACQUISITION_DISTANCE, 0., 0., 0., 1., 1., 0.])
        
        # State Transition Matrix
        self.kalman_filter.F = np.array([
            [1, 1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0],
            [0, 0, 1, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1]
        ])

        # Measurement Matrix
        self.kalman_filter.H = np.array([
            [1, 0, 0, 0, 0, 0, 0],  # x position
            [0, 0, 1, 0, 0, 0, 0],  # y position
            [0, 0, 0, 0, 1, 0, 0],  # var(x)
            [0, 0, 0, 0, 0, 1, 0],  # var(y)
            [0, 0, 0, 0, 0, 0, 1]   # cov(x,y)
        ])
        
        # Initial Uncertainty
        self.kalman_filter.P *= 1.0
        
        # Process Uncertainty
        self.kalman_filter.Q = np.eye(7) * 0.01  # Adjusted for simplicity, but you should customize this
        
        # Measurement Uncertainty
        self.kalman_filter.R = np.eye(5) * 0.5

    def track_cycle(self, hall_thread=None, hall_queue=None, stop_feedback=None):
        # Get initial start time
        initial_time = time.time()

        # Get LiDAR data from scan
        start_time = time.time()
        coordinates = self.lidar.fetch_scan_data()
        # print(f'\nScan: {round(time.time() - start_time, 4)}')

        # Restart the loop when distance and angle are empty
        if not coordinates.any():
            self.clusters.clear()
            return

        # Offset coordinates for vehicle displacement
        start_time = time.time()
        coordinates = self.offset_coordinates(coordinates, hall_thread, hall_queue, stop_feedback)
        
        # Perform DBSCAN clustering and returning labels
        labels = self.clustering(coordinates)

        # Process cluster labels to cluster dictionary
        clusters = self.process_clusters(labels, coordinates)

        # Compute cluster centers uisng Euler's formula
        clusters = self.compute_cluster_centers(clusters)

        # Pass clusters to user interface
        self.clusters = clusters

        # Compute covariance matrices for clusters 
        clusters = self.compute_covariance(clusters)

        # Acquire object if not tracking
        if self.tracking == False:
            # print(f'Full: {round(time.time() - initial_time, 4)}')
            self.acquisition(clusters)
            return

        start_time = time.time()
        # Assuming self.kalman_filter.x[4] = var(x), self.kalman_filter.x[5] = var(y), self.kalman_filter.x[6] = cov(x,y)

        # Extracting the covariance matrix for the cluster
        cluster_covariance = np.array([
            [self.kalman_filter.x[4], self.kalman_filter.x[6]],
            [self.kalman_filter.x[6], self.kalman_filter.x[5]]
        ])

        current_prediction = np.array([
            self.kalman_filter.x[0],  # x position
            self.kalman_filter.x[2]   # y position
        ])

        # Extract the covariance matrix Σ for the position parts from the Kalman filter's error covariance matrix P
        covariance_matrix = self.kalman_filter.P[np.ix_([0, 2], [0, 2])]

        # Compute Mahalanobis distance for each cluster
        clusters = self.compute_bhattacharyya(clusters, current_prediction, cluster_covariance)
        clusters = self.compute_mahalanobis(clusters, current_prediction, covariance_matrix)
        clusters = self.compute_mb_metric(clusters)
        
        # Converting prediction to polar coordinates
        current_prediction_polar = utils.cartesian_to_polar(*current_prediction)

        # Filter the keys by distance thresholds
        filtered_keys = [k for k in clusters.keys() if k != -1 and 
                         utils.distance_polar(clusters[k]['central_position'], self.tracked_point) < self.MAX_TRACK_RUNAWAY and
                         utils.distance_polar(clusters[k]['central_position'], current_prediction_polar) < self.MAX_TRACK_DEVIATION
                         ]
        
        # Find cluster with lowest Mahalanobis distance
        metric = 'bhattacharyya_distance' # 'mb_metric' 
        closest_cluster_label = min(filtered_keys, key=lambda k: clusters[k][metric], default=None)
        
        if closest_cluster_label:
            # Set tracked position
            self.tracked_point = clusters[closest_cluster_label]['central_position']
            self.last_track = time.time()

            print(f"CL mean: {np.round(clusters[closest_cluster_label]['mean_vector'], 3)} covar: {np.round(clusters[closest_cluster_label]['covariance'], 3)}")
            print(f"KF mean: {np.round(current_prediction, 3)} covar: {np.round(covariance_matrix, 3)}")

            # Update Kalman filter
            self.kalman_filter.update(np.array([
                *clusters[closest_cluster_label]['mean_vector'],
                clusters[closest_cluster_label]['covariance'][0][0],
                clusters[closest_cluster_label]['covariance'][1][1],
                clusters[closest_cluster_label]['covariance'][0][1]
            ]))

        elif (self.last_track + self.MAX_TRACK_LIFETIME) < time.time():
            # Reset lost track after lifetime exceeded
            self.tracked_point = []
            self.tracking = False
        
        # Pass prediction to user interface for drawing arrow
        self.kalman_filter.predict()
        self.prediction = self.kalman_filter.x[0], self.kalman_filter.x[2]

    def offset_coordinates(self, coordinates, hall_thread, hall_queue, stop_feedback):
        # Offset coordinates and read HALL data from queue
        if hall_thread:
            # Stop HALL readings 
            stop_feedback.set()
            
            # Get information from HALL queue
            hall_thread.join()
            linear_displacement, angular_displacement = hall_queue.get()

            # Offset cluster coordinates
            print(f"\r\033[K {round(linear_displacement, 3)} | {round(angular_displacement)}", end="")
            coordinates = utils.offset_polar_coordinates(coordinates, -linear_displacement, 180) # TODO implement angular_displacement
        else:
            # Offset cluster coordinates
            coordinates = utils.offset_polar_coordinates(coordinates, 0, 180)

        return coordinates

    def clustering(self, coordinates):
        # DBSCAN clustering
        
        # Converting polar to euclidean coordinates
        transformed_coordinates = np.column_stack((np.sin(coordinates[:, 1]), np.cos(coordinates[:, 1]), coordinates[:, 0]))

        # Euclidean metric for distance calculation
        dbscan = DBSCAN(eps=self.DBSCAN_EPS, min_samples=self.DBSCAN_MIN_SAMPLES, metric='euclidean').fit(transformed_coordinates)
        return dbscan.labels_

    def process_clusters(self, labels, coordinates):
        # Initialize clusters using defaultdict 
        clusters = defaultdict(lambda: {
            'points': [],
            'points_cartesian' : [],
            'count': 0,
            'central_position': (0, 0),
            'mean_vector' : (0, 0),
            'covariance' : [[0, 0], [0, 0]],
            'mahalanobis_distance': 0,
            'bhattacharyya_distance': 0,
            'mb_metric' : 0
        })
        
        for i, label in enumerate(labels):
            # Add points to cluster dictionary
            clusters[label]['points'].append(coordinates[i])
            clusters[label]['points_cartesian'].append(utils.polar_to_cartesian(*coordinates[i]))
            
            # Updating running totals for distance
            clusters[label]['count'] += 1

        return clusters

    def compute_cluster_centers(self, clusters):    
        # Compute central angle for each cluster using Euler's Formula to avoid cluster-wraparound
        for label, cluster_data in clusters.items():
            points = np.array(cluster_data['points']) 
            
            # Calculate mean angle with Euler's formula
            mean_angle = utils.calculate_mean_angle(points[:, 1])

            # Compute the mean distance
            mean_distance = np.mean(points[:, 0])

            # Update the central position
            cluster_data['central_position'] = (mean_distance, mean_angle)

        return clusters
        
    def acquisition(self, clusters):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue
            
            # Checking if any cluster is within the acquisition circle
            distance = utils.distance_polar(cluster_data['central_position'], (self.ACQUISITION_DISTANCE, self.ACQUISITION_ANGLE))
            
            if distance < self.ACQUISITION_RADIUS:
                # Setting tracked point to acquired cluster
                self.tracked_point = cluster_data['central_position']

                # Update Kalman filter
                self.kalman_filter.update(np.array([
                    *cluster_data['mean_vector'],
                    cluster_data['covariance'][0][0],
                    cluster_data['covariance'][1][1],
                    cluster_data['covariance'][0][1]
                ]))
                
                # Updating tracking status and last track time
                self.tracking = True
                self.last_track = time.time()
                break

    def compute_covariance(self, clusters):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue

            # Set Mahalanobis distance for each cluster 
            cluster_data['mean_vector'], cluster_data['covariance'] = utils.mean_and_covariance(cluster_data['points_cartesian'])
        
        return clusters

    def compute_bhattacharyya(self, clusters, current_prediction, covariance_matrix):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue

            # Set Mahalanobis distance for each cluster 
            cluster_data['bhattacharyya_distance'] = np.round(
                utils.bhattacharyya_distance(cluster_data['mean_vector'], cluster_data['covariance'], current_prediction, covariance_matrix),
                decimals=3
                )
        
        return clusters

    def compute_mahalanobis(self, clusters, current_prediction, covariance_matrix):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue
            
            # Set Mahalanobis distance for each cluster 
            cluster_data['mahalanobis_distance'] = np.round(
                utils.mahalanobis_distance(cluster_data['mean_vector'], current_prediction, covariance_matrix),
                decimals=3
                )
            # Set MB metru distance for each cluster 
            
        return clusters

    def compute_mb_metric(self, clusters):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue

            # Set MB metric
            cluster_data['mb_metric'] = cluster_data['mahalanobis_distance'] * cluster_data['bhattacharyya_distance']
        
        return clusters

if __name__ == "__main__":
        # Generating new tracking class
        tracking = Tracking()

        # Continuous tracking loop
        def continuous_tracking():
            while True:
                tracking.track_cycle()
        
        # setting up separate daemon thread for scanning and tracking
        tracking_thread = threading.Thread(target=continuous_tracking) 
        tracking_thread.daemon = True
        tracking_thread.start()

        # Socket UI
        while True:
            try:
                # Prepare data to send
                tracking_data = stream.convert_for_sending(tracking)
                
                # Send data to desktop
                try:
                    stream.send_data(tracking_data)
                except Exception as e:
                    print(e)

                # Sleep to avoid overwhealming network
                time.sleep(1)
            
            except KeyboardInterrupt:
                print('\nTerminating ...')
                sys.stderr = open(os.devnull, 'w')
                
                # Trigger exit handlers for GPIO and LiDAR
                tracking.lidar.exit_handler()
                
                # Exit program
                os._exit(0)
