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
from threading import Event

class Tracking:
    # scanning constants
    MAX_DISTANCE_METERS = 2.5
    SAMPLE_RATE = 441 #441

    # acquisition constants
    ACQUISITION_DISTANCE = 0.5
    ACQUISITION_ANGLE = 0
    ACQUISITION_RADIUS = 0.2

    # DBSCAN constants
    DBSCAN_EPS = 0.1
    DBSCAN_MIN_SAMPLES = 4

    # tracking constants
    MAX_TRACK_DEVIATION = 0.4
    MAX_TRACK_LIFETIME = 1.0
    MAX_TRACK_RUNAWAY = 0.8

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
        self.hclusters = []
        self.last_track = time.time()
        self.last_distribution=np.array([None])
        self.hcoordinates=np.array([None])
        self.first=True
        self.send_data = Event()

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

    def track_cycle(self):
        # Send data to user interface
        self.send_data.set()

        # Get LiDAR data from scan
        coordinates = self.lidar.fetch_scan_data()

        # Restart the loop when distance and angle are empty
        if not coordinates.any():
            self.clusters.clear()
            return
        
        # Skip the first iteration
        if self.first == True:
            self.first = False
            return

        # Offset coordinates
        coordinates = self.offset_coordinates(coordinates)

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
            self.acquisition(clusters)
            return

        # Extracting the covariance matrix for the cluster
        cluster_covariance = np.array([
            [self.kalman_filter.x[4], self.kalman_filter.x[6]],
            [self.kalman_filter.x[6], self.kalman_filter.x[5]]
        ])

        # Get current prediction from Kalman filter
        current_prediction = np.array([
            self.kalman_filter.x[0],  # x position
            self.kalman_filter.x[2]   # y position
        ])

        # Extracting the Kalman filters error covariance matrix Σ
        covariance_matrix = self.kalman_filter.P[np.ix_([0, 2], [0, 2])]

        # Compute Mahalanobis distance for each cluster
        clusters = self.compute_bhattacharyya(clusters, current_prediction, cluster_covariance)
        # clusters = self.compute_mahalanobis(clusters, current_prediction, covariance_matrix)
        # clusters = self.compute_mb_metric(clusters)
        
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

    def offset_coordinates(self, coordinates):
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
            'bhattacharyya_distance': 0
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
            res = np.round(
                utils.bhattacharyya_distance(cluster_data['mean_vector'], cluster_data['covariance'] * 10, current_prediction, covariance_matrix * 10),
                decimals=3
                )

            cluster_data['bhattacharyya_distance'] = res

            if res == 0 and self.tracking == True:  print('ZERO VALUE')
        
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
                # itime = time.time()
                tracking.track_cycle()
                # print(f"dtime: {time.time() - itime}")
        
        # setting up separate daemon thread for scanning and tracking
        tracking_thread = threading.Thread(target=continuous_tracking) 
        tracking_thread.daemon = True
        tracking_thread.start()

        # Socket UI
        while True:
            try:
                # Sync data stream with tracking
                tracking.send_data.wait()

                # Prepare data to send
                tracking_data = stream.convert_for_sending(tracking)
                
                # Send data to desktop
                try:
                    stream.send_data(tracking_data)
                except Exception as e:
                    print(f'\nERROR {e}')

                # Clear send_data event
                tracking.send_data.clear()
                time.sleep(0.1)
            
            except KeyboardInterrupt:
                print('\nTerminating ...')
                sys.stderr = open(os.devnull, 'w')
                
                # Trigger exit handlers for GPIO and LiDAR
                tracking.lidar.exit_handler()
                
                # Exit program
                os._exit(0)
