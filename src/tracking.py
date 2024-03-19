from collections import defaultdict
import threading
from sklearn.cluster import DBSCAN
import utils
from lidar import Lidar
import stream
from threading import Event
from kalman_filter import KalmanFilter
import os
import sys
import numpy as np
import time

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
    DBSCAN_MIN_SAMPLES = 5

    # tracking constants
    MAX_TRACK_DEVIATION = 0.4
    MAX_TRACK_LIFETIME = 1.0
    MAX_TRACK_RUNAWAY = 0.6
    MAX_CLUSTER_LENGTH = 0.4

    def __init__(self):
        # lidar and kalman setup
        self.lidar = Lidar(self.SAMPLE_RATE, self.MAX_DISTANCE_METERS)   

        # Kalman filter
        self.kalman_filter = KalmanFilter(self.ACQUISITION_DISTANCE)

        # User interface
        self.coordinates = []
        self.prediction = []

        # Current tracking
        self.tracking = False
        self.override = False
        self.tracked_point = []
        self.clusters = {}
        
        # Historical tracking
        self.hclosest_cluster_label = 0
        self.hclusters = {}
        self.last_track = time.time()

        # Class variables
        self.first_track=True
        self.send_data = Event()

    def track_cycle(self):
        # Send data to user interface
        self.send_data.set()

        # Get LiDAR data from scan
        coordinates = self.lidar.fetch_scan_data()

        # Restart loop if there is no data or it is the first iterations
        if not coordinates.any(): self.clusters.clear(); return
        
        # Offset coordinates
        coordinates = self.offset_coordinates(coordinates)

        # Perform DBSCAN clustering and returning labels
        labels = self.clustering(coordinates)

        # Stop sending data
        self.send_data.clear()

        # Process cluster labels to cluster dictionary
        clusters = self.process_clusters(labels, coordinates)

        # Compute pre-tracking properties of clusters in single loop
        clusters = self.compute_properties(clusters)

        # Pass clusters to user interface
        self.clusters = clusters

        # Acquire object if not tracking or return on override
        if      self.override == True   :  self.reset_tracking(); return
        elif    self.tracking == False  :  self.acquisition(clusters); return
        
        # Make a Kalman filter prediction for the next position
        self.kalman_filter.predict()

        # Extracting the covariance matrix for the cluster
        cluster_covariance = self.kalman_filter.get_cluster_covariance()

        # Get current prediction from Kalman filter
        current_prediction = self.kalman_filter.get_current_prediction()

        # TODO: Reimplement Extracting the Kalman filters error covariance matrix Σ
        covariance_matrix = self.kalman_filter.get_filter_covariance()

        # Compute Mahalanobis distance for each cluster
        clusters = self.compute_bhattacharyya(clusters, current_prediction, cluster_covariance)
        
        # Converting prediction to polar coordinates
        current_prediction_polar = utils.cartesian_to_polar(*current_prediction)

        # Filter keys for distance and length of the clusters
        filtered_keys = self.filter_keys(clusters, current_prediction_polar)

        # Find cluster with lowest Bhattacharyya / Mahalanobis distance
        metric = 'bhattacharyya_distance'
        closest_cluster_label = min(filtered_keys, key=lambda k: clusters[k][metric], default=None)
        
        # Set trackable property
        for key in list(clusters.keys()):
            if key not in filtered_keys:    clusters[key]['trackable'] = False        

        # TODO: Implement historical cross-check
        # NOTE: 
        # I could implement simplified tracking for non-target objects emitting the kalman filter, only using closest-Bhattacharyya
        # If the closest Bhattacharyya from an old object which was NOT the target in the last cycle, now is the target that indicates a failiure 
        
        if closest_cluster_label: #and self.historical_crosscheck(clusters, closest_cluster_label):
            # print(clusters[closest_cluster_label]['bhattacharyya_distance'])

            # Set current clusters to historic
            self.hclosest_cluster_label = closest_cluster_label
            self.hclusters = clusters

            # Reset first track
            self.first_track = False

            # Set tracked position
            self.tracked_point = clusters[closest_cluster_label]['central_position']
            self.last_track = time.time()

            # Update Kalman filter
            self.kalman_filter.update(clusters[closest_cluster_label])
        
        elif not self.historical_crosscheck(clusters, closest_cluster_label):
            # Reset tracking
            self.reset_tracking()

            print('FAILIURE')

        elif (self.last_track + self.MAX_TRACK_LIFETIME) < time.time():
            # Reset lost track after lifetime exceeded
            self.reset_tracking()

        # Pass prediction to user interface for drawing arrow
        self.kalman_filter.predict()
        self.prediction = self.kalman_filter.get_current_prediction()

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
            # Cartesian variables for tracking
            'points_cartesian'          : [],
            'mean_vector'               : (0, 0),
            'area'                      : 0,
            'mahalanobis_distance'      : 0,
            'bhattacharyya_distance'    : 0,
            'covariance'                : [[0, 0], [0, 0]],

            # Polar variables for plotting & control
            'points'                    : [],
            'count'                     : 0,
            'central_position'          : (0, 0),
            'trackable'                 : True
        })
        
        for i, label in enumerate(labels):
            # Add points to cluster dictionary
            clusters[label]['points'].append(coordinates[i])
            clusters[label]['points_cartesian'].append(utils.polar_to_cartesian(*coordinates[i]))
            
            # Updating running totals for distance
            clusters[label]['count'] += 1

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
                self.kalman_filter.update(cluster_data)
                
                # Updating tracking status and last track time
                self.tracking = True
                self.last_track = time.time()
                
                # Set current clusters to historic
                self.hclosest_cluster_label = label
                self.hclusters = clusters
                break

    def compute_properties(self, clusters):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue

            # Compute polar centers of each cluster
            cluster_data['central_position'] = utils.calculate_polar_center(np.array(cluster_data['points']))

            # Set mean vector and covariance of each cluster 
            cluster_data['mean_vector'], cluster_data['covariance'] = utils.mean_and_covariance(np.array(cluster_data['points_cartesian']))

            # Calculate the length of each cluster
            cluster_data['length'] = utils.calculate_cluster_length(cluster_data['points_cartesian'])

        return clusters

    def compute_bhattacharyya(self, clusters, current_prediction, covariance_matrix):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue

            # Set Bhattacharyya distance for each cluster
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
            
        return clusters

    def filter_keys(self, clusters, current_prediction_polar):
        # Filter the keys by distance thresholds
        filtered_keys = [k for k in clusters.keys() if k != -1 and 
                         utils.distance_polar(clusters[k]['central_position'], self.tracked_point) < self.MAX_TRACK_RUNAWAY and
                         utils.distance_polar(clusters[k]['central_position'], current_prediction_polar) < self.MAX_TRACK_DEVIATION and
                         clusters[k]['length'] < self.MAX_CLUSTER_LENGTH
                         ]
        
        return filtered_keys

    def historical_crosscheck(self, clusters, closest_cluster_label):
        # Return true if there is no closest_cluster_label 
        if not closest_cluster_label:   return True

        # Compute Bhattacharyya distances from historical clusters to target
        hclusters = self.compute_bhattacharyya(self.hclusters, clusters[closest_cluster_label]['mean_vector'], clusters[closest_cluster_label]['covariance'])

        # Filter keys
        filter_keys = self.filter_keys(hclusters, clusters[closest_cluster_label]['central_position'])

        # Find cluster with lowest Bhattacharyya / Mahalanobis distance
        closest_hcluster_key = min(filter_keys, key=lambda k: hclusters[k]['bhattacharyya_distance'], default=None)
        
        # NOTE: There are usually two legs :/ But they do tend to have the same covariance...
        return True #closest_hcluster_key == self.hclosest_cluster_label or self.first_track

    def reset_tracking(self):
        # Reset lost track after lifetime exceeded
        self.tracked_point = []
        self.tracking = False
        self.first_track = True
        # self.kalman_filter = KalmanFilter(self.ACQUISITION_DISTANCE)

if __name__ == "__main__":
    # Generating new tracking class
    tracking = Tracking()

    # Continuous tracking loop
    def continuous_tracking():
        while True:
            itime = time.time()
            tracking.track_cycle()
            print(f"dtime: {time.time() - itime}")
    
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
                pass
                # print(f'\nERROR {e}')

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
