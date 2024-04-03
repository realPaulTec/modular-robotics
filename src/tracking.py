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
import warnings
from scipy.stats import wasserstein_distance
warnings.filterwarnings('ignore')

class Tracking:
    # scanning constants
    MAX_DISTANCE_METERS     = 1.5
    SAMPLE_RATE             = 882 #441 #662

    # acquisition constants
    ACQUISITION_DISTANCE    = 0.5
    ACQUISITION_ANGLE       = 0
    ACQUISITION_RADIUS      = 0.2

    # DBSCAN constants
    DBSCAN_EPS              = 0.075
    DBSCAN_MIN_SAMPLES      = 7

    # tracking constants
    MAX_TRACK_DEVIATION     = 0.5
    MAX_TRACK_LIFETIME      = 1.0
    MAX_TRACK_RUNAWAY       = 0.8
    MAX_CLUSTER_LENGTH      = 2

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
        self.tracked_point = ()
        self.clusters = {}
        
        # Historical tracking
        self.previous_target    = {}
        self.previous_clusters  = {}
        self.last_track = time.time()

        # Class variables
        self.send_data = Event()
        self.kalman_accuracy = 0
        self.heading = 0

    def track_cycle(self, heading=0):
        # Send data to user interface
        self.send_data.set()

        # Get LiDAR data from scan
        coordinates = self.lidar.fetch_scan_data()

        # Restart loop if there is no data or it is the first iterations
        if not coordinates.any(): self.clusters.clear(); return
        
        # Offset coordinates
        coordinates = self.offset_coordinates(coordinates, angle=heading)

        # Perform DBSCAN clustering and returning labels
        labels = self.clustering(coordinates)

        # Stop sending data
        self.send_data.clear()

        # Process cluster labels to cluster dictionary
        clusters = self.process_clusters(labels, coordinates)

        # Compute pre-tracking properties of clusters in single loop
        clusters = self.compute_properties(clusters)

        # Pass clusters and heading to user interface
        self.clusters = clusters
        self.heading = heading

        # Acquire object if not tracking or return on override
        if      self.override == True   :  self.reset_tracking()                        ; return
        elif    self.tracking == False  :  self.acquisition(clusters, heading=heading)  ; return
        
        # Make a Kalman filter prediction for the next position
        self.kalman_filter.predict()

        # Get current prediction from Kalman filter
        current_prediction = self.kalman_filter.get_current_prediction()

        # Extracting the Kalman filters error covariance matrix Σ
        covariance_matrix = self.kalman_filter.get_filter_covariance()  

        # Converting prediction to polar coordinates
        current_prediction_polar = utils.cartesian_to_polar(*current_prediction)
        
        # Filter keys for distance and length of the clusters
        filtered_keys = self.filter_keys(clusters, current_prediction_polar)
        
        # Set trackable property
        for key in list(clusters.keys()):
            if key not in filtered_keys:    clusters[key]['trackable'] = False    

        # Compute composite distance metric for each cluster
        clusters = self.compute_distance_metric(clusters, current_prediction, covariance_matrix)

        # Select the cluster with the lowest distance metric
        current_target = min(filtered_keys, key=lambda k: clusters[k]["composite_distance"], default=None)

        # Set previous clusters
        self.previous_clusters = clusters

        # TODO: Implement Bayes-Filter
        if current_target:
            # Set previous target
            self.previous_target = clusters[current_target]
            
            # Set tracked point
            self.tracked_point = clusters[current_target]['central_position']

            # Set tracked property
            clusters[current_target]['tracked'] = True
            
            # Set time since last track
            self.last_track = time.time()

            # Update Kalman filter
            self.kalman_filter.update(clusters[current_target])

        # Reset tracking if MAX_TRACK_LIFETIME has been exceeded
        elif (self.last_track + self.MAX_TRACK_LIFETIME) < time.time():
            self.reset_tracking()

        # Pass prediction to user interface for drawing arrow
        self.kalman_filter.predict()
        self.prediction = self.kalman_filter.get_current_prediction()

    def offset_coordinates(self, coordinates, angle=0):
        # Offset cluster coordinates
        coordinates = utils.offset_polar_coordinates(coordinates, 0, (180 + angle) % 360)
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
            'length'                    : 0,
            'composite_distance'        : 0,
            'covariance'                : [[0, 0], [0, 0]],
            'prev_composite_distance'   : 0,
            'tracked'                   : False,   

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

        # Remove the noise
        if -1 in clusters: del clusters[-1]

        return clusters
       
    def acquisition(self, clusters, heading=0):
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1: continue
            
            # Checking if any cluster is within the acquisition circle
            distance = utils.distance_polar(cluster_data['central_position'], (self.ACQUISITION_DISTANCE, np.deg2rad(-self.heading)))
            
            if distance < self.ACQUISITION_RADIUS:
                # Setting tracked point to acquired cluster
                self.tracked_point = cluster_data['central_position']

                # Update Kalman filter
                self.kalman_filter.update(cluster_data)
                
                # Updating tracking status and last track time
                self.tracking = True
                self.last_track = time.time()
                
                # Set previous target
                self.previous_target = clusters[label]

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

    def compute_distance_metric(self, clusters, current_prediction, filter_covariance):
        # Identify previous covariance
        previous_covariance = self.kalman_filter.get_cluster_covariance()
        
        t1 = time.time()
        for label, cluster_data in clusters.items():
            # Skip noise
            if label == -1 or not cluster_data['trackable']: continue

            # Calculate distance metric
            cluster_data['composite_distance'] = utils.general_wasserstein_distance(
                np.array(self.previous_target['points_cartesian'] + (current_prediction - self.previous_target['mean_vector'])),
                np.array(cluster_data['points_cartesian'])
            )

            # cluster_data['composite_distance'] = utils.wasserstein_distance(
            #                 current_prediction,
            #                 previous_covariance,
            #                 cluster_data['mean_vector'],
            #                 cluster_data['covariance']
            #             )

        print(time.time() - t1)

        return clusters

    def filter_keys(self, clusters, current_prediction_polar):
        # # Filter the keys by distance thresholds
        primary_filtered_keys = [k for k in clusters.keys() if k != -1 and 
                        utils.distance_polar(clusters[k]['central_position'], self.tracked_point) < self.MAX_TRACK_RUNAWAY and
                        utils.distance_polar(clusters[k]['central_position'], current_prediction_polar) < self.MAX_TRACK_DEVIATION and
                        clusters[k]['length'] < self.MAX_CLUSTER_LENGTH
                        ]
        
        # Set the key amount
        if len(primary_filtered_keys) > 2   : c_MAX_TRACK_DEVIATION = 0.2
        else                                : c_MAX_TRACK_DEVIATION = 0.4

        # Filter the keys by distance thresholds
        filtered_keys = [k for k in clusters.keys() if k != -1 and 
                        utils.distance_polar(clusters[k]['central_position'], self.tracked_point) < self.MAX_TRACK_RUNAWAY and
                        utils.distance_polar(clusters[k]['central_position'], current_prediction_polar) < c_MAX_TRACK_DEVIATION and
                        clusters[k]['length'] < self.MAX_CLUSTER_LENGTH
                        ]

        t1 = time.time()
        for label, cluster_data in clusters.items():
            previous_composite_distances = []            
            for previous_label, previous_cluster_data in self.previous_clusters.items():
                previous_composite_distances.append((
                        previous_label,
                        utils.wasserstein_distance(
                            cluster_data['mean_vector'],
                            cluster_data['covariance'],
                            previous_cluster_data['mean_vector'],
                            previous_cluster_data['covariance']
                        )
                    )
                )

            # Check if the lowest distance metric is to a 
            # lowest_distance_entry = sorted(previous_composite_distances, key=lambda x: x[1])[0]
            # cluster_data['prev_composite_distance'] = lowest_distance_entry[1]

        print(time.time()-t1)

        return filtered_keys

    def reset_tracking(self):
        # Reset lost track after lifetime exceeded
        self.tracked_point = ()
        
        # Reset tracking and override
        self.tracking = False
        self.override = True

        # Reset Kalman filter
        self.kalman_filter = KalmanFilter(self.ACQUISITION_DISTANCE)

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
            try                     : stream.send_data(tracking_data)
            except Exception as e   : pass

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
