import numpy as np
import filterpy.kalman as KF

class KalmanFilter:
    def __init__(self, acquisition_distance) -> None:
        # Setup Kalman filter
        self.kalman_filter = KF.KalmanFilter(dim_x=4, dim_z=2)
        
        # Initial State [x_position, x_velocity, y_position, y_velocity]
        self.kalman_filter.x = np.array([acquisition_distance, 0., 0., 0.])
        
        # State Transition Matrix
        self.kalman_filter.F = np.array([
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1]
        ])

        # Measurement Matrix
        self.kalman_filter.H = np.array([
            [1, 0, 0, 0],  # x position
            [0, 0, 1, 0]   # y position
        ])
        
        # Initial Uncertainty
        self.kalman_filter.P *= 1.0
        
        # Process Uncertainty
        self.kalman_filter.Q = np.eye(4) * 1
        
        # Measurement Uncertainty
        self.kalman_filter.R = np.eye(2) * 0.1

        # Setup cluster covariance matrix
        self.cluster_covariance = np.eye(2)

    def predict(self):
        self.kalman_filter.predict()
    
    def get_current_prediction(self):
        return np.array([
            self.kalman_filter.x[0],  # x position
            self.kalman_filter.x[2]   # y position
        ])

    def get_cluster_covariance(self):
        return self.cluster_covariance
    
    def get_filter_covariance(self):
        return self.kalman_filter.P[np.ix_([0, 2], [0, 2])]

    def update(self, cluster):
        # Update Kalman filter
        self.kalman_filter.update(cluster['mean_vector'])

        # Update cluster covariance
        self.cluster_covariance = cluster['covariance']
