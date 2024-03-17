import numpy as np
import filterpy.kalman as KF

class KalmanFilter:
    def __init__(self, acquisition_distance) -> None:
        self.kalman_filter = KF.KalmanFilter(dim_x=7, dim_z=5)
        
        # Initial State [x_position, x_velocity, y_position, y_velocity, var(x), var(y), cov(xy)]
        self.kalman_filter.x = np.array([acquisition_distance, 0., 0., 0., 1., 1., 0.])
        
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
        self.kalman_filter.Q = np.eye(7) * 0.01
        
        # Measurement Uncertainty
        self.kalman_filter.R = np.eye(5) * 0.5

    def predict(self):
        self.kalman_filter.predict()
    
    def get_current_prediction(self):
        return np.array([
            self.kalman_filter.x[0],  # x position
            self.kalman_filter.x[2]   # y position
        ])

    def get_cluster_covariance(self):
        return np.array([
            [self.kalman_filter.x[4], self.kalman_filter.x[6]],
            [self.kalman_filter.x[6], self.kalman_filter.x[5]]
        ])
    
    def get_filter_covariance(self):
        return self.kalman_filter.P[np.ix_([0, 2], [0, 2])]

    def update(self, cluster):
        # Update Kalman filter
        self.kalman_filter.update(np.array([
            *cluster['mean_vector'],
            cluster['covariance'][0][0],
            cluster['covariance'][1][1],
            cluster['covariance'][0][1]
        ]))
