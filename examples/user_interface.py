import matplotlib.pyplot as plt
import numpy as np
import utils
import time

class UserInterface:
    # UI variables
    show_hits = True

    def __init__(self, MAX_DISTANCE_METERS, MAX_TRACK_DEVIATION, ACQUISITION_RADIUS, ACQUISITION_DISTANCE):
        # Tracking constants
        self.MAX_DISTANCE_METERS = MAX_DISTANCE_METERS
        self.MAX_TRACK_DEVIATION = MAX_TRACK_DEVIATION
        self.ACQUISITION_RADIUS = ACQUISITION_RADIUS
        self.ACQUISITION_DISTANCE = ACQUISITION_DISTANCE

        # Tracking variables
        self.tracking = False
        self.tracked_point = []
        self.prediction = []
        self.clusters = {}

        # Matplotlib styling for plot || font = {'family': 'URW Gothic', 'weight': 'bold', 'size': 14}
        font = {'weight': 'bold', 'size': 14}
        plt.rc('font', **font)
        self.fig = plt.figure(facecolor='black')

        # creating polar plot for visualization
        self.axis = self.fig.add_subplot(111, projection='polar')
        self.axis.set_facecolor('black')
        self.axis.tick_params(which='both', colors='#55ddffff')

        # Initialize window
        plt.show(block=False)

    def update_clusters(self, tracking, tracked_point, prediction, clusters):
        # Tracking variables
        self.tracking = tracking
        self.tracked_point = tracked_point
        self.prediction = prediction
        self.clusters = clusters

    def plot_clusters(self):
        for label, cluster_data in self.clusters.items():
            # Skip noise
            if label == -1:
                continue
            
            # Making clusters not currently considered transparent TODO
            alpha = 1
            if len(self.tracked_point) > 0 and utils.distance_polar(self.tracked_point, cluster_data['central_position']) > self.MAX_TRACK_DEVIATION:
                alpha = 0.15

            # Show individual points
            if self.show_hits == True:
                cluster_distances, cluster_angles  = zip(*cluster_data['points'])
                self.axis.scatter(cluster_angles, cluster_distances, s=40, marker='o', alpha=alpha)
           
            # Visualizing central position
            self.axis.scatter(*reversed(cluster_data['central_position']), s=60, c="#FFD700", marker='*')

            # Annotating Mahalanobis distance for each cluster
            # self.axis.annotate(cluster_data['mahalanobis_distance'], reversed(cluster_data['central_position']), color='white')

    def draw_tracked_point(self):
        # Visualizing tracked point
        if len(self.tracked_point) > 0:
            self.axis.scatter([self.tracked_point[1]], [self.tracked_point[0]], s=1500, c="#00FF00", marker='x')

        # Drawing acquisition circle
        else:
            circle = plt.Circle((self.ACQUISITION_DISTANCE, 0.0), self.ACQUISITION_RADIUS, transform=self.axis.transData._b, color="yellow", fill=False)
            self.axis.add_artist(circle)

    def draw_prediction_vector(self):
        # Drawing vector from track to prediction
        if len(self.prediction) > 0 and len(self.tracked_point) > 0:
            x_track, y_track = utils.polar_to_cartesian(*self.tracked_point)             
            arrow = plt.arrow(x_track, y_track, (self.prediction[0]-x_track), (self.prediction[1]-y_track), width=0.002, length_includes_head=True, transform=self.axis.transData._b, color='red')
            self.axis.add_artist(arrow)

    def graphing(self):
        # clearing axis and scattering new data
        self.axis.clear()
        
        # Plotting clusters
        self.plot_clusters()
        
        # Plotting tracked point and acquisition circle
        self.draw_tracked_point()

        # Drawing Kalman filter prediction vector
        self.draw_prediction_vector()

        # Scaling the axis to the max range
        self.axis.set_ybound(0, self.MAX_DISTANCE_METERS)

        # Labeling distance in plot
        self.axis.set_xlabel('distance')
        
        start_time = time.time()
        # Updating canvas
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # print(f"UI: {round(time.time() - start_time, 4)}")
        
        # Terminate if exiting Matplotlib || Check if return is true to break in the main loop
        if not plt.get_fignums():
            print("Matplotlib window is closed. Exiting.")
            return True
        else:
            return False

