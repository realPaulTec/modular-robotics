running_composite = 0
for label, cluster_data in clusters.items():
    # Skip noise
    if label == -1: continue

    # Mahalanobis
    # Bhattacharyya
    # Wasserstein

    # Compute weighted composit distance
    cluster_data['composite_distance']\
        = utils.wasserstein_distance(cluster_data['mean_vector'], cluster_data['covariance'], current_prediction, covariance_matrix)

    # Update running distance metrics!
    running_composite       += cluster_data['composite_distance']

#---

# Normalization loop
for label, cluster_data in clusters.items():
    if label == -1: continue

    # Normalize the distances by dividing by running totals!
    cluster_data['composite_distance']      /= running_composite

#---

# Find cluster with lowest distance metric
closest_cluster_label = min(filtered_keys, key=lambda k: clusters[k]["composite_distance"], default=None)

#---

if closest_cluster_label:
    if self.hungarian_estimate != closest_cluster_label:
        print(f'C{closest_cluster_label} H{self.hungarian_estimate}')

    if not self.hungarian_estimate:
        pass

    # Set current clusters to historic
    self.previous_target    = closest_cluster_label
    self.previous_clusters  = clusters

    # Reset first track
    self.first_track = False

    # Set tracked position
    self.tracked_point = clusters[closest_cluster_label]['central_position']
    self.last_track = time.time()

    # Update filter accuracy
    if len(self.prediction) > 0:
        self.kalman_accuracy = np.mean(clusters[closest_cluster_label]['mean_vector'] - self.prediction)

    # Update Kalman filter
    self.kalman_filter.update(clusters[closest_cluster_label])

elif (self.last_track + self.MAX_TRACK_LIFETIME) < time.time():
    # Reset lost track after lifetime exceeded
    self.reset_tracking()