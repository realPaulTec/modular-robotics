import numpy as np
import ot
from scipy.linalg import sqrtm
from scipy.interpolate import interp1d
from sklearn.kernel_approximation import svd
from sklearn.neighbors import KDTree
from scipy.optimize import linear_sum_assignment

# Conversion formulas
polar_to_cartesian = lambda r, theta: (r * np.cos(theta), r * np.sin(theta))
cartesian_to_polar = lambda x, y: (np.sqrt(x**2 + y**2), np.arctan2(y, x))

# Euler's formula: e^(iθ)=cos(θ)+i*sin(θ)
def calculate_mean_angle(angles):
    # convert angles to unit vectors in the complex plane using NumPy's vectorized operations
    complex_numbers = np.exp(1j * np.array(angles))
    
    # compute the mean complex number
    mean_complex = np.mean(complex_numbers)
    
    # retrieve the angle of the mean complex number
    mean_angle = np.angle(mean_complex)
    return mean_angle

# Mahalanobis Distance: D² = (x - μ)' Σ⁻¹ (x - μ)
def mahalanobis_distance(x, μ, Σ):
    delta = np.array(x) - np.array(μ)
    inv_Σ = np.linalg.inv(Σ)
    return np.sqrt(np.dot(np.dot(delta, inv_Σ), delta.T))

def composite_mahalanobis_distance(x, Y, VI):
    deltas = Y - x
    return np.sqrt(np.sum(np.dot(deltas, VI) * deltas, axis=1))

# Calculate polar distance
def distance_polar(primary, secondary):
    # Primary and secondary points
    distance1, angle1 = primary
    distance2, angle2 = secondary
    
    # Convert the primary and secondary points to complex numbers
    primary_complex = distance1 * np.exp(1j * angle1)
    secondary_complex = distance2 * np.exp(1j * angle2)
    
    # Calculate the distance between the primary and secondary points
    distance = np.abs(primary_complex - secondary_complex)
    
    return distance

# Offset points by angle and distance
def offset_polar_coordinates(coordinates, linear_displacement, angular_displacement):
    # Convert angular displacement to radians
    angular_displacement_rad = np.deg2rad(angular_displacement)

    # Extract radius and angle arrays
    radii, angles = np.transpose(coordinates)

    # Rotate the angle and normalize it within [0, 2*pi)
    angles_rotated = (angles - angular_displacement_rad) % (2 * np.pi)

    # Convert the polar coordinates to Cartesian coordinates
    x_rotated = radii * np.cos(angles_rotated)
    y_rotated = radii * np.sin(angles_rotated)

    # Calculate the displacement in Cartesian coordinates
    dx = linear_displacement * np.cos(angular_displacement_rad)
    dy = linear_displacement * np.sin(angular_displacement_rad)

    # Apply the displacement
    x_displaced = x_rotated + dx
    y_displaced = y_rotated + dy

    # Convert back to polar coordinates and stack them together
    radii_displaced = np.sqrt(x_displaced**2 + y_displaced**2)
    angles_displaced = np.arctan2(y_displaced, x_displaced)
    displaced_coordinates = np.stack((radii_displaced, angles_displaced), axis=-1)

    return displaced_coordinates

# Calculate mean and covariance from 2D point cloud
def mean_and_covariance(data, calc_covariance):
    # Compute mean vector of cluster
    mean_vector = np.mean(data, axis=0)
    
    # Compute covariance if necessary
    covariance_matrix = np.cov(data, rowvar=False) if calc_covariance else None
    
    return mean_vector, covariance_matrix

# Calculate Bhattacharyya distance metric
def bhattacharyya_distance(mean1, cov1, mean2, cov2):
    # Compute the mean difference between the two distributions
    mean_diff = mean2 - mean1
    # Calculate the average covariance matrix
    cov_mean = (cov1 + cov2) / 2
    
    # Use Cholesky decomposition for more stable inverse calculation
    chol_cov_mean = np.linalg.cholesky(cov_mean)
    inv_cov_mean = np.linalg.inv(chol_cov_mean).T @ np.linalg.inv(chol_cov_mean)
    
    # Compute the first term using the stable inverse
    term1 = 1/8 * mean_diff.T @ inv_cov_mean @ mean_diff
    
    # Use Cholesky decomposition for determinant calculation to improve numerical stability
    det_cov_mean = np.linalg.det(chol_cov_mean) ** 2
    det_cov1 = np.linalg.det(np.linalg.cholesky(cov1)) ** 2
    det_cov2 = np.linalg.det(np.linalg.cholesky(cov2)) ** 2
    
    # Compute the second term using determinants obtained from Cholesky decomposition
    term2 = 1/2 * np.log(det_cov_mean / np.sqrt(det_cov1 * det_cov2))
    
    # Calculate the Bhattacharyya distance
    distance = term1 + term2
    return distance

# Calculate Wasserstein distance metric for 2D distributions
# def wasserstein_distance(mean1, cov1, mean2, cov2):
#     mean_diff = np.array(mean1) - np.array(mean2)
#     mean_dist_squared = np.dot(mean_diff, mean_diff)

#     cov_sqrt = sqrtm(np.dot(np.dot(cov1, cov2), cov1))
#     cov_dist = np.trace(cov1 + cov2 - 2*cov_sqrt)

#     return np.sqrt(mean_dist_squared + cov_dist)

def wasserstein_distance(mean1, cov1, mean2, cov2):
    mean_diff = np.array(mean1) - np.array(mean2)
    mean_dist_squared = np.dot(mean_diff, mean_diff)

    cov2_sqrt = sqrtm(cov2)
    
    # Ensuring the square root matrix is real if its imaginary part is negligible.
    if np.iscomplexobj(cov2_sqrt)   : cov2_sqrt = np.real(cov2_sqrt)

    cov_sqrt = sqrtm(np.dot(np.dot(cov2_sqrt, cov1), cov2_sqrt))
    if np.iscomplexobj(cov_sqrt)    : cov_sqrt = np.real(cov_sqrt)
        
    cov_dist = np.trace(cov1) + np.trace(cov2) - 2 * np.trace(cov_sqrt)

    return np.sqrt(mean_dist_squared + cov_dist)

def general_wasserstein_distance(distribution1, distribution2, epsilon=0.5):
    # Define uniform weights for each cluster
    n1 = distribution1.shape[0]
    n2 = distribution2.shape[0]
    weights1 = np.ones(n1) / n1
    weights2 = np.ones(n2) / n2

    # Calculate the cost matrix (Euclidean distances between points in the two clusters)
    cost_matrix = ot.dist(distribution1, distribution2, metric='euclidean')

    # Compute the 2-Wasserstein distance
    return ot.sinkhorn2(weights1, weights2, cost_matrix, epsilon)

def composite_wasserstein_distance(distribution1, distribution2, cov_matrix2, epsilon=0.01):
    n1 = distribution1.shape[0]
    n2 = distribution2.shape[0]
    weights1 = np.ones(n1) / n1
    weights2 = np.ones(n2) / n2
    
    # Invert the covariance matrix for Mahalanobis distance calculations
    VI = np.linalg.inv(cov_matrix2)
    
    # Calculate the cost matrix using Mahalanobis distance
    cost_matrix = np.zeros((n1, n2))
    for i, x in enumerate(distribution1):
        cost_matrix[i, :] = composite_mahalanobis_distance(x, distribution2, VI)
    
    # Compute the 2-Wasserstein distance
    return ot.sinkhorn2(weights1, weights2, cost_matrix, epsilon)
    
    # # Define uniform weights for each cluster
    # n1 = distribution1.shape[0]
    # n2 = distribution2.shape[0]
    # weights1 = np.ones(n1) / n1
    # weights2 = np.ones(n2) / n2

    # # Calculate the inverse of the covariance matrix for distribution2
    # VI = np.linalg.inv(cov_matrix2)

    # # Calculate the cost matrix (Mahalanobis distances)
    # cost_matrix = np.zeros((n1, n2))
    # for i, x in enumerate(distribution1):
    #     cost_matrix[i, :] = composite_mahalanobis_distance(x, distribution2, VI)

    # # Compute the 2-Wasserstein distance
    # return ot.sinkhorn2(weights1, weights2, cost_matrix, epsilon)

# Calculate polar centers using Euler's formula
def calculate_polar_center(coordinates):
    # Calculate mean angle with Euler's formula
    mean_angle = calculate_mean_angle(coordinates[:, 1])

    # Compute the mean distance
    mean_distance = np.mean(coordinates[:, 0])

    # Update the central position
    return (mean_distance, mean_angle)

# Calculate the length of the clusters
def calculate_cluster_length(coordinates):
    # Get differences of consecutive points
    differences = np.diff(coordinates, axis=0)
    
    # Calculate euclidean distances between consecutive points
    distances = np.sqrt(np.sum(differences ** 2, axis=1))
    
    # Return the sum of the distances
    return  np.sum(distances)

# Match clusters based on Hungarian algorithm
def match_clusters(previous_clusters, current_clusters, threshold):
    # Convert cluster dictionaries to lists for easier indexing
    prev_labels, prev_data = zip(*previous_clusters.items())
    curr_labels, curr_data = zip(*current_clusters.items())
    
    # Create a cost matrix based on the Wasserstein distance
    cost_matrix = np.array([[wasserstein_distance(p['mean_vector'], p['covariance'], c['mean_vector'], c['covariance'])
                             for c in curr_data] for p in prev_data])
    
    # Apply the Hungarian algorithm
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    
    # Filter out unmatched previous clusters 
    matched_labels = [(prev_labels[i], curr_labels[j], cost_matrix[i, j]) for i, j in zip(row_ind, col_ind) if j < len(curr_data)]
    
    return matched_labels