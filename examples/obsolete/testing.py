import numpy as np
from scipy.spatial import KDTree
from numpy.linalg import svd

def find_rotation_matrix_2d(A, B):
    """
    Finds the rotation matrix to align two 2D point sets A and B.
    A and B are Nx2 and Mx2 arrays, respectively, with N != M.
    """
    # Center the points in A and B
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    A_centered = A - centroid_A
    B_centered = B - centroid_B

    # Build a KDTree for the larger set
    if A_centered.shape[0] > B_centered.shape[0]:
        tree = KDTree(A_centered)
        distances, indices = tree.query(B_centered)
        matched_A = A_centered[indices]
        matched_B = B_centered
    else:
        tree = KDTree(B_centered)
        distances, indices = tree.query(A_centered)
        matched_A = A_centered
        matched_B = B_centered[indices]

    # Compute the cross-covariance matrix
    H = np.dot(matched_A.T, matched_B)

    # Perform SVD
    U, S, Vt = svd(H)
    R = np.dot(Vt.T, U.T)

    # Ensure a proper rotation
    if np.linalg.det(R) < 0:
        Vt[1,:] *= -1
        R = np.dot(Vt.T, U.T)

    return R

# Example usage
A = np.random.rand(500, 2)  # Smaller point set
B = np.random.rand(650, 2)  # Larger point set, they always have different sizes

R = find_rotation_matrix_2d(A, B)
print("Rotation Matrix:\n", R)
