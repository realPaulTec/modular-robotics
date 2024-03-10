import numpy as np

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
def offset_polar_coordinates_old(coordinates, linear_displacement, angular_displacement):
    displaced_coordinates = []
    for r, theta in coordinates:
        # Rotate the angle and normalize it within [0, 2*pi) || NOTE: Degree input to the function
        theta_rotated = (theta - np.deg2rad(angular_displacement)) % (2 * np.pi)

        # Convert the polar coordinates to Cartesian coordinates
        x_rotated, y_rotated = polar_to_cartesian(r, theta_rotated)
        
        # Calculate the displacement in Cartesian coordinates
        dx = linear_displacement * np.cos(angular_displacement)
        dy = linear_displacement * np.sin(angular_displacement)
        
        # Apply the displacement
        x_displaced = x_rotated + dx
        y_displaced = y_rotated + dy
        
        # Convert back to polar coordinates
        displaced_coordinates.append(cartesian_to_polar(x_displaced, y_displaced))

    return np.array(displaced_coordinates)

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
