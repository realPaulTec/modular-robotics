import time
import numpy as np

def get_speed(speed, thresh=20):
    if np.abs(speed) < thresh:
        return np.sign(speed) * thresh
    
    else:
        return speed
    
 # 10 * np.sign(x) * np.sqrt(np.abs(x))

def correct_angle(angle, heading):
    # Adjust object_degree by magnetic_heading
    adjusted_angle = angle + heading - 360
    
    # Normalize the result to be within -180 to 180 degrees
    adjusted_angle = (adjusted_angle + 180) % 360 - 180
    
    return adjusted_angle

def _obstacle_detection(clusters, min_angle, max_angle, max_distance, heading):
    # Return false if there are no clusters
    if len(clusters) == 0: return False
    
    for label, cluster_data in clusters.items():
        # Get angle and radius from cluster center
        radius, angle = cluster_data['central_position']
        
        # Skip if radius not in range
        if radius > max_distance: continue

        # Normalize angle
        angle = correct_angle(np.rad2deg(-angle), heading)
        angle += 360 if angle < 0 else 0

        if min_angle > max_angle and\
            (angle > min_angle or angle < max_angle)    : return True
        elif min_angle < max_angle and\
            angle > min_angle and angle < max_angle     : return True

    # Return false is there is no obstacle in the area
    return False

def obstacle_detection(tracking, min_angle, max_angle, depth, label=None):
    # Return false if there are no clusters
    if len(tracking._coordinates) == 0: return False
  
    # Get comparison type
    s = True if min_angle > max_angle else False

    # Convert to radians
    min_angle = np.deg2rad(min_angle)
    max_angle = np.deg2rad(max_angle)

    for radius, angle in tracking._coordinates:
        # Skip if radius not in range
        if radius > depth: continue

        # print(-angle)

        if s and (-angle > min_angle or -angle < max_angle)                 : return True
        elif s == False and (-angle > min_angle and -angle < max_angle)     : return True

def get_heading(bno):
    # Read compass data (Heading, Roll, Pitch)
    for i in range(3):
        try:
            heading, roll, pitch = bno.getVector(bno.VECTOR_EULER)
            return heading
        except Exception:
            time.sleep(0.01)
            continue
