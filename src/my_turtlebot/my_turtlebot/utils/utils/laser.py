import numpy as np
from sensor_msgs_py import point_cloud2

# Not implemented error
class RaiseNotImplementedError(Exception):
    def __init__(self, message="Function not implemented yet"):
        self.message = message
        super().__init__(self.message)

def scan_to_points(self, laser_scan, is_2D = True):
    """
    Return 2D points from a LaserScan message in 3D homogeneous coordinates.
    If the scan is 2D (is_2D=True), the z dimension is set to 0.
    """

    # Convert LaserScan data to point cloud
    angles = np.arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)
    distances = np.array(laser_scan.ranges)

    # Remove inf values
    inf_indices = np.isinf(distances)
    distances = distances[~inf_indices]
    angles = angles[~inf_indices]       
    if is_2D: 
        points = np.vstack((distances * np.cos(angles), distances * np.sin(angles))) # x, y'
        points = np.vstack((points, np.zeros(points.shape[1]), np.ones(points.shape[1])))
    else:
        RaiseNotImplementedError("3D scan not implemented yet")

    return points

def point_cloud2_to_array(cloud) -> np.array:
    """
    Takes the point cloud message and returns a numpy array with the points.
    """
    reading = [list(p) for p in point_cloud2.read_points(cloud, field_names=("x", "y"), skip_nans=True)]
    return np.array(reading)