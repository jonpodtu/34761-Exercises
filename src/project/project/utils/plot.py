import matplotlib.pyplot as plt
import numpy as np


def plot_scans(laser_scan_2D, map, unique_id):
    """
    Plot the map and the laser scan on top of it.

    Args:
        laser_scan_2D (List[Tuple[float, float]]): The 2D laser scan points.
        map (List[List[int]]): The map data.
    """
    # Plot the map
    laser_scan_x = [point[0] for point in map]
    laser_scan_y = [point[1] for point in map]
    plt.scatter(laser_scan_x, laser_scan_y, color='black', s=1)
    
    # Plot the laser scan points on top of the map
    laser_scan_x = [point[0] for point in laser_scan_2D]
    laser_scan_y = [point[1] for point in laser_scan_2D]
    plt.scatter(laser_scan_x, laser_scan_y, color='red', s=0.5)

    # Show the plot
    plt.savefig(f"assets/plot_{unique_id}.png")