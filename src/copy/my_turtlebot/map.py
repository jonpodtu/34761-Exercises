# Exercise: Publishing a map
# - Create a 2D occupancy grid using a 2D matrix
# - Fill random cells in the matrix with the value 100 and the rest with 0
# - Create an OccupancyGrid message and fill in the information along with the map
# - Publish the occupancy grid map to the topic /map with a frequency of 0.5 Hz
# - Remember to add a transform that the map can live in 

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros.transform_broadcaster import TransformBroadcaster
import geometry_msgs
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher_node')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map2', qos)
        self.tfb_ = TransformBroadcaster(self)
        self.map = OccupancyGrid()
        self.map.info.resolution = 0.05
        self.map.info.width = 400
        self.map.info.height = 200

        # Center the map around robot
        self.map.info.origin.position.x = -self.map.info.width * self.map.info.resolution / 2
        self.map.info.origin.position.y = -self.map.info.height * self.map.info.resolution / 2
        self.map.info.origin.position.z = 0.0
        self.map.info.origin.orientation.w = 1.0
        self.map.header.frame_id = "map"
        self.map.header.stamp = self.get_clock().now().to_msg()
        # Create map and fill with 0
        self.map.data = [0] * (self.map.info.width * self.map.info.height)

        # Laser scan subscriber
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS profile depth
        )
        self.lidar_points = None

        # Robot transform subscriber
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # QoS profile depth
        )

        # Callback settings
        self.timer = self.create_timer(0.5, self.timed_callback)

    def odom_callback(self, msg):
        # The msg argument contains the odometry data
        # msg.pose.pose contains the position and orientation of the robot
        self.robot_pose = msg.pose.pose
    
    def publish_transform(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tfb_.sendTransform(t)

    def timed_callback(self):
        self.publish_map()
        self.publish_transform()

    def update_random_points(self):
        # Make random 2D grid with np
        points2d = np.random.rand(self.map.info.height, self.map.info.width)
        idx_i, idx_j = np.where(points2d > 0.95)
        
        for i, j in zip(idx_i, idx_j):
            self.map.data[i * self.map.info.width + j] = 100

    def scan_callback(self, laser_scan):
        """
        Reads the laser scan data and converts it to a point cloud
        """

        # Convert LaserScan data to point cloud
        angles = np.arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)
        distances = np.array(laser_scan.ranges)
        
        # Remove inf values
        inf_indices = np.isinf(distances)
        distances = distances[~inf_indices]
        angles = angles[~inf_indices]        
        self.lidar_points = np.vstack((distances * np.cos(angles), distances * np.sin(angles))) # x, y

    def update_lidar_points(self):
        if self.lidar_points is not None and self.robot_pose is not None:
            # Convert the lidar points to map coordinates
            # First, apply the robot's pose to the lidar points
            theta = 2 * np.arccos(self.robot_pose.orientation.w)  # Robot's yaw angle
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                        [np.sin(theta), np.cos(theta)]])
            translation_vector = np.array([self.robot_pose.position.x, self.robot_pose.position.y])
            transformed_points = np.dot(rotation_matrix, self.lidar_points) + translation_vector.reshape(-1, 1)

            # Then, convert to map coordinates
            map_x = (transformed_points[0] - self.map.info.origin.position.x) / self.map.info.resolution
            map_y = (transformed_points[1] - self.map.info.origin.position.y) / self.map.info.resolution
            map_x = np.floor(map_x).astype(int)
            map_y = np.floor(map_y).astype(int)
            map_x = np.clip(map_x, 0, self.map.info.width - 1)
            map_y = np.clip(map_y, 0, self.map.info.height - 1)
            map_indices = map_y * self.map.info.width + map_x
            for i in map_indices:
                self.map.data[i] = 100

    def publish_map(self):
        # Use np to fill the map with random values
        self.update_lidar_points()
        self.map_publisher.publish(self.map)

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin(map_publisher)
    rclpy.shutdown()