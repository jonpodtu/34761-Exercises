from copy import deepcopy
from math import atan2, cos, sin
from time import time

import geometry_msgs.msg
import numpy as np
import rclpy
from geometry_msgs.msg import Quaternion
from laser_geometry import LaserProjection
from nav2_msgs.msg import Particle, ParticleCloud
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from pyquaternion import Quaternion as PyQuaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial import KDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster
from project.utils import angle_from_quaternion
from project.utils.laser import point_cloud2_to_array


class ParticleEstimationNode(Node):
    def __init__(self):
        super().__init__('particle_estimation_node')

        # MAP SETTINGS
        # This is the service that will provide us with the map. It stalls until the map is available.
        # In our setup we don't need to be continously updating the map, so we just get it once.
        self.client = self.create_client(GetMap, '/map_server/map')
        self.get_logger().info('Waiting for service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service available, sending request...')
        self.send_request()

        self.delta_time = 1.0

        # Reference particle cloud (for comparison)
        self.particles_cloud_ref = None

        # Initialize particles
        self.N_init_particles = 50
        self.particle_cloud = ParticleCloud()

        # Set the header
        self.particle_cloud.header = Header()
        self.particle_cloud.header.stamp = self.get_clock().now().to_msg()
        self.particle_cloud.header.frame_id = 'map'

        # Initialize a sequence of Particle messages
        self.particle_cloud.particles = []

        # We initialize the particles in a small area around the origin.
        for i in range(self.N_init_particles):
            p = Particle()
            p.pose.position.x = np.random.uniform(-0.2, 0.2)
            p.pose.position.y = np.random.uniform(-0.2, 0.2)
            # TODO: Change the orientation so it is guideded by initial rotation
            p.pose.orientation.z = np.random.uniform(-np.pi/3, np.pi/3)
            p.weight = 1.0 / self.N_init_particles
            self.particle_cloud.particles.append(p)

        # Other variables
        self.laser_scan = None
        self.map = None
        self.velocity = None
        
        self.tfb_ = TransformBroadcaster(self)
       
        # Subscriptions
        ###########################
        # ParticleCloud subscription is just to compare with our implementation
        # LaserScan subscription is used to get the laser scan data
        # cmd_vel subscription is used to get the velocity command creating the motion model
        # map subscription is used to get the map data
        ###########################
        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.particle_cloud_subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos  # QoS profile depth
        )
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS profile depth
        )
        self.cmd_vel_subscription = self.create_subscription(
            geometry_msgs.msg.Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        ############################
        # Particle cloud publisher #
        ############################
        self.particle_cloud_publisher = self.create_publisher(ParticleCloud, '/particle_cloud_own', 10)
        self.timer = self.create_timer(self.delta_time, self.timed_callback)

    # Map stuff
    def send_request(self):
        request = GetMap.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        self.map = self.occupancy_grid_to_array(response.map)
        self.get_logger().info('Received map')

    def occupancy_grid_to_array(self, occupancy_grid: OccupancyGrid):
        # TODO: Review this function. This might be the reason why the map is negative.
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data).reshape(height, width)

        # Get the indices of the cells that are occupied.
        # Here, we consider a cell to be occupied if its value is greater than 20.
        occupied_cells = np.argwhere(data > 20)

        # Convert the indices to world coordinates
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y

        occupied_points = resolution * occupied_cells + np.array([origin_x, origin_y])

        return occupied_points

    def cmd_vel_callback(self, msg):
        self.velocity = msg
    
    def motion_update(self, cmd_vel):
        """
        The vehicle only has forward/backward motion with an angle
            cmd_vel.linear.x in m/s
            cmd_vel.angular.z in rad/s
        """
        # Update all the particles
        for i in range(len(self.particle_cloud.particles)):
            # Get a copy of the current particle pose
            particle_pose = deepcopy(self.particle_cloud.particles[i].pose)

            # Convert quaternion to euler for easier calculations
            yaw = angle_from_quaternion(particle_pose.orientation)

            # Add noise to the particle pose
            # TODO: Tune the noise parameters
            particle_pose.position.x += np.random.normal(0, 0.02)
            particle_pose.position.y += np.random.normal(0, 0.02)
            yaw += np.random.normal(0, 0.02)

            # Update the particle pose
            particle_pose.position.x += cmd_vel.linear.x * cos(yaw) * self.delta_time
            particle_pose.position.y += cmd_vel.linear.x * sin(yaw) * self.delta_time
            yaw += cmd_vel.angular.z * self.delta_time            

            # Normalize the yaw angle
            # TODO: Why do we need to do this?
            yaw = atan2(sin(yaw), cos(yaw))

            # Convert the yaw back to a quaternion
            quaternion = PyQuaternion(axis=[0, 0, 1], radians=yaw)
            particle_pose.orientation = Quaternion(x=quaternion.x, y=quaternion.y, z=quaternion.z, w=quaternion.w)

            # Create a new particle with the updated pose
            particle = Particle()
            particle.pose = particle_pose
            particle.weight = self.particle_cloud.particles[i].weight

            # Replace the old particle with the new one
            self.particle_cloud.particles[i] = particle

    def scan_callback(self, msg):
        laser_proj = LaserProjection()
        self.laser_scan = laser_proj.projectLaser(msg)

    def compare_scans(self, laser_points, map) -> float:        
        # Create a KDTree from the map points
        tree = KDTree(map)
        dist, _ = tree.query(laser_points) # For each point in the laser scan, find the nearest point in the map
        weight = 1.0 / np.mean(dist) # The weight is the inverse of the average distance

        return weight

    def sensor_update(self, laser_scan, map):
        num_particles = len(self.particle_cloud.particles)
        weights = np.zeros(num_particles)

        # Convert laser scan to 2D array once, not for each particle
        laser_scan_2D = point_cloud2_to_array(laser_scan)

        for i in range(num_particles):
            # Get the particle
            p = self.particle_cloud.particles[i].pose

            # Compute yaw angle once for each particle
            yaw = angle_from_quaternion(p.orientation)
            x = laser_scan_2D[:, 0]
            y = laser_scan_2D[:, 1]
            transformed_x = p.position.x + x * np.cos(yaw) - y * np.sin(yaw)
            transformed_y = p.position.y + x * np.sin(yaw) + y * np.cos(yaw)

            # Compute the weights
            # TODO: Figure out why the map is negative. IMPORTANT.
            weights[i] = self.compare_scans(np.stack((transformed_x, transformed_y), axis=-1), -map)

        # Normalize the weights
        weights /= np.sum(weights)
        assert np.isclose(np.sum(weights), 1.0), f"The weights should sum to 1 but they sum to {np.sum(weights)}"
        
        return weights

    def average_particles(self, particles, weights):
        particles = np.array(particles)
        weights = np.array(weights).reshape(-1, 1)
        x = np.sum(particles * weights, axis=0)
        return x
    
    def update_particles(self, u, z):
        """The main function that updates the particles based on the motion and sensor model. It uses the Monte Carlo Localization Algorithm."""

        # Monte Carlo Localization Algorithm
        new_particle_cloud = ParticleCloud()

        # Set the header
        new_particle_cloud.header = Header()
        new_particle_cloud.header.stamp = self.get_clock().now().to_msg()
        new_particle_cloud.header.frame_id = 'map'  # replace with the appropriate frame id

        # Initialize a sequence of Particle messages
        new_particle_cloud.particles = []

        self.motion_update(u)
        weights = self.sensor_update(z, self.map)
        
        # Resampling
        indices = np.random.choice(np.arange(len(self.particle_cloud.particles)), size=self.N_init_particles, p=weights)

        for i in indices:
            new_particle_cloud.particles.append(self.particle_cloud.particles[i])
            new_particle_cloud.particles[-1].weight = weights[i]
        
        self.particle_cloud = new_particle_cloud
        return weights
        
    def particle_cloud_callback(self, msg):
        """Callback for the particle cloud subscription. This is used to compare with our implementation."""
        self.particles_cloud_ref = msg

    def extract_particles(self, particle_cloud, weights = None):
        """Extract the particles and weights from a particle cloud message."""
        particles = np.array([[p.pose.position.x, p.pose.position.y, angle_from_quaternion(p.pose.orientation)] for p in particle_cloud.particles])
        if not isinstance(weights, np.ndarray):
            weights = np.array([p.weight for p in particle_cloud.particles])

        return particles, weights

    def timed_callback(self):
        if self.particle_cloud is not None and self.laser_scan is not None and self.map is not None and self.velocity is not None:          
            start_time = time()
            # Estimate position
            weights = self.update_particles(self.velocity, self.laser_scan)
            particles_array, weights_array = self.extract_particles(self.particle_cloud, weights=weights)
            self.position = self.average_particles(particles_array, weights_array)
            self.get_logger().info(f"Position: {self.position[0], self.position[1], self.position[2]}")
            
            # Publish the particle cloud
            self.particle_cloud_publisher.publish(self.particle_cloud)
            end_time = time()
            if end_time - start_time > self.delta_time:
                self.get_logger().warn(f"""Time taken: {end_time - start_time}. 
                                       The computation is taking too long since the delta time is {self.delta_time}.
                                       Consider increasing the delta time or optimizing the code.""")

            # particles_array, weights_array = self.extract_particles(self.particles_cloud_ref)
            # position_ref = self.average_particles(particles_array, weights_array)
            # self.get_logger().info(f"Estimated position our implementation: {position_ref[0], position_ref[1], angle_from_quaternion_zw(position_ref)}")
        else:
            self.get_logger().info("Waiting for data. The following is missing:")
            if self.particle_cloud is None:
                self.get_logger().info("Particle cloud")
            if self.laser_scan is None:
                self.get_logger().info("Laser scan")
            if self.map is None:
                self.get_logger().info("Map")
            if self.velocity is None:
                self.get_logger().info("Command velocity")



def main(args=None):
    rclpy.init()
    node = ParticleEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()