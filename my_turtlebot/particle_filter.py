# This files solves the following
# - Prints the number of particles us from the existing "/particle_cloud" topic
# - Computes the expected position of the robot using the particle cloud

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform
from pyquaternion import Quaternion as PyQuaternion
import tf2_geometry_msgs
from sensor_msgs_py import point_cloud2
from laser_geometry import LaserProjection
from scipy.spatial import KDTree
from nav_msgs.srv import GetMap
from std_msgs.msg import Header


class ParticleEstimationNode(Node):
    def __init__(self):
        super().__init__('particle_estimation_node')

        # MAP SETTINGS:
        self.client = self.create_client(GetMap, '/map_server/map')
        self.get_logger().info('Waiting for service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service available, sending request...')
        self.send_request()


        self.poses = []
        self.delta_time = 1.0

        # Reference particle cloud
        self.particles_cloud_ref = None

        # Initialize particles
        self.N_init_particles = 100
        self.particle_cloud = ParticleCloud()

        # Set the header
        self.particle_cloud.header = Header()
        self.particle_cloud.header.stamp = self.get_clock().now().to_msg()
        self.particle_cloud.header.frame_id = 'map'

        # Initialize a sequence of Particle messages
        self.particle_cloud.particles = []

        for i in range(self.N_init_particles):
            p = geometry_msgs.msg.Pose()
            p.position.x = np.random.uniform(-0.2, 0.2)
            p.position.y = np.random.uniform(-0.2, 0.2)
            p.orientation.z = np.random.uniform(-np.pi, np.pi)
            p.orientation.w = 1.0
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

    def send_request(self):
        request = GetMap.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        # The map is in response.map
        self.map = self.occupancy_grid_to_array(response.map)
        self.get_logger().info('Received map')

    def occupancy_grid_to_array(self, occupancy_grid: OccupancyGrid):
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        data = np.array(occupancy_grid.data).reshape(height, width)

        # Get the indices of the cells that are occupied.
        # Here, we consider a cell to be occupied if its value is greater than 50.
        occupied_cells = np.argwhere(data > 50)

        # Convert the indices to world coordinates
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y

        occupied_points = resolution * occupied_cells + np.array([origin_x, origin_y])

        return occupied_points

    def pose_to_transform(self, pose, frame_id):
        transform_stamped = TransformStamped()
        transform_stamped.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)
        transform_stamped.transform.translation = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        transform_stamped.transform.rotation = Quaternion(x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z, w=pose.orientation.w)
        return transform_stamped

    def cmd_vel_callback(self, msg):
        self.velocity = msg
    
    def motion_update(self, cmd_vel):
        """
        The vehicle only has forward/backward motion with an angle
            cmd_vel.linear.x in m/s
            cmd_vel.angular.z in rad/s
        This function should return the transformation matrix T
        """
        # Create a TransformStamped object
        T = TransformStamped()

        # Set the translation
        T.transform.translation = Vector3(x=cmd_vel.linear.x * self.delta_time, y=0., z=0.)

        # Set the rotation
        quaternion = PyQuaternion(axis=[0, 0, 1], angle=cmd_vel.angular.z * self.delta_time)
        T.transform.rotation = Quaternion(w=quaternion.w, x=quaternion.x, y=quaternion.y, z=quaternion.z)
        
        # Update all the particles
        for i in range(len(self.particle_cloud.particles)):
            self.particle_cloud.particles[i] = tf2_geometry_msgs.do_transform_pose(self.particle_cloud.particles[i], T)


    def scan_callback(self, msg):
        laser_proj = LaserProjection()
        self.laser_scan = laser_proj.projectLaser(msg)

    def point_cloud2_to_array(self, cloud):
        reading = [list(p) for p in point_cloud2.read_points(cloud, field_names=("x", "y"), skip_nans=True)]
        return np.array(reading)    

    def compare_scans(self, laser_points, map):
        # Create a KDTree from the map points
        tree = KDTree(map)
        # For each point in the laser scan, find the nearest point in the map
        dist, _ = tree.query(laser_points)

        # The weight is the inverse of the average distance
        weight = 1.0 / np.mean(dist)

        return weight


    def sensor_update(self, laser_scan, map):
        weights = np.zeros(len(self.particle_cloud.particles))
        for i in range(len(self.particle_cloud.particles)):
            # Get the particle
            p = self.particle_cloud.particles[i]

            # Transform the laser scan data according to the particle's pose
            laser_scan_2D = self.point_cloud2_to_array(laser_scan)

            # Transform 2D laserscan given the particle pose
            for j in range(len(laser_scan_2D)):
                x = laser_scan_2D[j][0]
                y = laser_scan_2D[j][1]
                laser_scan_2D[j][0] = p.position.x + x * np.cos(p.orientation.z) - y * np.sin(p.orientation.z)
                laser_scan_2D[j][1] = p.position.y + x * np.sin(p.orientation.z) + y * np.cos(p.orientation.z)
            
            # point_transform = self.pose_to_transform(p, 'frame_id')
            # transformed_laser_scan = tf2_geometry_msgs.do_transform_vector3(laser_scan, point_transform)

            # Compute the weights
            weights[i] = self.compare_scans(laser_scan_2D, map)

        # Normalize the weights
        weights = weights / np.sum(weights)

        assert np.isclose(np.sum(weights), 1.0), "The weights should sum to 1"
        
        return weights

    def average_particles(self, particles, weights):
        x = np.zeros((4, 1))
        for i in range(len(particles)):
            x += particles[i].reshape(-1, 1) * weights[i]

        return x
    
    def update_particles(self, u, z):
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
        indices = np.random.choice(np.arange(len(self.particle_cloud.particles)), size=len(self.particle_cloud.particles), p=weights)

        for i in indices:
            new_particle_cloud.particles.append(self.particle_cloud.particles[i])
        
        self.particle_cloud = new_particle_cloud
        return weights
        
    def particle_cloud_callback(self, msg):
        self.particles_cloud_ref = msg

    def extract_particles(self, particle_cloud, weights = None):
        # Extract relevant entities
        particles = np.array([[p.position.x, p.position.y, p.orientation.z, p.orientation.w] for p in particle_cloud.particles])
        if not isinstance(weights, np.ndarray):
            weights = np.array([p.weight for p in particle_cloud.particles])

        return particles, weights

    def angle_from_quaternion_zw(self, q):
        q = q.reshape(-1)
        r = Rot.from_quat([0.0, 0.0, q[2], q[3]])
        return r.as_euler('zyx')[0]

    def timed_callback(self):
        if self.particle_cloud is not None and self.laser_scan is not None and self.map is not None and self.velocity is not None:          
            # Estimate position
            weights = self.update_particles(self.velocity, self.laser_scan)
            particles_array, weights_array = self.extract_particles(self.particle_cloud, weights=weights)
            self.position = self.average_particles(particles_array, weights_array)
            self.get_logger().info(f"""Estimated position our implementation: 
                                   {self.position[0], self.position[1], self.angle_from_quaternion_zw(self.position)}
                                   & number of particles: {len(self.particle_cloud.particles)}
                                   """)
            
            # TODO: Fix bug regard publishing the pointcloud
            # Publish the particle cloud
            # self.particle_cloud_publisher.publish(self.particle_cloud)

            # particles_array, weights_array = self.extract_particles(self.particles_cloud_ref)
            # position_ref = self.average_particles(particles_array, weights_array)
            # self.get_logger().info(f"Estimated position our implementation: {position_ref[0], position_ref[1], self.angle_from_quaternion_zw(position_ref)}")
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