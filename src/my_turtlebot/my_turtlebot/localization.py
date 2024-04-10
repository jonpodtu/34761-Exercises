import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import geometry_msgs.msg
from scipy.spatial.transform import Rotation as Rot
from tf2_ros.transform_broadcaster import TransformBroadcaster
import numpy as np
from sklearn.neighbors import KDTree
from copy import deepcopy
import matplotlib.pyplot as plt
from my_turtlebot.utils.icp import IterativeClosestPoint

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.laser_scan = None
        self.laser_scan_old = None
        self.poses = []
        self.velocity = None
        self.prev_position = np.array([0, 0, 0]) # x, y, theta
        self.max_iterations = 100
        self.delta_time = 1
        
        self.tfb_ = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
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

        self.timer = self.create_timer(self.delta_time, self.timed_callback)
    
    def scan_callback(self, msg):
        self.laser_scan = self.scan_to_points(msg)
        
    def scan_to_points(self, laser_scan):
        # Convert LaserScan data to point cloud
        angles = np.arange(laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment)
        distances = np.array(laser_scan.ranges)

        # Remove inf values
        inf_indices = np.isinf(distances)
        distances = distances[~inf_indices]
        angles = angles[~inf_indices]        
        points = np.vstack((distances * np.cos(angles), distances * np.sin(angles))) # x, y

        # Add z dimesion and convert to homogeneous coordinates
        points = np.vstack((points, np.zeros(points.shape[1]), np.ones(points.shape[1])))

        return points
    
    def get_prediction_model(self):
        if len(self.poses) < 2:
            return np.eye(4)
        T = np.linalg.inv(self.poses[-2]) @ self.poses[-1]
        return T
    
    def cmd_vel_callback(self, msg):
        self.velocity = msg

    def get_prediction_model_cmd_vel(self):
        """
        The vehicle only has forward/backward motion with an angle
            cmd_vel.linear.x in m/s
            cmd_vel.angular.z in rad/s
        This function should return the transformation matrix T
        """
        T = np.eye(4)
        T[:3, 3] = np.array([self.velocity.linear.x, 0, 0]) * self.delta_time
        T[:3, :3] = Rot.from_euler('z', self.velocity.angular.z * self.delta_time, degrees=False).as_matrix()

        return T
    
    def update_position(self, laser_scan):
        # Step 1: Motion prediction
        T_pred = self.get_prediction_model()

        T_prev = self.poses[-1] if self.poses else np.eye(4)

        if self.laser_scan_old is None:
            self.laser_scan_old = T_pred @ laser_scan
            return T_pred
        
        # Step 2: Apply T_pred and  (local frame) and T_prev (global frame) to get the source points S
        initial_guess = T_prev @ T_pred
        
        # Source get moved with initial guess
        S = initial_guess @ laser_scan

        # Target is the old laser scan or map
        q = self.laser_scan_old

        # Step 3: Match the updated points S with the current laser scan points
        R, t, k = IterativeClosestPoint(S[:3, :], q[:3, :], self.max_iterations)

        Delta_T_ICP = np.eye(4)
        Delta_T_ICP[:3, :3] = R
        Delta_T_ICP[:3, 3] = t.reshape(-1)

        # Step 4: Update the error of the motion prediction
        T = Delta_T_ICP @ T_prev @ T_pred
        self.laser_scan_old = T @ laser_scan

        self.poses.append(T)

        return T
    
    def plot_clouds(self, S, q, T_prev, T):
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        ax.scatter(self.laser_scan_old[0, :], self.laser_scan_old[1, :], c='g', label=r'$S_{prev}$')
        ax.scatter(S[0, :], S[1, :], c='r', label=r'$S$ ($T_{pred} @ S$)', s = 12)
        ax.scatter(q[0, :], q[1, :], c='b', label=r'$q$ ($S_{prev})', s = 12)
        ax.scatter(T_prev[0, 3], T_prev[1, 3], c='k', label='Robot before', s = 100, marker='x')
        ax.scatter(T[0, 3], T[1, 3], c='k', label='Robot now', s = 100, marker='x')
        ax.scatter(self.laser_scan_old[0, :], self.laser_scan_old[1, :], c='orange', label=r'Moved laserscan', s = 12)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title("Current speed in x diretion: {} m/s".format(T_prev[0, 3] / self.delta_time))
        ax.legend()
        ax.set_xlim(-1, 10)
        ax.set_ylim(-4, 4)
        plt.savefig('pointclouds.png')
    
    def publish_transform(self, position, rotation):
        # Publish the transform
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        
        q = Rot.from_euler('z', rotation).as_quat()
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tfb_.sendTransform(t)

    def timed_callback(self):
        if self.velocity is not None and self.laser_scan is not None:
            laser_scan = deepcopy(self.laser_scan)
            T_prev = self.update_position(laser_scan)
            self.prev_position = T_prev[:3, 3]
            self.prev_position[2] = Rot.from_matrix(T_prev[:3, :3]).as_euler('xyz')[-1]

            #self.get_logger().info(f"Current position is: {self.prev_position}")
            self.publish_transform(T_prev[:3, 3], self.prev_position[2])
            self.get_logger().info(f"New position: \n{self.prev_position}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()