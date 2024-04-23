# This files solves the following
# - Prints the number of particles us from the existing "/particle_cloud" topic
# - Computes the expected position of the robot using the particle cloud

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav2_msgs.msg import ParticleCloud
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from tf2_ros.transform_broadcaster import TransformBroadcaster

class ParticleEstimationNode(Node):
    def __init__(self):
        super().__init__('particle_estimation_node')
        self.poses = []
        self.delta_time = 1.0
        self.particles = None
        self.position = None
        self.particles = None
        self.weights = None
        
        self.tfb_ = TransformBroadcaster(self)
        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos  # QoS profile depth
        )

        print("YAHOO")

        self.timer = self.create_timer(self.delta_time, self.timed_callback)
    
    def estimate_position(self, particles, weights):
        particles = np.array(particles)
        weights = np.array(weights).reshape(-1, 1)
        x = np.sum(particles * weights, axis=0)
        return x

    def particle_cloud_callback(self, msg):
        print("KEEP GOING!")
        particle_cloud = msg
        print("msg")
        # Extract relevant entities
        self.particles = np.array([[p.pose.position.x, p.pose.position.y, self.angle_from_quaternion(p.pose.orientation)] for p in particle_cloud.particles])
        self.weights = np.array([p.weight for p in particle_cloud.particles])

    def angle_from_quaternion(self, q):
        # Needs an orientation from the pose message
        r = Rot.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('zyx')[0]

    def timed_callback(self):
        print("ENTERING TIMER!")
        print(self.particles)
        if self.particles is not None:
            self.get_logger().info(f"Number of particles: {len(self.particles)}")
            print(f"Number of particles: {len(self.particles)}")
            
            # Estimate position
            self.position = self.estimate_position(self.particles, self.weights)
            self.get_logger().info(f"Estimated position: {self.position[0], self.position[1], self.position[2]}")
            print(f"Estimated position: {self.position[0], self.position[1], self.position[2]}")

def main(args=None):
    print("STARTING ESTIMATION!")
    rclpy.init()
    print("CONTINUING ESTIMATION")
    node = ParticleEstimationNode()
    rclpy.spin(node)
    print("NODE SPINNING")
    rclpy.shutdown()