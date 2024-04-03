"""
Implement the planner for the turtlebot.

We will use a probabilistic roadmap (PRM) to plan the path for the turtlebot. The following steps will be implemented:
- Subscribe to the map topic
- Generate random nodes
- Compute edges between the nodes
- Find the start and goal nodes
- Find the path using Djikstra's algorithm
- Publish the path as a list of points and directions

"""

import rclpy
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PoseStamped, Pose
from my_turtlebot.utils.djikstras import dijkstra 
from my_turtlebot.utils.prm import PRM


SUB_MSG = OccupancyGrid
SUB_TOPIC = "/map"

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.map = None
        self.robot_pose = None
        map_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            SUB_TOPIC,
            self.map_callback,
            map_profile
        )
        
        # Get localization based on odomotry position
        self.localization_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.localization_callback,
            10
        )
        self.prm = None
        self.path = []
        self.path_publisher = self.create_publisher(PoseStamped, '/path', 10)
        self.timer = self.create_timer(10, self.timed_callback)

    def localization_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def get_start(self):
        half_width = int(self.map_width/2)
        half_height = int(self.map_height/2)
        if self.robot_pose is not None:
            return (int(self.robot_pose.position.x + half_width), int(self.robot_pose.position.y + half_height))
        return (half_width, half_height)
    
    def get_goal(self):
        return (270, 40)

    def map_callback(self, msg):
        self.map = msg
        self.map = msg
        self.map_data = msg.data
        self.map_info = msg.info
        self.map_metadata = msg.info
        self.map_resolution = self.map_metadata.resolution
        self.map_width = self.map_metadata.width
        self.map_height = self.map_metadata.height
        self.map_origin = self.map_metadata.origin

    def create_prm(self, goal = None):
        self.prm = PRM(num_nodes = 400, map_size = (self.map_width, self.map_height), num_neighbors = 6)
        self.prm.dilate_map(self.map_data, clearance=0.2, map_resolution=self.map_resolution)
        self.prm.generate_random_nodes()
        self.prm.compute_edges()
        start = self.get_start()
        if goal is None:
            goal = self.get_goal()
        self.prm.add_start_and_goal(start, goal)

    def publish_path(self):
        path_msg = PoseStamped()
        path_msg.header.frame_id = "map"
        for point in self.path:
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)

    def timed_callback(self):
        if self.map is None:
            self.get_logger().info("Map not yet received")
            return
        self.create_prm()
        self.path, distances = dijkstra(self.prm.edges, self.prm.start_idx, self.prm.goal_idx)
        self.get_logger().info(f"Path: {self.path}")
        self.prm.plot(path = self.path, distances = distances, file_name = "path.png")
        #self.publish_path()

def main(args=None):
    rclpy.init(args=args)
    planner_node = PlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()