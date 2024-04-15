"""
Implement the planner for the turtlebot.

We will use a probabilistic roadmap (PRM) to plan the path for the turtlebot. The following steps will be implemented:
- Subscribe to the map topic
- Generate random nodes
- Compute edges between the nodes
- Find the start and goal nodes
- Find the path using Djikstra's algorithm
- Publish the path as a list of points and directions

Extend the Planner class so it sets the new point to a place which needs to be explored.

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
from geometry_msgs.msg import Twist
from math import atan2, sqrt
import math
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from my_turtlebot.utils import bresenham

class PlannerNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        self.map = None
        self.robot_pose = None
        self.state = "DONE"
        self.prev_distance = math.inf
        self.over_distance_counter = 0
        map_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            "/map",
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.planner = self.create_timer(5, self.planner_timed_callback)
        self.executor_ = self.create_timer(0.1, self.executor_timed_callback)

        # Next-best-view parameters
        self.num_views = 5
        self.prev_gain = 0
        self.lambda_ = 1
        self.max_range = 3.5/2

    def localization_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def world_to_map(self, point):
        """
        Convert world coordinates to map coordinates.
        """
        x = point[0] / self.map_resolution
        y = point[1] / self.map_resolution

        x = int(x + self.map_width // 2)
        y = int(y + self.map_height // 2)

        return (x, y)

    def get_start(self):
        if self.robot_pose is not None:
            self.get_logger().info(f"Robot pose: {self.robot_pose}")
            return self.world_to_map((self.robot_pose.position.x, self.robot_pose.position.y))
        else:
            return (int(self.map_width/2), int(self.map_height/2))
    

    def calculate_information_info(self, node) -> int:
        """
        Calculate information gain given a node. The map consists of
        unknown cells (-1), free cells (0) and occupied cells (> threshold).

        We only take the area surrounding the node into account, using raycasting of laserscan.

        The gain corresponds to the number of unknown cells in the area surrounding the node.
        """
        x, y = node
        unknown_cells = set()
        free_cells = set()
        obstacle_cells = set()

        map_dimensions = (self.map_width, self.map_height)

        for angle in range(0, 360):
            point_x = x + self.max_range/self.map_resolution * math.cos(math.radians(angle))
            point_y = y + self.max_range/self.map_resolution * math.sin(math.radians(angle))
            point = (int(point_x), int(point_y))

            ray = bresenham(node, point)
            for point in ray:
                point = (int(point[0]), int(point[1]))
                try:
                    map_index = point[1] * map_dimensions[0] + point[0]
                    cell_value = self.map_data[map_index]
                    if cell_value == -1:
                        unknown_cells.add(point)
                    elif cell_value == 0:
                        free_cells.add(point)
                    elif cell_value > 0:
                        obstacle_cells.add(point)
                        break    # stop ray casting, go to next angle
                except IndexError:  # when the point is out of the map, go to next angle
                    break

        return len(unknown_cells)
    
    def dist_between_nodes(self, node1, node2):
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    
    def next_best_view(self, start):
        """
        Find the next best view by maximizing the information gain.
        """
        for node in self.prm.nodes:
            info = self.calculate_information_info(node)
            weight = self.dist_between_nodes(node, start)
            gain = self.prev_gain + info*np.exp(-self.lambda_ * weight)
            if gain > self.prev_gain:
                self.prev_gain = gain
                best_view = node
        self.get_logger().info(f"Best view: {best_view} with gain: {self.prev_gain}")
        
        return best_view
        

    def map_callback(self, msg):
        self.map = msg
        self.map_data = msg.data
        self.map_info = msg.info
        self.map_metadata = msg.info
        self.map_resolution = self.map_metadata.resolution
        self.map_width = self.map_metadata.width
        self.map_height = self.map_metadata.height
        self.map_origin = self.map_metadata.origin
        
    def create_prm(self, goal = None):
        self.prm = PRM(num_nodes = 100, map_size = (self.map_width, self.map_height), num_neighbors = 6, map_resolution=self.map_resolution)
        self.prm.dilate_map(self.map_data, clearance=0.2)
        self.prm.generate_random_nodes()
        self.prm.compute_edges()
        start = self.get_start()
        goal = self.next_best_view(start)
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

    def executor_timed_callback(self):
        # State machine which is either in ROTATE, DRIVE_FORWARD, GET_NEXT_POINT or DONE state
        if self.state == "ROTATE":
            self.rotate_to_angle(self.target_angle)
        elif self.state == "DRIVE_FORWARD":
            self.drive_forward_to_point(self.target_point)
        elif self.state == "GET_NEXT_POINT" and len(self.path) > 0:
            current_point = self.path.pop(0)
            # Convert the point from map coordinates to world coordinates
            self.target_point = self.prm.map_to_world((current_point['x'], current_point['y']))
            # Its better to the current position of the robot from the odometry rather that the predicted position from the PRM so
            # we can get the correct angle to the target point
            self.target_angle = atan2(self.target_point[1] - self.robot_pose.position.y, self.target_point[0] - self.robot_pose.position.x) 
            self.state = "ROTATE"
            self.rotate_to_angle(self.target_angle)
        elif self.state == "GET_NEXT_POINT" and len(self.path) == 0:
            self.state = "DONE"
        elif self.state == "DONE":
            self.get_logger().info("Path execution completed")
        else:
            self.get_logger().info("Unknown state. Path planning has probably never started.")

    def angle_from_quaternion(self, q):
        # Needs an orientation from the pose message
        r = Rot.from_quat([q.x, q.y, q.z, q.w])
        return r.as_euler('zyx')[0]

    def rotate_to_angle(self, target_angle):
        current_angle = self.angle_from_quaternion(self.robot_pose.orientation)
        error = target_angle - current_angle
        twist = Twist()

        # If the robot is not facing the target angle, rotate it
        if abs(error) > 0.01:  # 0.01 is a small tolerance for the error
            twist.angular.z = 0.075 if error > 0 else -0.075
            self.cmd_vel_publisher.publish(twist)
        else:
            # If the robot is facing the target angle, stop rotating
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info(f"Reached target angle: {target_angle}")
            self.state = "DRIVE_FORWARD"
        
    def drive_forward_to_point(self, target_point):
        # Get the current position of the robot
        current_position = self.robot_pose.position

        # Calculate the distance to the target point
        dx = target_point[0] - current_position.x
        dy = target_point[1] - current_position.y
        distance = sqrt(dx**2 + dy**2)

        # Create a Twist message
        twist = Twist()

        # If the robot is not at the target point, drive forward
        if distance < 0.01 or self.over_distance_counter > 10:  # 0.01 is a small tolerance for the error
            twist.linear.x = 0.0
            self.get_logger().info("Reached target point")
            self.cmd_vel_publisher.publish(twist)
            self.state = "GET_NEXT_POINT"
            self.over_distance_counter = 0
        else:
            # If the robot is at the target point, stop driving
            twist.linear.x = 0.02
            self.get_logger().info(f"Driving forward from {current_position} to {target_point}. Distance: {distance}")
            self.cmd_vel_publisher.publish(twist)
            if distance > self.prev_distance:
                self.over_distance_counter += 1
            else:
                self.over_distance_counter = 0
        self.prev_distance = distance

    def planner_timed_callback(self):
        if self.map is None:
            self.get_logger().info("Map not yet received")
            return
        if len(self.path) == 0:
            self.create_prm()
            new_path, distances = dijkstra(self.prm.edges, self.prm.start_idx, self.prm.goal_idx)
            self.get_logger().info(f"Path: {new_path}")
            self.path = self.prm.get_path_info(new_path)[1:]
            time = self.get_clock().now().nanoseconds
            self.prm.plot(path = new_path, distances = distances, file_name = f"path_{time}.png")
            self.state = "GET_NEXT_POINT"

def main(args=None):
    rclpy.init(args=args)
    planner_node = PlannerNode()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()