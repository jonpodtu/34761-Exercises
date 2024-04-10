import numpy as np
import matplotlib.pyplot as plt
import math
from collections import defaultdict
from shapely.geometry import LineString, Point
from skimage.morphology import dilation, disk

np.random.seed(1337)

def point_inside_circle(node1, obstacle) -> bool:
    """Calculates if a points falls inside an obstacle

    Args:
        node1 (Tuple[float, float]): Sampled node
        obstacle (Tuple[float, float, float]): Obstacle to be checked

    Returns:
        bool: True if node falls inside obstacle, else False
    """
    cx, cy, r = obstacle
    x1, y1 = node1
    # Check if point is inside the circle
    return False if math.sqrt((x1 - cx) ** 2 + (y1 - cy) ** 2) > r else True

def find_k_nearest_neighbors(nodes, k):
    """Findes the indicies of the K nearest neighbors of to any give node

    Args:
        nodes (List[Tuple[float, float]]): List of the nodes positions
        k (int): Number of nearest nodes to connect to

    Returns:
        dict: dictionary where the key-value pairs are:  node_idx: [neighbor_1_idx, neighbor_2_idx, ..., neighbor_k_idx]
    """
    node_neighbors = defaultdict(list)

    for i, node1 in enumerate(nodes):
        distances = []
        for j, node2 in enumerate(nodes):
            if i != j:
                (x1, y1), (x2, y2) = node1, node2
                dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                distances.append((dist, j))
        k_nearest = sorted(distances)[:k]
        for _, neighbor_idx in k_nearest:
            node_neighbors[i].append(neighbor_idx)

    return node_neighbors


class PRM:
    def __init__(self, num_nodes, map_size, num_neighbors, map_resolution):
        self.num_nodes = num_nodes
        self.map_width = map_size[0]
        self.map_height = map_size[1]
        self.num_neighbors = num_neighbors
        self.obstacles = []
        self.nodes = []
        self.node_neighbors = None
        self.edges = defaultdict(list)
        self.map_resolution = map_resolution

        self.start = None
        self.start_idx = None

        self.goal = None
        self.goal_idx = None

        self.map = None

    def dilate_map(self, map_data, clearance):
        """
        Dilate the obstacles on the map to replicate path clearance.

        Args:
            map_data (List[int]): The map data representing obstacles.
            clearance (float): The clearance value to add around obstacles in metres.

        Returns:
            List[int]: The dilated map data.
        """
        # Save an isntance of map data
        self.map_data = map_data

        disk_radius = int(clearance / self.map_resolution)

        # Convert map_data to a 2D array
        map_array = np.array(map_data).reshape((self.map_height, self.map_width))
        dilated_map = dilation(map_array, disk(disk_radius))

        # Convert the dilated map back to a 1D array 
        self.map = dilated_map.ravel().tolist()

    def map_to_world(self, point):
        """
        Convert map coordinates to world coordinates.
        """
        x = point[0] - self.map_width // 2
        y = point[1] - self.map_height // 2

        # Take resolution into account
        x = x * self.map_resolution
        y = y * self.map_resolution

        return (x, y)

    def generate_random_nodes(self):
        for _ in range(self.num_nodes):
            collides = True
            while collides:
              x = np.random.randint(0, self.map_width)
              y = np.random.randint(0, self.map_height)
              # Check if nodes collides with obstacles
              node = (x, y)
              if not self.node_collides_obstacle(node):
                collides = False
            self.nodes.append(node)

    def compute_edges(self):
        # Calculate the k-nearest neighbors, and store them for future use.
        self.node_neighbors = find_k_nearest_neighbors(self.nodes, self.num_neighbors)
        for i, node in enumerate(self.nodes):
            for neighbor_idx in self.node_neighbors[i]:
                neighbor = self.nodes[neighbor_idx]
                # If path is free, calculate distance and create the edge
                if not self.path_collides_obstacle(node, neighbor):
                    dist = math.sqrt((node[0] - neighbor[0]) ** 2 + (node[1] - neighbor[1]) ** 2)
                    if (neighbor_idx, dist) not in self.edges[i]:
                        self.edges[i].append((neighbor_idx, dist))
                    if (i, dist) not in self.edges[neighbor_idx]:
                        self.edges[neighbor_idx].append((i, dist))

    def node_collides_obstacle(self, node):
        """
        Check if the node collides with an obstacle in the map.
        """
        x, y = node
        if self.map[y * self.map_width + x] > 0:
            return True
        return False

    def path_collides_obstacle(self, node1, node2):
        path = bresenham(node1, node2)
        for point in path:
            if self.map[point[1] * self.map_width + point[0]] > 0:
                return True
        return False

    def add_start_and_goal(self, start, goal):
        if self.node_collides_obstacle(start):
            print("Error: Starting point is not valid")
            exit(-1)

        if self.node_collides_obstacle(goal):
            print("Error: Goal point is not valid")
            exit(-1)

        # Set start and goal positions, and add them to the nodes
        self.start, self.goal = start, goal

        self.start_idx = len(self.nodes)
        self.nodes.append(start)

        self.goal_idx = len(self.nodes)
        self.nodes.append(goal)

        # Find nearest node
        neighbours = find_k_nearest_neighbors(self.nodes, k=1)
        # Get the closest (first) neighbour to the star and goal positions
        goal_neighbour_idx = neighbours[self.goal_idx][0]
        start_neighbour_idx = neighbours[self.start_idx][0]

        goal_neighbour = self.nodes[goal_neighbour_idx]
        start_neighbour = self.nodes[start_neighbour_idx]

        # computed distances
        d_goal = math.sqrt((goal[0] - goal_neighbour[0]) ** 2 + (goal[1] - goal_neighbour[1]) ** 2)
        d_start = math.sqrt((start[0] - start_neighbour[0]) ** 2 + (start[1] - start_neighbour[1]) ** 2)

        self.edges[goal_neighbour_idx].append((self.goal_idx, d_goal))
        self.edges[start_neighbour_idx].append((self.start_idx, d_start))
        self.edges[self.goal_idx].append((goal_neighbour_idx, d_goal))
        self.edges[self.start_idx].append((start_neighbour_idx, d_start))

    def plot(self, path=None, distances=None, file_name=None):
        plt.figure(dpi=150)
        for i,value in enumerate(self.map_data):
            if value > 0:
                x = i % self.map_width
                y = i // self.map_width
                plt.plot(x, y, color='black', marker='o', markersize=1)

        for node_idx, edges in self.edges.items():
            for edge in edges:
                node1 = self.nodes[node_idx]
                node2 = self.nodes[edge[0]]
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], "b-")
        for node in self.nodes:
            plt.plot(node[0], node[1], "bo")
        for obstacle in self.obstacles:
            circle = plt.Circle(
                (obstacle[0], obstacle[1]), obstacle[2], color="gray", alpha=0.5
            )
            plt.gca().add_patch(circle)

        if self.goal:
            plt.plot(self.goal[0], self.goal[1], "yo")
        if self.start:
            plt.plot(self.start[0], self.start[1], "go")

        if path:
            for i in range(len(path) - 1):
                node1 = self.nodes[path[i]]
                node2 = self.nodes[path[i + 1]]
                if path[i] != self.start_idx:
                    plt.plot(node1[0], node1[1], "ro")
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], "r-")
                plt.text(node1[0], node1[1], f"{distances[path[i]]:.2f}", fontsize=8)              

        plt.title("Probabilistic Roadmap with Obstacles")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)
        if file_name:
            plt.savefig(file_name)
        else:
            plt.show()
        # close figure
        plt.close()

    def get_path_info(self, path):
        path_info = []

        for i in range(len(path) - 1):
            node1 = self.nodes[path[i]]
            node2 = self.nodes[path[i + 1]]

            x1, y1 = node1
            x2, y2 = node2

            # Calculate the angle between the nodes
            angle = math.atan2(y2 - y1, x2 - x1)

            path_info.append({'x': x1, 'y': y1, 'theta': angle})

        # Add the last node in the path
        x, y = self.nodes[path[-1]]
        path_info.append({'x': x1, 'y': y1, 'theta': angle})  # The angle is set to 0 for the last node

        return path_info
    

def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end including.

    (original from roguebasin.com)
    >> points1 = bresenham((4, 4), (6, 10))
    >> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points
