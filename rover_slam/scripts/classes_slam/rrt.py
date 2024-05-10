#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
import matplotlib.pyplot as plt

# Import ROS messages
from nav_msgs.msg import OccupancyGrid

# Node is the class for each point generated for the RRT
class Node:
    def __init__(self, x:int, y:int, parent=None) -> None:
        self.x = x
        self.y = y
        self.parent = parent
        self.dist = float("inf")
    
    def __lt__(self, node) -> bool:
        return self.dist < node.dist

class RRT:
    def __init__(self) -> None:
        # Initialize variables
        self.__map = None
        self.__width, self.__height = 0, 0

        # Points parameters
        self.__iterations = rospy.get_param("/planning/iterations/value", default = 1000)
        self.__max_dist = rospy.get_param("/planning/max_dist/value", default = 5)
        self.__safe_dist = rospy.get_param("/planning/safe_dist/value", default = 5)
        self.__interpolation = rospy.get_param("/planning/interpolation/value", default = 30)

        # Obstacles
        self.__obstacles = []
        
        # Subscribe to the odometry and scan topics
        rospy.Subscriber("/map", OccupancyGrid, self.__map_callback)
        rospy.wait_for_message("/map", OccupancyGrid, timeout = 30)

    # Callback function for the SLAM map
    def __map_callback(self, data:OccupancyGrid) -> None:
        self.__width = data.info.width
        self.__height = data.info.height
        self.__map = np.array(data.data, dtype = np.uint8).reshape((data.info.height, data.info.width))

    # Get each obstacle point in the map
    def __get_obstacles(self) -> None:
        for y in range(len(self.__map)):
            for x in range(len(self.__map[y])):
                if self.__map[y][x] == 100:
                    self.__obstacles.append((x, y))

    # Generate a random and valid point in the map
    def __generate_point(self) -> tuple:
        while True:
            x = np.random.randint(0, self.__height)
            y = np.random.randint(0, self.__width)
            if self.__validate_point((x, y)):
                break
        return x, y

    # Get distance between two points
    def __dist(self, point1:tuple, point2:tuple) -> float:
        return np.linalg.norm(np.array(point1) - np.array(point2))

    # Validate point to be in open space and far from obstacles
    def __validate_point(self, point:tuple) -> bool:
        if self.__map[point[1]][point[0]] != 0:
            return False
        
        if any(self.__dist(point, obs) < self.__safe_dist for obs in self.__obstacles):
            return False
        return True

    # Get the nearest existing node for the current point
    def __nearest_node(self, nodes:list, point:tuple) -> Node:
        min_dist = float('inf')
        nearest = None
        for node in nodes:
            dist = self.__dist(point, (node.x, node.y))
            if dist < min_dist:
                interpolation = np.linspace(point, (node.x, node.y), self.__interpolation)
                if any(self.__map[int(p[1])][int(p[0])] != 0 for p in interpolation):
                    continue
                min_dist = dist
                nearest = node
        return nearest

    # Get the current point to a maximum distance from the nearest node
    def __approach_point(self, nearest:Node, point:tuple) -> tuple:
        if self.__dist(point, (nearest.x, nearest.y)) <= self.__max_dist:
            return point

        theta = np.arctan2(point[1] - nearest.y, point[0] - nearest.x)
        x = nearest.x + int(self.__max_dist * np.cos(theta))
        y = nearest.y + int(self.__max_dist * np.sin(theta))
        new_point = (x, y)

        if self.__validate_point(new_point):
            return new_point
        return None

    # Perform the RRT algorithm
    def rrt(self) -> list:
        start = self.__generate_point()
        nodes = [Node(start[0], start[1])]
        self.__get_obstacles()

        for _ in range(self.__iterations):
            point = self.__generate_point()
            nearest = self.__nearest_node(nodes, point)
            if nearest is not None:
                point = self.__approach_point(nearest, point)
                if point is not None:
                    node = Node(x=point[0], y=point[1], parent=nearest)
                    nodes.append(node)
        return nodes

    # Plot the map, RRT points and path
    def plot(self, nodes:list, path:list) -> None:
        plt.figure(figsize = (8, 8))
        plt.imshow(self.__map, cmap = 'Greys', origin = 'lower')
        for node in nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue')
        if path is not None:
            path_x = [node.x for node in path]
            path_y = [node.y for node in path]
            plt.plot(path_x, path_y, color = 'red', linewidth = 2)
        plt.show()

    # Stop the algorithm
    def stop(self) -> None:
        rospy.loginfo("Shutting down the RRT")
