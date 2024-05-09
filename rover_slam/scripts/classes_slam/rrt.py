#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
import matplotlib.pyplot as plt

# Import ROS messages
from nav_msgs.msg import OccupancyGrid

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT:
    def __init__(self):
        # Initialize variables
        self.__map = None
        self.__width, self.__height = 0, 0

        # Points parameters
        self.__iterations = 2000
        self.__max_dist = 10
        self.__safe_dist = 6

        # Obstacles
        self.__obstacles = []
        self.__interpolation = 20
        
        # Subscribe to the odometry and scan topics
        rospy.Subscriber("/map", OccupancyGrid, self.__map_callback)
        rospy.wait_for_message("/map", OccupancyGrid, timeout = 30)

    # Callback function for the SLAM map
    def __map_callback(self, data:OccupancyGrid) -> None:
        # Convert the map data to a numpy array
        self.__width = data.info.width
        self.__height = data.info.height
        self.__map = np.array(data.data, dtype = np.uint8).reshape((data.info.height, data.info.width))

    def __get_obstacles(self):
        for y in range(len(self.__map)):
            for x in range(len(self.__map[y])):
                if self.__map[y][x] == 100:
                    self.__obstacles.append((x, y))

    def __generate_point(self):
        x = np.random.randint(0, self.__height)
        y = np.random.randint(0, self.__width)
        return x, y

    def __validate_point(self, point):
        if self.__map[point[1]][point[0]] != 0:
            return False

        for obstacle in self.__obstacles:
            x, y = obstacle
            if np.sqrt((point[0] - x)**2 + (point[1] - y)**2) < self.__safe_dist:
                return False
        return True

    def __nearest_node(self, nodes, point):
        min_dist = float('inf')
        nearest = None
        for node in nodes:
            dist = np.sqrt((node.x - point[0])**2 + (node.y - point[1])**2)
            if dist < min_dist:
                interpolation = np.linspace(point, (node.x, node.y), self.__interpolation)
                if any(self.__map[int(p[1])][int(p[0])] != 0 for p in interpolation):
                    continue
                min_dist = dist
                nearest = node
        return nearest

    def __approach_point(self, nearest, point):
        dist = np.sqrt((point[0] - nearest.x)**2 + (point[1] - nearest.y)**2)
        if dist <= self.__max_dist:
            return point

        theta = np.arctan2(point[1] - nearest.y, point[0] - nearest.x)
        x = nearest.x + int(self.__max_dist * np.cos(theta))
        y = nearest.y + int(self.__max_dist * np.sin(theta))
        # x = x if x < self.__height else self.__height-1
        # y = y if y < self.__width else self.__width-1
        new_point = (x, y)

        if self.__validate_point(new_point):
            return new_point
        return None

    def rrt(self):
        start = Node(180, 68)
        nodes = [start]
        self.__get_obstacles()

        for _ in range(self.__iterations):
            while True:
                point = self.__generate_point()
                if self.__validate_point(point):
                    break
            nearest = self.__nearest_node(nodes, point)
            if nearest is not None:
                point = self.__approach_point(nearest, point)
                if point is not None:
                    node = Node(point[0], point[1])
                    node.parent = nearest
                    nodes.append(node)
        return nodes

    def plot(self, nodes):
        plt.figure(figsize=(8, 8))
        plt.imshow(self.__map, cmap='Greys', origin='lower')
        for node in nodes:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], color='blue')
        plt.show()

    def stop(self):
        rospy.loginfo("Shutting down the rrt")
