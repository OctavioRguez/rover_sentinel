#!/usr/bin/env python3

# Import python libraries
import rospy
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

class PRM:
    def __init__(self) -> None:
        # Initialize variables
        self.__map = np.asarray(Image.open("/home/puzzlebot/map.png"))
        self.__width, self.__height = self.__map.shape

        # Points parameters
        self.__iterations = rospy.get_param("/planning/iterations/value", default = 1000)
        self.__max_dist = rospy.get_param("/planning/max_dist/value", default = 5)
        self.__safe_dist = rospy.get_param("/planning/safe_dist/value", default = 5)
        self.__interpolation = rospy.get_param("/planning/interpolation/value", default = 30)

        # PRM structures
        self.__graph = nx.Graph()
        self.__obstacles = []

    # Get distance between two points
    def __dist(self, p1:tuple, p2:tuple) -> float:
        return np.linalg.norm(np.array(p1) - np.array(p2))

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

    # Validate point to be in open space and far from obstacles
    def __validate_point(self, point:tuple) -> bool:
        if any(self.__dist(point, obs) < self.__safe_dist for obs in self.__obstacles) or self.__map[point[1]][point[0]] != 0:
            return False
        return True

    # Perform the RRT algorithm
    def calculate_prm(self) -> None:
        self.__get_obstacles()
        for i in range(self.__iterations):
            rospy.loginfo(i)
            point = self.__generate_point()
            self.__graph.add_node(point)
            for neighbor in self.__graph.nodes():
                if neighbor != point:
                    dist = self.__dist(point, neighbor)
                    if dist <= self.__max_dist:
                        interpolation = np.linspace(point, neighbor, self.__interpolation)
                        if any(self.__map[int(y)][int(x)] != 0 for (x, y) in interpolation):
                            continue
                        self.__graph.add_edge(point, neighbor, weight = dist)
        return self.__graph

    # Plot the map, RRT points and path
    def plot(self, path:list) -> None:
        plt.figure(figsize = (8, 8))
        plt.imshow(self.__map, cmap = 'Greys', origin = 'lower')
        for edge in self.__graph.edges():
            plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='blue')
        if path is not None:
            x, y = zip(*path)
            plt.plot(x, y, color = 'red', linewidth = 2)
        plt.show()

    # Stop the algorithm
    def stop(self) -> None:
        rospy.loginfo("Shutting down the PRM")
