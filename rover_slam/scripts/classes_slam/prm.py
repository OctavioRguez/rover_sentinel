#!/usr/bin/env python3

# Python libraries
import rospy
import cv2 as cv
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

class PRM:
    def __init__(self) -> None:
        self.__map = cv.imread("/home/puzzlebot/map.png", cv.IMREAD_GRAYSCALE)
        self.__map_original = self.__map.copy()
        self.__width, self.__height = self.__map.shape

        self.__safe_dist = rospy.get_param("/planning/safe_dist/value", default = 4)
        self.__interpolation = rospy.get_param("/planning/interpolation/value", default = 20)

        self.__graph = nx.Graph()
        self.__iterations = (self.__width + self.__height) // 2
        self.__max_dist = (self.__width + self.__height) // 24

    def __dist(self, p1:tuple, p2:tuple) -> float:
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def __widen_obstacles(self) -> None:
        # kernel = cv.getStructuringElement(cv.MORPH_RECT, (2, 2))
        # self.__map = cv.morphologyEx(self.__map, cv.MORPH_OPEN, kernel, iterations = 1)
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (self.__safe_dist, self.__safe_dist))
        self.__map = cv.morphologyEx(self.__map, cv.MORPH_DILATE, kernel, iterations = 2)

    def __generate_point(self) -> tuple:
        while True:
            x = np.random.randint(0, self.__height)
            y = np.random.randint(0, self.__width)
            if self.__validate_point((x, y)):
                break
        return x, y

    def __validate_point(self, point:tuple) -> bool:
        # Validate point to be in open space (represented by 0) and far from obstacles
        if self.__map[point[1]][point[0]] != 0:
            return False
        return True

    def __nearest_node(self, point:tuple) -> tuple:
        min_dist = float('inf')
        nearest = None
        for neighbor in self.__graph.nodes():
            dist = self.__dist(point, neighbor)
            if dist < min_dist:
                min_dist = dist
                nearest = neighbor
        return nearest

    def __approach_point(self, nearest:tuple, point:tuple) -> tuple:
        if self.__dist(point, nearest) <= self.__max_dist:
            return point

        # Reallocate the point to a maximum distance from the nearest node
        theta = np.arctan2(point[1] - nearest[1], point[0] - nearest[0])
        x = nearest[0] + int(self.__max_dist * np.cos(theta))
        y = nearest[1] + int(self.__max_dist * np.sin(theta))
        new_point = (x, y)

        if self.__validate_point(new_point):
            return new_point
        return None

    def __add_edges(self, point:tuple, neighbor:tuple) -> None:
        nodes = list(self.__graph.nodes())
        for neighbor in nodes:
            if neighbor != point:
                dist = self.__dist(point, neighbor)
                if dist <= self.__max_dist:
                    # Check if the edge intersects with anything aside from open space
                    interpolation = np.linspace(point, neighbor, self.__interpolation)
                    if any(self.__map[int(y)][int(x)] != 0 for (x, y) in interpolation):
                        continue
                    self.__graph.add_edge(point, neighbor, weight = dist)

    def calculate_prm(self) -> None:
        self.__widen_obstacles()
        # Add an initial point to the graph
        point = self.__generate_point()
        self.__graph.add_node(point)

        for i in range(self.__iterations):
            rospy.loginfo(i)
            point = self.__generate_point()
            neighbor = self.__nearest_node(point)
            point = self.__approach_point(neighbor, point)
            if point is not None:
                self.__add_edges(point, neighbor)
        return self.__graph, self.__map.shape

    # Plot the map, RRT points and Dijkstra path
    def plot(self, path:list) -> None:
        plt.figure(figsize = (8, 8))
        plt.imshow(self.__map_original, cmap = 'Greys', origin = 'lower')
        for edge in self.__graph.edges():
            plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], color='blue')
        if path is not None:
            x, y = zip(*path)
            plt.plot(x, y, color = 'red', linewidth = 2)
        plt.show()

    def stop(self) -> None:
        rospy.loginfo("Shutting down the PRM")
