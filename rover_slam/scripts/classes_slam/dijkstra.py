#!/usr/bin/env python3

# Import python libraries
import rospy
import heapq
import numpy as np
from .rrt import Node

# Import ROS messages
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry, Path

class Dijkstra_Path:
    def __init__(self, nodes:list) -> None:
        # Initialized variables
        self.__nodes = nodes
        self.__start = (0.0, 0.0)

        # Publisher for generated path
        self.__path_pub = rospy.Publisher("/path", Path, queue_size = 10)

        # Subscribe to the odometry and scan topics
        rospy.Subscriber("/odom_slam", Odometry, self.__odom_callback)
        rospy.Subscriber("/plan", Bool, self.__plan_callback)

    def __odom_callback(self, msg):
        # Get current robot position
        self.__start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def __plan_callback(self, msg):
        if msg.data:
            self.dijkstra()

    def dijkstra(self):
        # Generate random goal point
        goal_node = self.__nodes[np.random.randint(0, len(self.__nodes))]
        # Approximate for a start node
        start_node = min(self.__nodes, key = lambda x: self.__dist(x, Node(self.__start[0], self.__start[1])))
        start_node.dist = 0

        # Initialize priority queue
        pq = [start_node]
        while pq:
            # Pop node with smallest distance from priority queue
            node = heapq.heappop(pq)
            
            # If current node is the goal, return the path
            if node == goal_node:
                return self.__reconstruct_path(start_node, goal_node)
            
            # Iterate through neighbors of current node
            for neighbor in self.__get_neighbors(node):
                dist = node.dist + self.__dist(node, neighbor)
                
                # If new distance is smaller, update distance and parent
                if dist < neighbor.dist:
                    neighbor.dist = dist
                    heapq.heappush(pq, neighbor)
        return None

    # Get neighboring nodes for a current node
    def __get_neighbors(self, node:Node) -> list:
        neighbors = []
        for other_node in self.__nodes:
            if other_node != node:
                neighbors.append(other_node)
        return neighbors

    # Calculate distance between two nodes
    def __dist(self, node1:Node, node2:Node) -> float:
        return np.linalg.norm(np.array((node1.x, node1.y)) - np.array((node2.x, node2.y)))

    # Reconstruct the path from start to goal
    def __reconstruct_path(self, start_node:Node, goal_node:Node) -> list:
        current_node = goal_node
        path = [current_node]
        while current_node != start_node:
            current_node = current_node.parent
            path.append(current_node)
        path = list(reversed(path))
        self.__path_pub.publish(Path(PoseStamped(pose=Pose(position=Point(x=pos.x, y=pos.y))) for pos in path))
        return path
