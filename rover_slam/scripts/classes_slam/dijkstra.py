#!/usr/bin/env python3

# Python libraries
import rospy
import networkx as nx
import numpy as np

# ROS messages
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry, Path

class Dijkstra_Path:
    def __init__(self, graph:nx.Graph) -> None:
        self.__graph = graph
        # self.__start = (0, 0)

        self.__path_pub = rospy.Publisher("/path", Path, queue_size = 10)
        rospy.Subscriber("/odom_slam", Odometry, self.__odom_callback)
        rospy.Subscriber("/get_path", Bool, self.__get_path_callback)

    def __odom_callback(self, msg):
        self.__start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def __get_path_callback(self, msg):
        if msg.data:
            self.calculate_dijkstra()

    def calculate_dijkstra(self):
        # Approximate for a start node
        start_node = min(self.__graph.nodes(), key = lambda x: self.__dist(self.__start, x))
        while True:
            # Execute dijkstra until a valid path is found
            try:
                goal_node = list(self.__graph.nodes())[np.random.randint(0, len(self.__graph.nodes()))]
                path = nx.dijkstra_path(self.__graph, start_node, goal_node)
                break
            except nx.NetworkXNoPath:
                print("No path found, retrying...")

        rospy.loginfo(start_node, goal_node)
        self.__path_pub.publish(Path(poses=(PoseStamped(pose=Pose(position=Point(x=pos[0], y=pos[1]))) for pos in path)))
        return path
