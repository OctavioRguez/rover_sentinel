#!/usr/bin/env python3

# Python libraries
import rospy
import networkx as nx
import numpy as np

# ROS messages
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry, Path
from rover_slam.msg import Border

class Dijkstra_Path:
    def __init__(self, graph:nx.Graph, shape:tuple) -> None:
        self.__graph = graph
        self.__start = (0, 0)
        self.__width, self.__height = shape

        self.__path_pub = rospy.Publisher("/path", Path, queue_size = 10)
        rospy.Subscriber("/odom/raw", Odometry, self.__odom_callback)
        rospy.wait_for_message("/odom/raw", Odometry, timeout = 30)

    def __odom_callback(self, msg) -> None:
        self.__start = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def __dist(self, p1:tuple, p2:tuple) -> float:
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def calculate_dijkstra(self, border:Border) -> None:
        # Approximate for a start node
        start_node = min(self.__graph.nodes(), key = lambda x: self.__dist(self.__start, x))
        while True:
            # Execute dijkstra until a valid path is found
            try:
                while True:
                    goal_node = list(self.__graph.nodes())[np.random.randint(0, len(self.__graph.nodes()))]
                    if border.upper.x <= goal_node[0] <= border.lower.x and border.upper.y <= goal_node[1] <= border.lower.y:
                        break
                path = nx.dijkstra_path(self.__graph, start_node, goal_node)
                break
            except nx.NetworkXNoPath:
                print("No path found, retrying...")

        print(start_node, goal_node)
        self.__path_pub.publish(Path(poses=[PoseStamped(pose=Pose(position=
                Point(x=(pos[0]-self.__width/2)/40, y=(pos[1]-self.__height/2)/40))) for pos in path]))
        return path
