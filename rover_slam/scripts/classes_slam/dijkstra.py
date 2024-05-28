#!/usr/bin/env python3

# Python libraries
import rospy
import networkx as nx
import numpy as np
from PIL import Image

# ROS messages
from geometry_msgs.msg import PoseStamped, Pose, Point, Point32
from nav_msgs.msg import Odometry, Path
from rover_slam.msg import Border

class Dijkstra_Path:
    def __init__(self, graph:nx.Graph, shape:tuple) -> None:
        self.__graph = graph
        self.__width, self.__height = shape
        self.__start = (0, 0)
        self.__end = (0, 0)

        self.__path_pub = rospy.Publisher("/path", Path, queue_size = 10)
        rospy.Subscriber("/odom/raw", Odometry, self.__odom_callback)

    def __odom_callback(self, msg:Odometry) -> None:
        self.__start = (msg.pose.pose.position.x*40+self.__height/2, -(msg.pose.pose.position.y)*40+self.__width/2)
        
    def __dist(self, p1:tuple, p2:tuple) -> float:
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def calculate_dijkstra(self, border:Border, offset:Point32) -> list:
        offset = Point32(0, 0, 0) if offset is None else offset
        # Get start and end poses
        self.__start = (self.__width/2, self.__height/2)
        self.__start = (self.__start[0] - offset.x, self.__start[1] - offset.y)
        self.__end = (((border.upper.x+border.lower.x)/2)*40+self.__width/2, ((border.upper.y+border.lower.y)/2)*40+self.__height/2)
        self.__end = (self.__end[0] - offset.x, self.__end[1] - offset.y)
        # Approximate for a start and end node
        start_node = min(self.__graph.nodes(), key = lambda x: self.__dist(self.__start, x))
        goal_node = min(self.__graph.nodes(), key = lambda x: self.__dist(self.__end, x))

        path = nx.dijkstra_path(self.__graph, start_node, goal_node)
        # Transform to meters
        path = [((pos[0]+offset.x-self.__width/2)/40, -(pos[1]+offset.y-self.__height/2)/40) for pos in path]
        self.__path_pub.publish(Path(poses=[PoseStamped(pose=Pose(position=Point(x=pos[0], y=pos[1]))) for pos in path]))
        return path
