#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# Import ROS messages
from geometry_msgs.msg import Twist, Polygon
from nav_msgs.msg import Odometry

# Import Classes
from .Puzzlebot import Puzzlebot

class Controller(Puzzlebot):
    def __init__(self):
        # Initialize the puzzlebot attributes
        Puzzlebot.__init__(self)

        # Initialize variables
        self.__set_point = []
        self.__vmax, self.__wmax = 0.15, 0.5
        self.__kp = 5*np.eye(2)
        self.__i = 0

        # Noise parameters
        self.__mean = 0.0
        self.__std = 1.8

        # Declare the publish messagess
        self.__vel = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to the odometry and set_point topics
        rospy.Subscriber("/odom", Odometry, self.__callback_odom)
        rospy.Subscriber("/set_point", Polygon, self.__set_point_callback)

    # Callback function for the odometry
    def __callback_odom(self, msg):
        # Get position
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        # Get orientation
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    # Callback function for the set point
    def __set_point_callback(self, msg):
        self.__set_point = msg.points

    def control(self):
        # Skip if the robot is not moving
        if not self.__set_point or self.__i >= len(self.__set_point):
            self.__i = 0
            return
        
        # Setup vectors
        q = np.array([self._states["x"], self._states["y"]])
        qd = np.array([self.__set_point[self.__i].x, self.__set_point[self.__i].y])
        
        # Control
        err = qd - q
        D = np.array([
            [(self._r / 2 * np.cos(self._states["theta"]) - self._h*self._r / self._l * np.sin(self._states["theta"])),
             (self._r / 2 * np.cos(self._states["theta"]) + self._h*self._r / self._l * np.sin(self._states["theta"]))],
            [(self._r / 2 * np.sin(self._states["theta"]) + self._h*self._r / self._l * np.cos(self._states["theta"])), 
             (self._r / 2 * np.sin(self._states["theta"]) - self._h*self._r / self._l * np.cos(self._states["theta"]))]
        ])
        u = np.dot(np.linalg.inv(D), np.dot(self.__kp, err))
        q_dot = np.dot(D, u) + np.random.normal(self.__mean, self.__std)
        theta_dot = np.dot(np.array([[self._r / self._l, -self._r / self._l]]), u) + np.random.normal(self.__mean, self.__std)

        # Check if the robot has reached the set point
        self.__i = self.__i + 1 if np.linalg.norm(err) < 0.05 else self.__i
        
        # Publish the control input
        self.__vel.linear.x = self.__vmax*np.tanh(np.sqrt(q_dot[0]**2 + q_dot[1]**2) / self.__vmax)
        self.__vel.angular.z = self.__wmax*np.tanh(theta_dot[0] / self.__wmax)
        self.__vel_pub.publish(self.__vel)