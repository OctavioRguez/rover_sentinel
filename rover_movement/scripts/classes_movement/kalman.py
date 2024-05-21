#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose

# Parent Class
from .rover import Rover

class Kalman_Filter(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)

        # Kalman filter parameters
        self.__x = np.array([0, 0, 0])
        self.__z = np.array([0, 0, 0])
        self.__C = np.eye(3)
        self.__Q = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self.__P = self.__Q
        self.__R = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])

        self.__odom = Odometry()
        self.__odom.header.frame_id = "map"
        self.__odom.child_frame_id = "base_link"

        self.__pose = Pose()

        self.__odom_pub = rospy.Publisher("/odom/kalman", Odometry, queue_size = 10)
        self.__nav_pub = rospy.Publisher("/kalman_predict/pose/navigation", Pose, queue_size = 10)
        self.__control_pub = rospy.Publisher("/kalman_predict/pose/controller", Pose, queue_size = 10)
        rospy.Subscriber("/odom/slam", Odometry, self.__odom_callback)
        rospy.Subscriber("/kalman_predict/vel", Twist, self.__vel_callback)
        rospy.wait_for_message("/odom/slam", Odometry, timeout = 30)

    def __odom_callback(self, msg:Odometry) -> None:
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    
    def __vel_callback(self, msg:Twist) -> None:
        self._v = msg.linear.x
        self._w = msg.angular.z

    def __predict(self, controlled:bool) -> None:
        dt = self._get_dt()
        self.__pose.position.x = self.__x[0]
        self.__pose.position.y = self.__x[1]
        self.__pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.__x[2]))
        if controlled:
            self.__control_pub.publish(self.__pose)
        else:
            self.__nav_pub.publish(self.__pose)
        rospy.wait_for_message("/kalman_predict/vel", Twist, timeout = 30)
        x_dot = [self._v*np.cos(self.__x[2]), self._v*np.sin(self.__x[2]), self._w]
        self.__x = self.__x + x_dot*dt
        self.__P = self.__P + self.__Q

    def __correct(self) -> None:
        self.__z = np.array([self._states["x"], self._states["y"], self._states["theta"]])
        self.__K = np.dot(np.dot(self.__P, self.__C.T), np.linalg.inv(np.dot(np.dot(self.__C, self.__P), self.__C.T) + self.__R))
        self.__x = self.__x + np.dot(self.__K, (self.__z.T - np.dot(self.__C, self.__x.T))).T
        self.__P = np.dot(np.dot(np.eye(3) - np.dot(self.__K, self.__C), self.__P), (np.eye(3) - np.dot(self.__K, self.__C)).T) + np.dot(np.dot(self.__K, self.__R), self.__K.T)

    def __publish(self) -> None:
        self.__odom.header.stamp = rospy.Time.now()
        self.__odom.pose.pose.position.x = self.__x[0]
        self.__odom.pose.pose.position.y = self.__x[1]
        self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.__x[2]))
        self.__odom_pub.publish(self.__odom)

    def apply_filter(self, controlled:bool) -> None:
        self.__predict(controlled)
        self.__correct()
        self.__publish()