#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from collections import deque
from tf.transformations import quaternion_from_euler

# ROS messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# Parent Class
from .rover import Rover

class Localization(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)

        # Right and left wheel velocities (rad/s)
        self.__wr, self.__wl = 0.0, 0.0
        self.__queue_wr = deque(maxlen = 5)
        self.__queue_wl = deque(maxlen = 5)

        self.__odom = Odometry()
        self.__odom.header.frame_id = "odom"
        self.__odom.child_frame_id = "base_link"

        self.__odom_pub = rospy.Publisher("/odom/raw", Odometry, queue_size = 10)
        rospy.Subscriber("/wr", Float32, self.__wr_callback)
        rospy.Subscriber("/wl", Float32, self.__wl_callback)
        rospy.wait_for_message("/wr", Float32, timeout = 30)
        rospy.wait_for_message("/wl", Float32, timeout = 30)

    def __wr_callback(self, msg:Float32) -> None:
        self.__queue_wr.append(msg.data)
        self.__wr = np.median(self.__queue_wr)

    def __wl_callback(self, msg:Float32):
        self.__queue_wl.append(msg.data)
        self.__wl = np.median(self.__queue_wl)

    def update_odometry(self) -> None:
        dt = self._get_dt()

        self._v = self._r * (self.__wr + self.__wl) / 2.0
        self._w = self._r * (self.__wr - self.__wl) / self._l

        # Perform 4th order Runge-Kutta integration
        k1 = self.__kinematics(self._states["theta"])
        k2 = self.__kinematics(self._states["theta"] + 0.5 * dt * k1[2])
        k3 = self.__kinematics(self._states["theta"] + 0.5 * dt * k2[2])
        k4 = self.__kinematics(self._states["theta"] + dt * k3[2])
        
        self._states["x"] += (dt / 6) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0])
        self._states["y"] += (dt / 6) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1])
        self._states["theta"] += (dt / 6) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2])

        self.__publish_odometry()

    def __kinematics(self, theta:float) -> tuple:
        vx = self._v * np.cos(theta)
        vy = self._v * np.sin(theta)
        return vx, vy, self._w

    def __publish_odometry(self) -> None:
        self.__odom.header.stamp = rospy.Time.now()
        self.__odom.pose.pose.position.x = self._states["x"]
        self.__odom.pose.pose.position.y = self._states["y"]
        self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self._states["theta"]))
        self.__odom_pub.publish(self.__odom)
