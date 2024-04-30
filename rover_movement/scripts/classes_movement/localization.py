#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from collections import deque
from tf.transformations import quaternion_from_euler

# Import ROS messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# Import Classes
from .rover import Rover

class Localization(Rover):
    def __init__(self):
        # Initialize the rover attributes
        Rover.__init__(self)

        # Initial wheel velocities
        self.__wr, self.__wl = 0.0, 0.0

        # Median filter
        self.__queue_wr = deque(maxlen = 30)
        self.__queue_wl = deque(maxlen = 30)

        # Declare the publish messagess
        self.__odom = Odometry()
        self.__odom.header.frame_id = "odom"
        self.__odom.child_frame_id = "base_link"

        # Publisher for odometry
        self.__odom_pub = rospy.Publisher("/odom", Odometry, queue_size = 10)
        
        # Subscribe to wheel encoder topics
        rospy.Subscriber("/wr", Float32, self.__wr_callback)
        rospy.Subscriber("/wl", Float32, self.__wl_callback)

    # Callback for the right wheel velocity
    def __wr_callback(self, msg):
        self.__queue_wr.append(msg.data)
        self.__wr = np.mean(self.__queue_wr)

    # Callback for the left wheel velocity
    def __wl_callback(self, msg):
        self.__queue_wl.append(msg.data)
        self.__wl = np.mean(self.__queue_wl)

    # Solve the odometry equations
    def update_odometry(self):
        # Get the time step
        dt = self._get_dt()

        # Compute odometry
        self._v = self._r * (self.__wr + self.__wl) / 2.0
        self._w = self._r * (self.__wr - self.__wl) / self._l

        # Update states
        self._states["theta"] = self._wrap_to_Pi(self._states["theta"] + self._w*dt)
        self._states["x"] += self._v * np.cos(self._states["theta"]) * dt
        self._states["y"] += self._v * np.sin(self._states["theta"]) * dt

        # Publish odometry message
        self.__publish_odometry()

    def __publish_odometry(self):
        # Set the header
        self.__odom.header.stamp = rospy.Time.now()

        # Set the position
        self.__odom.pose.pose.position.x = self._states["x"]
        self.__odom.pose.pose.position.y = self._states["y"]

        # Set the orientation
        self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self._states["theta"]))

        # Set the velocities
        self.__odom.twist.twist.linear.x = self._v
        self.__odom.twist.twist.angular.z = self._w

        # Publish the message
        self.__odom_pub.publish(self.__odom)

    def stop(self):
        rospy.loginfo("Stopping localization")