#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Import ROS messages
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class Localization():
    def __init__(self):
        # Tranform created by SLAM
        self.__transform = TransformStamped()

        # Odometry message
        self.__odom = Odometry()
        self.__odom.header.frame_id = "map"
        self.__odom.child_frame_id = "odom"

        # Publisher for odometry
        self.__odom_pub = rospy.Publisher("/odom_slam", Odometry, queue_size = 10)

        # Subscribe to topics
        rospy.Subscriber("/tf", TFMessage, self.__transform_callback)
        rospy.Subscriber("/odom", Odometry, self.__odom_callback)
        rospy.wait_for_message("/tf", TFMessage, timeout = 30)
        rospy.wait_for_message("/odom", TFMessage, timeout = 30)

    def __transform_callback(self, msg:TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id == "odom" and transform.header.frame_id == "map":
                self.__transform = transform

    def __odom_callback(self, msg:Odometry):
        # Extract the data from the odometry message
        self.__odom.pose.pose.position = msg.pose.pose.position
        self.__odom.pose.pose.orientation = msg.pose.pose.orientation

    def update_odom(self):
        # Update the linear position (x, y)
        self.__odom.pose.pose.position.x = self.__transform.transform.translation.x
        self.__odom.pose.pose.position.y = self.__transform.transform.translation.y

        # Update the angular position (z)
        # q = self.__odom.pose.pose.orientation
        # angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] # Odom angle

        self.__odom.pose.pose.orientation = self.__transform.transform.rotation
        # angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] # Add transform angle

        # Wrap angle to [-pi, pi] and transform to Quaternion
        # angle = (angle + np.pi) % (2 * np.pi) - np.pi
        # self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, angle))

        # Publish the odometry message
        self.__odom.header.stamp = rospy.Time.now()
        self.__odom_pub.publish(self.__odom)

    def stop(self):
        rospy.loginfo("Shutting down the Slam Position Publisher")
