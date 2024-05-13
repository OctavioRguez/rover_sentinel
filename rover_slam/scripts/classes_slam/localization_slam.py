#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS messages
from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class Localization():
    def __init__(self):
        self.__transform = TransformStamped()
        self.__pose = Pose()

        self.__pose_pub = rospy.Publisher("/pose/slam", Pose, queue_size = 10)
        rospy.Subscriber("/tf", TFMessage, self.__transform_callback)
        rospy.Subscriber("/odom", Odometry, self.__odom_callback)
        rospy.wait_for_message("/tf", TFMessage, timeout = 30)
        rospy.wait_for_message("/odom", TFMessage, timeout = 30)

    def __transform_callback(self, msg:TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id == "odom" and transform.header.frame_id == "map":
                self.__transform = transform

    def __odom_callback(self, msg:Odometry):
        # self.__odom.pose.pose.position = msg.pose.pose.position
        # self.__odom.pose.pose.orientation = msg.pose.pose.orientation
        pass

    def update_pose(self):
        self.__pose.position = self.__transform.transform.translation
        self.__pose.orientation = self.__transform.transform.rotation

        # q = self.__odom.pose.pose.orientation
        # angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] # Odom angle
        # q = self.__transform.transform.rotation
        # angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] # Add transform angle

        # Wrap angle to [-pi, pi] and transform to Quaternion
        # angle = (angle + np.pi) % (2 * np.pi) - np.pi
        # self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, angle))
        self.__pose_pub.publish(self.__pose)

    def stop(self):
        rospy.loginfo("Shutting down the Slam Position Publisher")
