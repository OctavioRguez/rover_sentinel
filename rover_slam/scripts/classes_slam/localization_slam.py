#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS messages
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

class Localization_SLAM():
    def __init__(self):
        self.__odom = Odometry()
        self.__odom.header.frame_id = "map"
        self.__odom.child_frame_id = "base_link"
        self.__map_tf = TransformStamped()
        self.__odom_tf = TransformStamped()

        self.__odom_pub = rospy.Publisher("/odom/slam", Odometry, queue_size = 10)
        rospy.Subscriber("/tf", TFMessage, self.__tf_callback)
        rospy.wait_for_message("/tf", TFMessage, timeout = 30)

    def __tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.__map_tf = transform
            elif transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                self.__odom_tf = transform

    def update_pose(self):
        q = self.__map_tf.transform.rotation
        rotation = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        T1 = np.array([[np.cos(rotation), -np.sin(rotation), 0, self.__map_tf.transform.translation.x],
                       [np.sin(rotation), np.cos(rotation), 0, self.__map_tf.transform.translation.y],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        
        q = self.__odom_tf.transform.rotation
        rotation = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        T2 = np.array([[np.cos(rotation), -np.sin(rotation), 0, self.__odom_tf.transform.translation.x],
                       [np.sin(rotation), np.cos(rotation), 0, self.__odom_tf.transform.translation.y],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        T = np.dot(T1, T2)
        self.__odom.pose.pose.position.x = T[0, 3]
        self.__odom.pose.pose.position.y = T[1, 3]
        self.__odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, np.arctan2(T[1, 0], T[0, 0])))

        self.__odom.header.stamp = rospy.Time.now()
        self.__odom_pub.publish(self.__odom)

    def stop(self):
        rospy.loginfo("Shutting down the Slam Position Publisher")
