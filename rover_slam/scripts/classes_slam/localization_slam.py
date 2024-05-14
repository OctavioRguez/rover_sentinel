#!/usr/bin/env python3

# Python libraries
import rospy
from tf2_ros import Buffer

# ROS messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Localization_SLAM():
    def __init__(self):
        self.__odom = Odometry()
        self.__map_pose = PoseStamped()
        self.__map_pose.header.frame_id = "map"

        self.__buffer = Buffer()
        self.__odom_pub = rospy.Publisher("/odom/slam", Odometry, queue_size = 10)

    def update_pose(self):
        tf = self.__buffer.lookup_transform("map", "odom", rospy.Time(0))

        self.__map_pose.header.stamp = rospy.Time.now()
        self.__map_pose.pose.position = tf.transform.translation
        self.__map_pose.pose.orientation = tf.transform.rotation
        slam_pose = self.__buffer.transform(self.__map_pose, "base_link")

        self.__odom.header.stamp = rospy.Time.now()
        self.__odom.pose.pose = slam_pose.pose
        self.__odom_pub.publish(self.__odom)

    def stop(self):
        rospy.loginfo("Shutting down the Slam Position Publisher")
