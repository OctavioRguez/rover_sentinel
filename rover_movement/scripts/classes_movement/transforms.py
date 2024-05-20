#!/usr/bin/env python3

# Python libraries
import rospy
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# ROS messages
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry

# Parent Class
from .rover import Rover

class Transform(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)

        self.__tf = TransformStamped()
        self.__tf.header.frame_id = "odom"
        self.__tf.child_frame_id = "base_link"

        self.__tf_broadcaster = TransformBroadcaster()
        rospy.Subscriber("/odom/raw", Odometry, self.__callback_odom)
        rospy.wait_for_message("/odom/raw", Odometry, timeout = 30)

    def __callback_odom(self, msg:Odometry) -> None:
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def update_transform(self) -> None:
        self.__tf.header.stamp = rospy.Time.now()
        self.__tf.transform.translation.x = -self._states["x"]
        self.__tf.transform.translation.y = -self._states["y"]
        self.__tf.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, self._states["theta"]))
        self.__tf_broadcaster.sendTransform(self.__tf)
