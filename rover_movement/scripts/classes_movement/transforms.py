#!/usr/bin/env python3

# Import python libraries
import rospy
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import ROS messages
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry

# Import Classes
from .rover import Rover

class Transform(Rover):
    def __init__(self) -> None:
        # Initialize the rover attributes
        Rover.__init__(self)

        # Declare the publish messages
        self.__tf = TransformStamped()
        self.__tf.header.frame_id = "odom"
        self.__tf.child_frame_id = "base_link"

        # Setup the publishers
        self.__tf_broadcaster = TransformBroadcaster()

        # Setup the subscribers
        rospy.Subscriber("/odom", Odometry, self.__callback_odom)

    # Callback function for the odometry
    def __callback_odom(self, msg:Odometry) -> None:
        # Get position
        self._states["x"] = msg.pose.pose.position.x
        self._states["y"] = msg.pose.pose.position.y
        # Get orientation
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    # Update the transform
    def update_transform(self) -> None:
        # Set the transform header (time)
        self.__tf.header.stamp = rospy.Time.now()
        # Set the transform translation and rotation
        self.__tf.transform.translation.x = self._states["x"]
        self.__tf.transform.translation.y = self._states["y"]
        self.__tf.transform.rotation = Quaternion(*quaternion_from_euler(0, 0, self._states["theta"]))
        # Publish the transform
        self.__tf_broadcaster.sendTransform(self.__tf)
