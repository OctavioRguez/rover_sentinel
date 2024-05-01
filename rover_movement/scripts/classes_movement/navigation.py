#!/usr/bin/env python3

# Import python libraries
import rospy

# Import ROS messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Import Classes
from .rover import Rover

class Rover_Navigation(Rover):
    def __init__(self) -> None:
        # Initialize the rover attributes
        Rover.__init__(self)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self._v, self._w = 0.25, 0.3

        # Flags for rotating
        self.__turn_right = False
        self.__turning = True

        # Lidar data info
        self.__forward, self.__left, self.__right = [], [], []

        # Publish messages
        self.__velocity = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to lidar data topic
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[144:430]
        self.__right = msg.ranges[717:1004]

    def move(self) -> None:
        # Minimum distance from obstacles at each direction
        min_forward, min_left, min_right = [min(self.__forward), min(self.__left), min(self.__right)]

        # No obstacles in any direction
        if min_forward > self._safe_distance:
            self.__turning = True
            self.__set_vel(self._v, 0.0)
        # Obstacles in all directions
        elif all(dist < self._safe_distance for dist in [min_forward, min_left, min_right]):
            self.__set_vel(0.0, -self._w)
        # Obstacles in both left and right sides
        elif abs(min_left - min_right) < self._safe_distance:
            self.__set_vel(0.0, self._w)
        # Obstacles at the left
        elif min_left < self._safe_distance:
            self.__set_vel(0.0, -self._w)
        # Obstacles at the right
        elif min_right < self._safe_distance:
            self.__set_vel(0.0, self._w)
        # Obstacles at the front
        else:
            self.__obstacle_forward(min_left, min_right)

        # Publish the velocity
        self.__vel_pub.publish(self.__velocity)

    def __obstacle_forward(self, left:float, right:float) -> None:
        if self.__turning:
            self.__turn_right = True if right >= left else False
            self.__turning = False
        
        # Rotate to the direction with the higher distance
        self.__set_vel(0.0, -self.__w) if self.__turn_right else self.__set_vel(0.0, self.__w)

    def __set_vel(self, v:float, w:float) -> None:
        self.__velocity.linear.x = v
        self.__velocity.angular.z = w

    def stop(self) -> None:
        rospy.loginfo("Stopping rover navigation")
        self.__set_vel(0.0, 0.0)
        self.__vel_pub.publish(self.__velocity)
