#!/usr/bin/env python3

# Import python libraries
import rospy

# Import ROS messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Rover_Navigation:
    def __init__(self) -> None:
        # Count to avoid jerking
        self.__right_count, self.__left_count = 0, 0
        # Safe distance from obstacles at any direction (m)
        self.__safe_distance = rospy.get_param('/rover/safe_distance/value', default = 0.3)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self.__v, self.__w = 0.25, 0.3

        # Lidar data info
        self.__forward, self.__left, self.__right = [], [], []

        # Publish messages
        self.__velocity = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to lidar data topic
        rospy.Subscriber("/scan", LaserScan, self.__scanCallback)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    def __scanCallback(self, msg:LaserScan) -> None:
        self.__forward = msg.data[0:144] + msg.data[1004:1147]
        self.__left = msg.data[275:350]
        self.__right = msg.data[800:925]

    def move(self):
        # Minimum distance from obstacles at each direction
        min_forward, min_left, min_right = [min(self.__forward), min(self.__left), min(self.__right)]

        # Obstacles in all directions
        if all(dist < self.__safe_distance for dist in [min_forward, min_left, min_right]):
            self.__set_vel(0.0, -self.__w)
        # Obstacles at the front
        elif min_forward < self.__safe_distance:
            self.__obstacle_forward()
        # Obstacles at the left
        elif min_left < self.__safe_distance:
            self.__obstacle_left()
        # Obstacles at the right
        elif min_right < self.__safe_distance:
            self.__obstacle_right()
        # No obstacles in any direction
        else:
            self.__set_vel(self.__v, 0.0)

        # Publish the velocity
        self.__vel_pub.publish(self.__velocity)

    def __obstacle_forward(self) -> None:
        # Maximum distances from obstacles at left and right directions
        max_left, max_right = [max(self.__left), max(self.__right)]
        
        # Rotate to the direction with the higher distance
        if max_right > max_left:
            self.__set_vel(0.0, -self.__w)
        else:
            self.__set_vel(0.0, self.__w)

    def __obstacle_left(self) -> None:
        self.__right_count = 0
        self.__left_count += 1
        # Turn is taken after 2 counts, to avoid jerking
        if self.__left_count >= 2:
            self.__set_vel(0.0, -self.__w)
    
    def __obstacle_right(self) -> None:
        self.__right_count += 1
        self.__left_count = 0
        # Turn is taken after 2 counts, to avoid jerking
        if self.__right_count >= 2:
            self.__set_vel(0.0, self.__w)

    def __set_vel(self, v:float, w:float) -> None:
        self.__velocity.linear.x = v
        self.__velocity.angular.z = w

    def stop(self) -> None:
        rospy.loginfo("Stopping rover navigation")
        self.__publishVel(0.0, 0.0)