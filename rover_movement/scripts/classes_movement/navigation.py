#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rover_slam.msg import Borders

# Parent Class
from .rover import Rover

class Rover_Navigation(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self._v, self._w = 0.2, 0.2
        self.__turn_right = False
        self.__turning = True

        # Border limits
        self.__x_min, self.__x_max = -2.0, 2.0
        self.__y_min, self.__y_max = -2.0, 2.0

        # Lidar data
        self.__forward, self.__left, self.__right = [], [], []
        self.__front_laser, self.__left_laser, self.__right_laser = float("inf"), float("inf"), float("inf")

        self.__velocity = Twist()

        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.Subscriber("/odom/raw", Odometry, self.__odom_callback)
        rospy.Subscriber("/curr_border", Borders, self.__borders_callback)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[144:430]
        self.__right = msg.ranges[717:1004]

    def __odom_callback(self, msg:Odometry) -> None:
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def __borders_callback(self, msg:Borders) -> None:
        self.__x_min, self.__x_max = msg.upper.x, msg.lower.x
        self.__y_min, self.__y_max = msg.upper.y, msg.lower.y

    def move(self) -> None:
        self.__compute_lasers()
        min_forward = min(min(self.__forward), self.__front_laser)
        min_left = min(min(self.__left), self.__left_laser)
        min_right = min(min(self.__right), self.__right_laser)

        # No obstacles in any direction
        if min_forward > self._safe_distance:
            self.__turning = True
            self.__set_vel(self._v, 0.0)
        # Obstacles in all directions
        elif self.__turning:
            if all(dist < self._safe_distance for dist in [min_forward, min_left, min_right]):
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
            self.__turning = False
        self.__vel_pub.publish(self.__velocity)

    def __obstacle_forward(self, left:float, right:float) -> None:
        if self.__turning:
            self.__turn_right = True if right >= left else False
        # Rotate to the direction with the higher distance
        self.__set_vel(0.0, -self._w) if self.__turn_right else self.__set_vel(0.0, self._w)

    def __compute_lasers(self) -> None:
        rover_point = np.array([self._states["x"], self._states["y"]])
        # Points from the lasers colliding with the borders
        front_point = self.__get_laser_point(self._states["theta"])
        left_point = self.__get_laser_point(self._states["theta"] + np.pi/2)
        right_point = self.__get_laser_point(self._states["theta"] - np.pi/2)
        # Distances from the rover to the points
        self.__front_laser = np.linalg.norm(rover_point - front_point)
        self.__left_laser = np.linalg.norm(rover_point - left_point)
        self.__right_laser = np.linalg.norm(rover_point - right_point)

    def __get_laser_point(self, theta:float) -> np.array:
        point = np.array([0.0, 0.0])
        theta = theta+2*np.pi if theta < 0 else theta # Set theta from 0 to 2*pi
        # Right border
        if theta <= np.pi/4:
            point[0] = self.__x_max
            point[1] = self._states["y"] + np.tan(theta)*(self.__x_max - self._states["x"])
        # Upper border
        elif theta <= 3*np.pi/4:
            point[0] = self._states["x"] + np.tan(np.pi/2 - theta)*(self._states["y"] - self.__y_max)
            point[1] = self.__y_max
        # Left border
        elif theta <= 5*np.pi/8:
            point[0] = self.__x_min
            point[1] = self._states["y"] + np.tan(np.pi - theta)*(self.__x_min - self._states["x"])
        # Lower border
        else:
            point[0] = self._states["x"] + np.tan(3*np.pi/2 - theta)*(self._states["y"] - self.__y_min)
            point[1] = self.__y_min
        return point

    def __set_vel(self, v:float, w:float) -> None:
        self.__velocity.linear.x = v
        self.__velocity.angular.z = w

    def stop(self) -> None:
        rospy.loginfo("Stopping rover navigation")
        self.__set_vel(0.0, 0.0)
        self.__vel_pub.publish(self.__velocity)
