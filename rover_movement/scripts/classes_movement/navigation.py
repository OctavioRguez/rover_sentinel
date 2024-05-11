#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# Import ROS messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rover_slam.msg import Quadrants

# Import Classes
from .rover import Rover

class Rover_Navigation(Rover):
    def __init__(self) -> None:
        # Initialize the rover attributes
        Rover.__init__(self)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self._v, self._w = 0.15, 0.2

        # Flags for rotating
        self.__turn_right = False
        self.__turning = True

        # Current border limits
        self.__x_min, self.__x_max = -2.0, 2.0
        self.__y_min, self.__y_max = -2.0, 2.0

        # Lidar data info
        self.__forward, self.__left, self.__right = [], [], []
        # Simulated lasers for the borders
        self.__front_laser, self.__left_laser, self.__right_laser = float("inf"), float("inf"), float("inf")

        # Publish messages
        self.__velocity = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to lidar data topic
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.Subscriber("/odom", Odometry, self.__odom_callback)
        rospy.Subscriber("/borders", Quadrants, self.__borders_callback)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[144:430]
        self.__right = msg.ranges[717:1004]

    # Callback function for the odometry
    def __odom_callback(self, msg:Odometry) -> None:
        # Get position
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        # Get orientation
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def __borders_callback(self, msg:Quadrants) -> None:
        pass

    def move(self) -> None:
        # Minimum distance from obstacles and borders at each direction
        # self.__compute_lasers()
        min_forward = min(min(self.__forward), self.__front_laser)
        min_left = min(min(self.__left), self.__left_laser)
        min_right = min(min(self.__right), self.__right_laser)

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
        self.__set_vel(0.0, -self._w) if self.__turn_right else self.__set_vel(0.0, self._w)

    def __compute_lasers(self) -> None:
        # Set rover position as array
        rover_point = np.array([self._states["x"], self._states["y"]])
        # Compute the left, right and front points on the borders based on the rover position
        front_point = self.__get_laser_point(self._states["theta"])
        left_point = self.__get_laser_point(self._states["theta"] + np.pi/2)
        right_point = self.__get_laser_point(self._states["theta"] - np.pi/2)
        # Compute the distance from the rover to the borders
        self.__front_laser = np.linalg.norm(rover_point - front_point)
        self.__left_laser = np.linalg.norm(rover_point - left_point)
        self.__right_laser = np.linalg.norm(rover_point - right_point)

    def __get_laser_point(self, theta:float) -> np.array:
        point = np.array([0.0, 0.0])
        theta = self._wrap_to_Pi(theta)
        # Right border
        if -np.pi/4 <= theta <= np.pi/4:
            point[0] = self.__x_max
            point[1] = self._states["y"] + np.tan(theta)*(self.__x_max - self._states["x"])
        # Upper border
        elif np.pi/4 <= theta <= 3*np.pi/4:
            point[0] = self._states["x"] + np.tan(theta)*(self._states["y"] - self.__y_max)
            point[1] = self.__y_max
        # Left border
        elif 3*np.pi/4 <= theta <= -3*np.pi/4:
            point[0] = self.__x_min
            point[1] = self._states["y"] + np.tan(theta)*(self.__x_min - self._states["x"])
        # Lower border
        else:
            point[0] = self._states["x"] + np.tan(theta)*(self._states["y"] - self.__y_min)
            point[1] = self.__y_min
        return point

    def __set_vel(self, v:float, w:float) -> None:
        self.__velocity.linear.x = v
        self.__velocity.angular.z = w

    def stop(self) -> None:
        rospy.loginfo("Stopping rover navigation")
        self.__set_vel(0.0, 0.0)
        self.__vel_pub.publish(self.__velocity)
