#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# ROS messages
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rover_slam.msg import Border

# Parent Class
from .rover import Rover

class Rover_Navigation(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self._v, self._w = 0.2, 0.2
        self.__turning = True
        self.__turning_kalman = True
        self.__w_past, self.__w_past_kalman = 0.0, 0.0

        # Border limits
        self.__x_min, self.__x_max = -100.0, 100.0
        self.__y_min, self.__y_max = -100.0, 100.0

        # Lidar data
        self.__forward, self.__left, self.__right = [], [], []
        self.__dist = float("inf")

        self.__velocity = Twist()
        self.__vel_kalman = Twist()

        self.__kalman_pub = rospy.Publisher("/kalman_predict/vel", Twist, queue_size = 1)
        rospy.Subscriber("/kalman_predict/pose/navigation", Pose, self.__kalman_callback)

        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.Subscriber("/odom/raw", Odometry, self.__odom_callback)
        rospy.Subscriber("/sensor/distance", Float32, self.__distance_callback)
        rospy.Subscriber("/curr_border", Border, self.__borders_callback)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    def __kalman_callback(self, msg:Pose) -> None:
        q = msg.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        v, w, self.__turning_kalman = self.__avoid_obstacles(*self.__compute_lasers(msg.position.x, msg.position.y, theta), self.__turning_kalman)
        w = self.__w_past_kalman if w is None else w
        self.__w_past_kalman = w
        self.__vel_kalman.linear.x = v
        self.__vel_kalman.angular.z = w
        self.__kalman_pub.publish(self.__vel_kalman)

    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[144:430]
        self.__right = msg.ranges[717:1004]

    def __distance_callback(self, msg:Float32) -> None:
        self.__dist = msg.data

    def __odom_callback(self, msg:Odometry) -> None:
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def __borders_callback(self, msg:Border) -> None:
        self.__x_min, self.__x_max = msg.upper.x, msg.lower.x
        self.__y_min, self.__y_max = msg.upper.y, msg.lower.y

    def move(self) -> None:
        v, w, self.__turning = self.__avoid_obstacles(*self.__compute_lasers(self._states["x"], self._states["y"], self._states["theta"]), self.__turning)
        w = self.__w_past if w is None else w
        self.__w_past = w
        self.__set_vel(v, w)
        self.__vel_pub.publish(self.__velocity)
    
    def __avoid_obstacles(self, front_laser, left_laser, right_laser, turning) -> tuple:
        min_forward = min(min(self.__forward), front_laser, self.__dist)
        min_left = min(min(self.__left), left_laser)
        min_right = min(min(self.__right), right_laser)

        v, w = 0.0, None
        # No obstacles in any direction
        if min_forward > self._safe_distance:
            turning = True
            v, w = self._v, 0.0
        elif turning:
            turning = False
            # Obstacles in all directions
            if all(dist < self._safe_distance for dist in [min_forward, min_left, min_right]):
                w = -self._w
            # Obstacles at the left
            elif min_left < self._safe_distance:
                w = -self._w
            # Obstacles at the right
            elif min_right < self._safe_distance:
                w =  self._w
            # Obstacles at the front
            else:
                # Rotate to the direction with the higher distance
                w = -self._w if min_right >= min_left else self._w
        return v, w, turning

    def __compute_lasers(self, x:float, y:float, theta:float) -> tuple:
        rover_point = np.array([x, y])
        # Points from the lasers colliding with the borders
        front_points = self.__get_laser_points(x, y, theta)
        left_points = self.__get_laser_points(x, y, theta + np.pi/2)
        right_points = self.__get_laser_points(x, y, theta - np.pi/2)
        # Distances from the rover to the points
        front_laser = min(np.linalg.norm(rover_point - point) for point in front_points)
        left_laser = min(np.linalg.norm(rover_point - point) for point in left_points)
        right_laser = min(np.linalg.norm(rover_point - point) for point in right_points)
        return front_laser, left_laser, right_laser

    def __get_laser_points(self, x:float, y:float, theta:float) -> np.array:
        points = []
        lasers = np.linspace(theta-np.pi/4, theta+np.pi/4, 10)
        for laser in lasers:
            point = np.array([0.0, 0.0])
            laser = laser % (2*np.pi)
            # Right border
            if laser < np.pi/4 or laser >= 7*np.pi/4:
                point[0] = self.__x_max
                point[1] = y + (self.__x_max - x) * np.tan(laser)
            # Upper border
            elif laser < 3*np.pi/4:
                point[0] = x + (y - self.__y_max) / np.tan(laser)
                point[1] = self.__y_max
            # Left border
            elif laser < 5*np.pi/4:
                point[0] = self.__x_min
                point[1] = y + (self.__x_min - x) * np.tan(laser)
            # Lower border
            else:
                point[0] = x + (y - self.__y_min) / np.tan(laser)
                point[1] = self.__y_min
            points.append(point)
        return points

    def __set_vel(self, v:float, w:float) -> None:
        self.__velocity.linear.x = v
        self.__velocity.angular.z = w

    def stop(self) -> None:
        self.__set_vel(0.0, 0.0)
        self.__vel_pub.publish(self.__velocity)
