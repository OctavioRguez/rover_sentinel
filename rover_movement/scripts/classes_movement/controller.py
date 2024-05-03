#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# Import ROS messages
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Import Classes
from .rover import Rover

class Controller(Rover):
    def __init__(self) -> None:
        # Initialize the rover attributes
        Rover.__init__(self)

        # Initialize variables
        self.__vmax, self.__wmax = 0.1, 0.3
        self.__kp = np.eye(2)
        self.__kd = 0.6
        self.__kr = 2.5
        self.__point = {"x":1.0, "y":0.0}

        # Lidar data info
        self.__forward, self.__left, self.__right = [], [], []

        # Flags for rotating
        self.__turn_right = False
        self.__turning = True

        # Declare the publish messagess
        self.__vel = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to the odometry and set_point topics
        rospy.Subscriber("/odom", Odometry, self.__odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.wait_for_message("/odom", Odometry, timeout = 30)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    # Callback function for the odometry
    def __odom_callback(self, msg:Odometry) -> None:
        # Get position
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        # Get orientation
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    # Callback function for the lidar data
    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[275:350]
        self.__right = msg.ranges[800:925]

    def control(self) -> None:
        # Get the error in position
        dx = self.__point["x"] - self._states["x"]
        dy = self.__point["y"] - self._states["y"]
        dist = np.sqrt(dx**2 + dy**2)

        # Get the error in angle
        thetad = np.arctan2(dy, dx)
        thetae = self._wrap_to_Pi(thetad - self._states["theta"])

        # Get control input
        self._v = self.__vmax*np.tanh(dist / self.__vmax) if dist > 0.03 else 0.0
        self._w = self.__wmax*np.tanh(thetae / self.__wmax) if abs(thetae) > 0.03 else 0.0

        # # Setup vectors
        # q = np.array([self._states["x"], self._states["y"]])
        # qd = np.array([self.__point["x"], self.__point["y"]])
        
        # # Control
        # err = qd - q
        # D = np.array([
        #     [(self._r / 2 * np.cos(self._states["theta"]) - self._h*self._r / self._l * np.sin(self._states["theta"])),
        #      (self._r / 2 * np.cos(self._states["theta"]) + self._h*self._r / self._l * np.sin(self._states["theta"]))],
        #     [(self._r / 2 * np.sin(self._states["theta"]) + self._h*self._r / self._l * np.cos(self._states["theta"])), 
        #      (self._r / 2 * np.sin(self._states["theta"]) - self._h*self._r / self._l * np.cos(self._states["theta"]))]
        # ])
        # u = np.dot(np.linalg.inv(D), np.dot(self.__kp, err))
        # q_dot = np.dot(self.__kp, err)
        # theta_dot = np.dot(np.array([[self._r / self._l, -self._r / self._l]]), u)

        # # Check if the robot has reached the set point
        # if np.linalg.norm(err) < 0.01:
        #     self._v = 0.0
        #     self._w = 0.0
        # else:
        #     # Get linear and angular velocities
        #     self._v = self.__vmax*np.tanh(np.sqrt(q_dot[0]**2 + q_dot[1]**2) / self.__vmax)
        #     self._w = self.__wmax*np.tanh(theta_dot[0] / self.__wmax)

        # Avoid obstacles
        self.__avoid()

    def __avoid(self) -> None:
        # Minimum distance from obstacles at each direction
        min_forward, min_left, min_right = [min(self.__forward), min(self.__left), min(self.__right)]
        if all(dist < self._safe_distance for dist in [min_forward, min_left, min_right]):
            self._v, self._w = 0.0, self.__wmax
        elif min_left < self._safe_distance:
            self._w -= self.__kr*(self._safe_distance - min_left)
        elif min_right < self._safe_distance:
            self._w += self.__kr*(self._safe_distance - min_right)
        elif min_forward < self._safe_distance:
            self._v -= self.__kd*(self._safe_distance - min_forward)
            self._w += self.__obstacle_forward(min_forward, min_left, min_right)
            
        # Publish the control input
        self.__vel.linear.x = self._v
        self.__vel.angular.z = self._w
        self.__vel_pub.publish(self.__vel)

    def __obstacle_forward(self, forward:float, left:float, right:float) -> float:
        if self.__turning:
            self.__turn_right = True if right >= left else False
            self.__turning = False
        
        # Rotate to the direction with the higher distance
        return -self.__kr*(self._safe_distance - forward) if self.__turn_right else self.__kr*(self._safe_distance - forward)
    
    def stop(self) -> None:
        # Stop the rover
        self.__vel.linear.x = 0.0
        self.__vel.angular.z = 0.0
        self.__vel_pub.publish(self.__vel)
        rospy.loginfo("Stopping controller")