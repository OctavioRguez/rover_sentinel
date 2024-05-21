#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# ROS messages
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, Path

# Parent Class
from .rover import Rover

class Controller(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)
        self.__vmax, self.__wmax = 0.15, 0.3
        self.__kpd = 1.25
        self.__kpr = 7.0

        self.__path = []
        self.__point = 0

        # Lidar data
        self.__forward, self.__left, self.__right = [], [], []
        self.__dist = float("inf")
        self.__turn_right = False
        self.__turning = True

        self.__vel = Twist()

        self.__kalman_pub = rospy.Publisher("/kalman_predict/vel", Twist, queue_size = 10)
        rospy.Subscriber("/kalman_predict/pose/controller", Pose, self.__kalman_callback)

        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/odom/raw", Odometry, self.__odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.Subscriber("/sensor/distance", Float32, self.__distance_callback)
        rospy.Subscriber("/path", Path, self.__path_callback)
        rospy.wait_for_message("/odom/raw", Odometry, timeout = 30)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)
        rospy.wait_for_message("/sensor/distance", Float32, timeout = 30)
        rospy.wait_for_message("/path", Path, timeout = 30)

    def __kalman_callback(self, msg:Pose) -> None:
        q = msg.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]        
        v, w = self.__control_velocity(msg.position.x, msg.position.y, theta)
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        self.__kalman_pub.publish(vel)

    def __odom_callback(self, msg:Odometry) -> None:
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[144:430]
        self.__right = msg.ranges[717:1004]
    
    def __distance_callback(self, msg:Float32) -> None:
        self.__dist = msg.data

    def __path_callback(self, msg:Path) -> None:
        self.__path = msg.poses

    def control(self) -> None:
        v, w = self.__control_velocity(self._states["x"], self._states["y"], self._states["theta"])
        self.__vel.linear.x = v
        self.__vel.angular.z = w
        self.__vel_pub.publish(self.__vel)

    def __control_velocity(self, x:float, y:float, theta:float) -> tuple:
        v, w = 0.0, 0.0
        if self.__point >= len(self.__path):
            return
        
        dx = self.__path[self.__point].pose.position.x - x
        dy = self.__path[self.__point].pose.position.y - y
        dist = np.sqrt(dx**2 + dy**2)

        thetad = np.arctan2(dy, dx)
        thetae = self._wrap_to_Pi(thetad - theta)

        v = self.__vmax*np.tanh(dist / self.__vmax) if dist > 0.03 else 0.0
        w = self.__wmax*np.tanh(thetae / self.__wmax) if abs(thetae) > 0.03 else 0.0
        self.__point = self.__point + 1 if (dist < 0.03 and abs(thetae) < 0.03) else self.__point
        v, w = self.__avoid(v, w)
        return v, w

    def __avoid(self, v:float, w:float) -> None:
        # Minimum distance from obstacles at each direction
        min_forward, min_left, min_right = [min(self.__dist, min(self.__forward)), min(self.__left), min(self.__right)]
        if all(dist < self._safe_distance for dist in [min_forward, min_left, min_right]):
            v, w = 0.0, self.__wmax
        elif min_left < self._safe_distance:
            w -= self.__kpr*(self._safe_distance - min_left)
        elif min_right < self._safe_distance:
            w += self.__kpr*(self._safe_distance - min_right)
        elif min_forward < self._safe_distance:
            v -= self.__kpd*(self._safe_distance - min_forward)
            w += self.__kpr*self.__obstacle_forward(min_forward, min_left, min_right)
        else:
            self.__turning = True
        return v, self.__wmax*np.tanh(w / self.__wmax)

    def __obstacle_forward(self, forward:float, left:float, right:float) -> float:
        if self.__turning:
            self.__turn_right = True if right >= left else False
            self.__turning = False
        # Rotate to the direction with the higher distance
        return -(self._safe_distance - forward) if self.__turn_right else (self._safe_distance - forward)

    def rotate(self) -> None:
        self.__vel.linear.x = 0.0
        self.__vel.angular.z = self.__wmax
        self.__vel_pub.publish(self.__vel)

    def stop(self) -> None:
        self.__vel.linear.x = 0.0
        self.__vel.angular.z = 0.0
        self.__vel_pub.publish(self.__vel)
        rospy.loginfo("Stopping controller")
