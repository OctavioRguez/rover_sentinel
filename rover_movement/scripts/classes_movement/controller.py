#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

# ROS messages
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry, Path

# Parent Class
from .rover import Rover

class Controller(Rover):
    def __init__(self) -> None:
        Rover.__init__(self)
        self.__vmax, self.__wmax = 0.1, 0.15
        self.__kpr = 2.0

        self.__enable = False
        self.__reactive = False
        self.__w_past, self.__w_past_kalman = 0.0, 0.0

        self.__path = []
        self.__point = 0

        # Lidar data
        self.__forward, self.__left, self.__right = [], [], []
        self.__dist = float("inf")
        self.__ir_left = float("inf")
        self.__ir_right = float("inf")
        
        self.__turning, self.__turning_kalman = True, True
        self.__avoid_turning, self.__avoid_turning_kalman = True, True
        self.__left_time = rospy.Time.now().to_sec()
        self.__right_time = rospy.Time.now().to_sec()

        self.__vel = Twist()
        self.__vel_kalman = Twist()

        self.__kalman_pub = rospy.Publisher("/kalman_predict/vel", Twist, queue_size = 1)
        rospy.Subscriber("/kalman_predict/pose/controller", Pose, self.__kalman_callback)

        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.__ready_pub = rospy.Publisher("/ready/control", Bool, queue_size = 10)
        rospy.Subscriber("/enable/control", Bool, self.__enable_callback)
        rospy.Subscriber("/odom/kalman", Odometry, self.__odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.__lidar_callback)
        rospy.Subscriber("/path", Path, self.__path__callback)
        rospy.Subscriber("/sensor/distance", Float32, self.__distance_callback)
        rospy.Subscriber("/sensor/ir/left", Bool, self.__ir_left_callback)
        rospy.Subscriber("/sensor/ir/right", Bool, self.__ir_right_callback)
        rospy.wait_for_message("/scan", LaserScan, timeout = 30)

    def __kalman_callback(self, msg:Pose) -> None:
        if self.__enable and self.__path:
            if not self.__reactive:
                q = msg.orientation
                theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]   
                v, w, self.__avoid_turning_kalman = self.__control_velocity(msg.position.x, msg.position.y, theta, self.__avoid_turning_kalman)
            else:
                v, w, self.__turning_kalman = self.__reactive_navegation(self.__turning_kalman)
                w = self.__w_past_kalman if w is None else w
                self.__w_past_kalman = w
            self.__vel_kalman.linear.x = v
            self.__vel_kalman.angular.z = w
            self.__kalman_pub.publish(self.__vel_kalman)

    def __enable_callback(self, msg:Bool) -> None:
        self.__enable = msg.data

    def __odom_callback(self, msg:Odometry) -> None:
        self._states["x"] = msg.pose.pose.position.x 
        self._states["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._states["theta"] = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def __lidar_callback(self, msg:LaserScan) -> None:
        self.__forward = msg.ranges[0:144] + msg.ranges[1004:1147]
        self.__left = msg.ranges[144:430]
        self.__right = msg.ranges[717:1004]

    def __path__callback(self, msg:Path) -> None:
        self.__path = msg.poses
        self.__point = 0
        self.__ready_pub.publish(False)

    def __distance_callback(self, msg:Float32) -> None:
        self.__dist = msg.data

    # Transform the digital signal of the IR sensor, according to its threshold (17 cm)
    def __ir_left_callback(self, msg:Bool) -> None:
        if (rospy.Time.now().to_sec() - self.__left_time > 0.5) or msg.data:
            self.__ir_left = 0.12 if msg.data else float("inf")
            self.__left_time = rospy.Time.now().to_sec()

    def __ir_right_callback(self, msg:Bool) -> None:
        if (rospy.Time.now().to_sec() - self.__right_time > 0.5) or msg.data:
            self.__ir_right = 0.09 if msg.data else float("inf")
            self.__right_time = rospy.Time.now().to_sec()

    def control(self) -> None:
        if self.__enable and self.__path:
            if not self.__reactive:
                v, w, self.__avoid_turning = self.__control_velocity(self._states["x"], self._states["y"], self._states["theta"], self.__avoid_turning)
            else:
                v, w, self.__turning = self.__reactive_navegation(self.__turning)
                w = self.__w_past if w is None else w
                self.__w_past = w
            self.__vel.linear.x = v
            self.__vel.angular.z = w
            self.__vel_pub.publish(self.__vel)

    def __control_velocity(self, x:float, y:float, theta:float, turning:bool) -> tuple:
        v, w = 0.0, 0.0
        if self.__point >= len(self.__path):
            self.__ready_pub.publish(True)
            return v, w, turning
    
        dx = self.__path[self.__point].pose.position.x - x
        dy = self.__path[self.__point].pose.position.y - y
        dist = np.sqrt(dx**2 + dy**2)

        thetad = np.arctan2(dy, dx)
        thetae = self._wrap_to_Pi(thetad - theta)

        v = self.__vmax*np.tanh(dist / self.__vmax) if (dist > 0.03 and abs(thetae) <= 0.1) else 0.0
        w = self.__wmax*np.tanh(thetae / self.__wmax) if abs(thetae) > 0.03 else 0.0
        self.__point = self.__point + 1 if (dist <= 0.15) else self.__point
        v, w, turning = self.__avoid(v, w, turning)
        return v, w, turning

    def __avoid(self, v:float, w:float, turning:bool) -> tuple:
        # Minimum distance from obstacles at each direction
        self.__left = self.__left + self.__forward[50:144]
        self.__right = self.__right + self.__forward[144:154]
        self.__forward = self.__forward[:50] + self.__forward[154:]

        min_forward = min(min(self.__forward), self.__dist)
        min_left = min(min(self.__left), self.__ir_left)
        min_right = min(min(self.__right), self.__ir_right)

        if all(dist < (self._safe_distance - 0.1) for dist in [min_forward, min_left, min_right]):
            v, w = 0.0, self.__wmax
        elif min_forward < self._safe_distance - 0.08:
            self.__reactive = True
            turning = True
        elif turning:
            turning = False
            if min_left < self._safe_distance:
                w = self.__kpr*(self._safe_distance - min_left)
            elif min_right < self._safe_distance:
                w = self.__kpr*(self._safe_distance - min_right)
        return v, self.__wmax*np.tanh(w / self.__wmax), turning

    def __reactive_navegation(self, turning:bool) -> tuple:
        min_forward = min(min(self.__forward), self.__dist, self.__ir_left, self.__ir_right)
        min_left = min(min(self.__left), self.__ir_left)
        min_right = min(min(self.__right), self.__ir_right)
        
        v, w = 0.0, None
        # No obstacles in any direction
        if all(dist > self._safe_distance for dist in [min_forward, min_left, min_right]):
            self.__reactive = False
        elif min_forward > self._safe_distance:
            turning = True
            v, w = self.__vmax, 0.0
        elif turning:
            turning = False
            # Obstacles in all directions
            if all(dist < self._safe_distance for dist in [min_forward, min_left, min_right]):
                w = -self.__wmax
            # Obstacles at the left
            elif min_left < self._safe_distance:
                w = -self.__wmax
            # Obstacles at the right
            elif min_right < self._safe_distance:
                w =  self.__wmax
            # Obstacles at the front
            else:
                # Rotate to the direction with the higher distance
                w = -self.__wmax if min_right >= min_left else self.__wmax
        return v, w, turning

    def rotate(self) -> None:
        self.__vel.linear.x = 0.0
        self.__vel.angular.z = self.__wmax
        self.__vel_pub.publish(self.__vel)

    def stop(self) -> None:
        self.__vel.linear.x = 0.0
        self.__vel.angular.z = 0.0
        self.__vel_pub.publish(self.__vel)
        rospy.loginfo("Stopping controller")
