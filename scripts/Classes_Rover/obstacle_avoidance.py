#!/usr/bin/python

# Import python libraries
import rospy

# Import ROS messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Class to avoid obstacles
class obstacleAvoidance:
    def __init__(self):
        # Count to avoid jerking
        self.__count1, self.__count2 = 0, 0
        # Safe distance from obstacles at any direction (m)
        self.__safeDistance = rospy.get_param('/safe_distance', default = 0.3)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self.__v, self.__w = 0.0, 0.0
        self.__vmax, self.__wmax = 0.5, 1.0

        # Publish messages
        self.__velocity = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to wheel encoder topics
        rospy.Subscriber("/scan", LaserScan, self.__scanCallback)

    # Stop function
    def _stop(self) -> None:
        self.__v, self.__w = 0.0, 0.0
        self.publishVel()

    # Private function for the callback of the laser scan
    def __scanCallback(self, msg):
        self.__avoid(msg.ranges)
        self.publishVel()

    # Private function for turning right of left
    def __rotate(self, dists, left, right):
        if left > right:
            self.__count1 += 1
            self.__count2 = 0
            # Turn is taken after 20 counts, to avoid jerking
            if self.__count1 >= 20:
                self.__minLinear_maxAngular()
                if all(dist > self.__safeDistance for dist in dists):
                    self.__count1 = 0
                    self.__maxLinear_minAngular()
        elif left < right :
            self.__count1 = 0
            self.__count2 += 1
            # Turn is taken after 20 counts, to avoid jerking
            if self.__count2 >= 20:
                self.__minLinear_maxAngular(sign = -1)
                if all(dist > self.__safeDistance for dist in dists):
                    self.__count2 = 0
                    self.__maxLinear_minAngular()

    # Public function for avoiding obstacles by changing linear and angular velocities
    def __avoid(self, scanData):
        # Minimum distance from obstacles at each direction
        forwardDist = min(scanData[0:144] + scanData[1004:1147])
        rightDist = min(scanData[861:1004])
        leftDist = min(scanData[144:287])
        dists = [forwardDist, leftDist, rightDist]

        # Prioritize right rotation
        if rightDist > self.__safeDistance:
            self.__minLinear_maxAngular(sign = -1)
        # Check if there are no obstacles at any direction
        elif all(dist > self.__safeDistance for dist in dists) or forwardDist > self.__safeDistance:
            self.__maxLinear_minAngular()
        # Check if there are obstacles in all directions
        elif all(dist < self.__safeDistance for dist in dists):
            self.__minLinear_maxAngular()
        else:
            self.__rotate(dists, leftDist, rightDist)

    # Private function for setting linear at max and angular at min
    def __maxLinear_minAngular(self) :
        self.__v = self.__vmax
        self.__w = 0.0

    # Private function for setting linear at min and angular at max
    def __minLinear_maxAngular(self, sign = 1):
        self.__v = 0.0
        self.__w = sign * self.__wmax

    # Public function for publishing the velocity
    def publishVel(self):
        self.__velocity.linear.x = self.__v
        self.__velocity.angular.z = self.__w
        self.__vel_pub.publish(self.__velocity)
