#!/usr/bin/python3

# Import python libraries
import rospy

# Import ROS messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class obstacleAvoidance:
    def __init__(self):
        # Count to avoid jerking
        self.__count1, self.__count2 = 0, 0
        # Safe distance from obstacles at any direction (m)
        self.__safeDistance = rospy.get_param('/puzzlebot/safe_distance/value', default = 0.3)

        # Linear (v) and angular (w) velocities (m/s, rad/s)
        self.__v, self.__w = 0.0, 0.0
        self.__vmax, self.__wmax = 0.25, 0.3

        # Publish messages
        self.__velocity = Twist()

        # Publisher for cmd_vel
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        # Subscribe to wheel encoder topics
        rospy.Subscriber("/scan", LaserScan, self.__scanCallback)

    # Stop function
    def _stop(self):
        self.__v, self.__w = 0.0, 0.0
        self.publishVel()

    # Private function for the callback of the laser scan
    def __scanCallback(self, msg):
        self.__avoid(msg.ranges)
        self.publishVel()

    def __getDistances(self, data):
        forwardDist = min(data[0:144] + data[1004:1147])
        rightDist = min(data[800:925])
        leftDist = min(data[275:350])
        # rospy.loginfo(forwardDist)
        return [forwardDist, leftDist, rightDist]

    # Private function for turning right of left
    def __rotate(self, left, right):
        if right < self.__safeDistance:
            self.__count1 += 1
            self.__count2 = 0
            # Turn is taken after 2 counts, to avoid jerking
            if self.__count1 >= 2:
                self.__minLinear_maxAngular()
        elif left < self.__safeDistance:
            self.__count1 = 0
            self.__count2 += 1
            # Turn is taken after 2 counts, to avoid jerking
            if self.__count2 >= 2:
                self.__minLinear_maxAngular(sign = -1)
        else:
            self.__v = 0.0
            self.__w = 0.0

    # Public function for avoiding obstacles by changing linear and angular velocities
    def __avoid(self, scanData):
        # Minimum distance from obstacles at each direction
        dists = self.__getDistances(scanData)
        forwardDist, leftDist, rightDist = dists

        # Check if there are no obstacles at the front
        if forwardDist < self.__safeDistance:
            self.__count1, self.__count2 = 0, 0
            self.__minLinear_maxAngular(sign=-1)
        # Check if there are obstacles in all directions
        elif all(dist < self.__safeDistance for dist in dists):
            self.__count1, self.__count2 = 0, 0
            self.__minLinear_maxAngular(sign=-1)
        else:
            self.__rotate(leftDist, rightDist)

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

# 0:300 -> 0, 100
# 300:600 -> 100, 200
# 600:900 -> 200, 290
# 900:1147 -> 290, 360