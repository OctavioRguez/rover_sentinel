#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

class Rover:
    def __init__(self) -> None:
        # Initial rover parameters
        self._l = rospy.get_param("/rover/wheelbase/value", default = 0.2)
        self._r = rospy.get_param("/rover/wheel_radius/value", default = 0.05)
        self._h = rospy.get_param("/rover/hinge/value", default = 0.01)
        self._safe_distance = rospy.get_param("/rover/safe_distance/value", default = 0.3)
        self._last_time = 0.0

        # Rover states
        self._states = {"x":0.0, "y":0.0, "theta":0.0}
        self._v, self._w = 0.0, 0.0

    # Get the time difference for dt
    def _get_dt(self) -> float:
        current_time = rospy.Time.now().to_sec()
        dt = (current_time - self._last_time)
        self._last_time = current_time
        return dt

    # Wrap to pi function
    def _wrap_to_Pi(self, theta:float) -> float:
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi
    
    # Stop function for ROS
    def stop(self) -> None:
        rospy.loginfo(f"Stopping {self.__class__.__name__}")